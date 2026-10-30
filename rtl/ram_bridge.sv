`timescale 1ns/1ps

// ram_bridge -- Amstrad CPC PSRAM bridge, dynamic-phase calibration build (v3).
//
// Two INDEPENDENT APS6404L QSPI chips, both calibrated at boot by ONE shared
// psram_cal3 over one shared capture clock:
//
//   chip 1 (U1): CPU / loader / tape RAM. Full {bank, addr[20:0]} byte map,
//                byte accesses (wtbt=00). Serves all host reads and writes.
//   chip 2 (U2): shadow of the base-64 KB RAM (the byte range the CRTC's
//                VRAM fetches live in). Serves video only, as 16-bit word
//                reads (wtbt=11), plus byte-for-byte mirrors of the CPU's
//                base-window writes.
//
// Address rules (byte addresses in the 8 MB PSRAM space):
//   * host ops with addr[22:16] == 7'd2 ("base window"): mapped to
//     {2'b00, addr[20:0]}, i.e. 0x020000..0x02FFFF ALWAYS -- a single shared
//     base-64K, exactly like the base_ram BRAM it replaces (the old design's
//     per-bank PSRAM copies in that window were written but never read).
//   * all other host ops go to chip 1 at {bank, addr[20:0]} and to nothing
//     else (bank = the per-model 2 MB region the old design used).
//   * video: chip 2 word reads at {2'b00, vram_addr[20:0]} (vram_addr is fed
//     as {2'b10, crtc_addr, 1'b0}: inside the base window, word addressed).
//
// Video holding buffer: vram_dout holds the last complete 16-bit word; a new
// fetch is queued whenever the live CRTC address moves off it, when the
// buffer is empty, when boot_load ends, or when a mirrored write changed the
// displayed word. No BRAM involved: PSRAM only.
//
// Issue path: no req/ack toggle -- the scheduler presents rd/we plus address
// and data while the psram controller reports ready, pulse-style, then waits
// for ready to fall and rise again (that rise is the result-assembly edge,
// see psram.sv ST_RDY). Host requests land in small FIFOs so no loader write
// can be dropped while an op is in flight (the old single-slot design did
// drop them).
//
// Ownership of every register lives in exactly one always block:
//   * host capture block: edge capture, FIFO pushes/pointers, tape slot,
//     cpu_rd_pending (cleared via the engine's rd_fin pulse), dout mux input
//   * chip1 engine: its strobes + ops, tape acks (via a pulse reg), results
//   * chip2 engine: its strobes + ops, vram tracking regs
//
// Calibration: psram_cal3 owns both psram host ports while `busy` and leaves
// (c*, s*) as cap_c for both controllers; cal_done/fail/status are exported.
// The capture clock (clk_cap) and the DPS phase ports of the PLL that runs it
// come from the top level; psram_dps (inside) drives them. DPS_CNTL must be
// the logical counter of the capture output (PLL outclk_2 -> C2) and
// DPS_DIRUP must give "later phase"; both are covered by the calibration
// self-test bits in cal_status (det_tog / step timeout).

module ram_bridge #(
    parameter integer S          = 20,          // phase steps per HALF period: 4*Fvco/Fout (640/128 -> 20)
    parameter [22:0]  CAL_BASE   = 23'h2F0000,  // cal scratch: unreachable from any host mapping
    parameter [4:0]   DPS_CNTL   = 5'd2,        // logical counter of the capture output (C2 = outclk_2)
    parameter         DPS_DIRUP  = 1'b1,        // updn value that means "later phase"
    parameter integer OP_TMO     = 16384,       // clk cycles before an op is abandoned
    parameter integer DEPTH      = 4,           // host request FIFO depth
    // psram power-up chain + calibration workload (board values; sim overrides)
    parameter integer PWRUP_CYCLES = 30000,
    parameter integer RST_CYCLES   = 4000,
    parameter integer MID_CYCLES   = 100,
    parameter integer CAL_WORDS   = 16,
    parameter integer CAL_PASSES  = 4
) (
    // ===== pads, chip 1 (U1): CPU RAM, LOANIO 33/19/17/34, CLK 54, CE# 48 =====
    output              PSRAM_SCLK,
    output              PSRAM_CE_N,
    output [3:0]        PSRAM_SIO_OUT,
    output [3:0]        PSRAM_SIO_OE,
    input  [3:0]        PSRAM_SIO_IN,

    // ===== pads, chip 2 (U2): base-64K shadow / video, LOANIO 29/27/53/30, CLK 28, CE# 23 =====
    output              PSRAM2_SCLK,
    output              PSRAM2_CE_N,
    output [3:0]        PSRAM2_SIO_OUT,
    output [3:0]        PSRAM2_SIO_OE,
    input  [3:0]        PSRAM2_SIO_IN,

    // ===== clocks / calibration plumbing =====
    input               init,        // held while the PLL is not locked
    input               clk,         // 128 MHz; fabric clock AND both SCLK pads
    input               clk_cap,     // PLL outclk_2: capture clock (phase-stepped)
    input               scanclk,     // 50 MHz PLL reference (DPS scanclk domain)
    input               pll_phase_done,
    output              pll_phase_en,
    output              pll_updn,
    output [4:0]        pll_cntsel,
    output              cal_done,
    output              cal_fail,
    output              cal_busy,
    output [63:0]       cal_status,

    // ===== cpu/chipset interface (clk domain, 128 MHz) =====
    input  [1:0]        bank,
    input  [7:0]        din,
    output [7:0]        dout,
    input  [22:0]       addr,
    input               oe,
    input               we,

    output [15:0]       vram_dout,
    input  [22:0]       vram_addr,   // {2'b10, crtc_addr, 1'b0} from the top level
    input  [1:0]        vram_bank,   // unused: video always maps to the shared base window

    input  [22:0]       tape_addr,
    input  [7:0]        tape_din,
    output reg [7:0]    tape_dout,
    input               tape_wr,
    output              tape_wr_ack,
    input               tape_rd,
    output reg          tape_rd_ack,
    input               boot_load,   // loader streaming: video fetches suppressed

    // high unless a CPU read is outstanding (not consumed by the core today,
    // kept for compatibility with the old port list)
    output              rdy
);

    assign PSRAM_SCLK  = clk;
    assign PSRAM2_SCLK = clk;

    wire rst_n = ~init;

    // ======================================================================
    // psram controllers (one per chip)
    // ======================================================================
    wire        p1_ready, p2_ready;
    wire [15:0] p1_dout, p2_dout;
    reg  [22:0] p1_addr, p2_addr;
    reg  [1:0]  p1_wtbt, p2_wtbt;
    reg  [15:0] p1_din,  p2_din;
    reg         p1_rd, p1_we, p2_rd, p2_we;

    // calibration host traffic (addr/wtbt/din are shared between the chips,
    // rd/we are per chip)
    wire [22:0] cal_addr;
    wire [1:0]  cal_wtbt;
    wire [15:0] cal_din;
    wire        cal_rd1, cal_we1, cal_rd2, cal_we2;
    wire [2:0]  cal_c;

    psram #(
        .PWRUP_CYCLES(PWRUP_CYCLES),
        .RST_CYCLES(RST_CYCLES),
        .MID_CYCLES(MID_CYCLES),
        .LEAN(1)      // stage 2: lean drain (pick-only tap, depth 2+c/2)
    ) psram1 (
        .clk    (clk),
        .reset  (~rst_n),
        .init   (init),

        .ce_n   (PSRAM_CE_N),
        .sio_out(PSRAM_SIO_OUT),
        .sio_oe (PSRAM_SIO_OE),
        .sio_in (PSRAM_SIO_IN),

        .clk_cap(clk_cap),
        .cap_sel(cal_c),

        .ready  (p1_ready),
        .addr   (cal_busy ? cal_addr : p1_addr),
        .wtbt   (cal_busy ? cal_wtbt : p1_wtbt),
        .rd     (cal_busy ? cal_rd1  : p1_rd),
        .we     (cal_busy ? cal_we1  : p1_we),
        .din    (cal_busy ? cal_din  : p1_din),
        .dout   (p1_dout)
    );

    psram #(
        .PWRUP_CYCLES(PWRUP_CYCLES),
        .RST_CYCLES(RST_CYCLES),
        .MID_CYCLES(MID_CYCLES),
        .LEAN(1)      // stage 2: lean drain (pick-only tap, depth 2+c/2)
    ) psram2 (
        .clk    (clk),
        .reset  (~rst_n),
        .init   (init),

        .ce_n   (PSRAM2_CE_N),
        .sio_out(PSRAM2_SIO_OUT),
        .sio_oe (PSRAM2_SIO_OE),
        .sio_in (PSRAM2_SIO_IN),

        .clk_cap(clk_cap),
        .cap_sel(cal_c),

        .ready  (p2_ready),
        .addr   (cal_busy ? cal_addr : p2_addr),
        .wtbt   (cal_busy ? cal_wtbt : p2_wtbt),
        .rd     (cal_busy ? cal_rd2  : p2_rd),
        .we     (cal_busy ? cal_we2  : p2_we),
        .din    (cal_busy ? cal_din  : p2_din),
        .dout   (p2_dout)
    );

    // ======================================================================
    // shared calibration (runs at every init pulse, i.e. every PLL lock)
    // ======================================================================
    wire        dps_phase_en, dps_updn, dps_phase_done;
    wire [4:0]  dps_cntsel;
    wire        dps_req_tgl, dps_req_up, dps_done_tgl, dps_err;

    reg clk_det = 1'b0;
    always @(posedge clk_cap) clk_det <= clk;   // capture-clock self-test

    psram_cal3 #(
        .BASE_ADDR(CAL_BASE),
        .N_WORDS(CAL_WORDS),
        .PASSES(CAL_PASSES),
        .S(S),
        .DEF_C(3'd3)
    ) cal (
        .clk (clk),
        .rst_n(rst_n),
        .recal(1'b0),

        .ready1(p1_ready), .dout1(p1_dout), .rd1(cal_rd1), .we1(cal_we1),
        .ready2(p2_ready), .dout2(p2_dout), .rd2(cal_rd2), .we2(cal_we2),
        .addr (cal_addr), .wtbt(cal_wtbt), .din(cal_din),
        .cap_c(cal_c),

        .dps_req_tgl(dps_req_tgl), .dps_req_up(dps_req_up),
        .dps_done_tgl(dps_done_tgl), .dps_err(dps_err),
        .clk_det(clk_det),

        .busy (cal_busy),
        .done (cal_done),
        .fail (cal_fail),
        .status(cal_status)
    );

    psram_dps #(
        .CNTSEL(DPS_CNTL),
        .DIR_UP(DPS_DIRUP)
    ) dps (
        .scanclk (scanclk),
        .rst_n   (rst_n),
        .req_tgl (dps_req_tgl),
        .req_up  (dps_req_up),
        .done_tgl(dps_done_tgl),
        .err     (dps_err),
        .phase_en(dps_phase_en),
        .updn    (dps_updn),
        .cntsel  (dps_cntsel),
        .phase_done(dps_phase_done)
    );

    assign pll_phase_en   = dps_phase_en;
    assign pll_updn       = dps_updn;
    assign pll_cntsel     = dps_cntsel;
    assign dps_phase_done = pll_phase_done;   // PLL -> psram_dps (input!)

    // ======================================================================
    // host capture block (sole owner of: edge regs, FIFO pointers, tape slot)
    // ======================================================================
    function automatic [22:0] op_map(input [1:0] bk, input [22:0] a);
        op_map = (a[22:16] == 7'd2) ? {2'b00, a[20:0]}   // base window -> shared 64K
                                    : {bk,    a[20:0]};  // per-model 2 MB region
    endfunction

    // chip1 entry layout: [31]=wr, [30:8]=premapped addr(23), [7:0]=din
    // DEPTH must be 4: the array is indexed with the low 2 bits of a 3-bit
    // wrap-around pointer (difference counts 0..4 unambiguously mod 8)
    reg [31:0] fifo_cpu [0:DEPTH-1];
    reg [2:0]  cpu_wp;
    reg [2:0]  cpu_rp;
    wire [2:0] cpu_cnt = cpu_wp - cpu_rp;           // 0..DEPTH with 3-bit wrap
    wire cpu_full  = (cpu_cnt == DEPTH);
    wire cpu_valid = (cpu_cnt != 0);
    wire [31:0] cpu_head = fifo_cpu[cpu_rp[1:0]];

    // chip2 entry layout: [33]=pad0, [32]=video(read), [31]=wr, [30:23]=din8,
    // [22:0]=addr(23); 34 bits total in all three writes below
    reg [33:0] fifo_v [0:DEPTH-1];
    reg [2:0]  v_wp;
    reg [2:0]  v_rp;
    wire [2:0] v_cnt = v_wp - v_rp;
    wire v_full  = (v_cnt == DEPTH);
    wire v_valid = (v_cnt != 0);
    wire [33:0] v_head = fifo_v[v_rp[1:0]];

    wire        tape_act = tape_rd | tape_wr;

    reg  old_oe = 1'b0, old_we = 1'b0;
    reg  [7:0] ram_dout;

    // read completion pulse from the chip1 engine (one clk cycle after the
    // result latches); the consumer of the _old_ value, used to clear
    // cpu_rd_pending deterministically
    wire rd_fin;                 // driven in the engine block

    // cpu_rd_pending: set by read capture, cleared by rd_fin
    reg cpu_rd_pending;

    assign rdy = ~cpu_rd_pending;
    assign dout = (oe | we) ? ram_dout : 8'hFF;

    always @(posedge clk) begin
        if (init) begin
            cpu_wp <= 0;
            v_wp <= 0;
            cpu_rd_pending <= 1'b0;
            old_oe <= 1'b0; old_we <= 1'b0;
        end
        else begin
            old_oe <= oe;
            old_we <= we;

            // cpu_rd_pending: a read edge sets it; engine completion clears it
            if (rd_fin)
                cpu_rd_pending <= 1'b0;
            else if ((~old_oe & oe) & ~we)
                cpu_rd_pending <= 1'b1;

            // capture every CPU/loader edge: CPU requests queue in order,
            // base-window writes are additionally mirrored into the chip-2
            // fifo on the same edge with the same mapped address
            if ((~old_oe & oe) | (~old_we & we)) begin
                if (!cpu_full) begin
                    fifo_cpu[cpu_wp[1:0]] <= {we, op_map(bank, addr), din};
                    cpu_wp <= cpu_wp + 3'd1;
                end
                // a full fifo drops the request (the old single-slot design
                // dropped one too); unreachable at loader/CPU rates

                if (we & (addr[22:16] == 7'd2) & !v_full) begin
                    fifo_v[v_wp[1:0]] <= {1'b0, 1'b0, 1'b1, din, op_map(bank, addr)};
                    v_wp <= v_wp + 3'd1;
                end
            end
        end
    end

    // ======================================================================
    // chip 1 scheduler: CPU fifo (in order) > tape
    // ======================================================================
    localparam [1:0] E_IDLE = 2'd0, E_RDY = 2'd1, E_LOW = 2'd2, E_HIGH = 2'd3;
    reg [1:0]  est;
    reg [31:0] eop;
    reg        e_tape;
    reg [15:0] etmo;
    reg        twr_pl;      // tape write ack pulse
    reg        rd_fin_pl;   // cpu read completion pulse

    assign rd_fin = rd_fin_pl;
    assign tape_wr_ack = twr_pl;

    always @(posedge clk) begin
        if (init) begin
            est <= E_IDLE;
            eop <= 32'd0;
            e_tape <= 1'b0;
            etmo <= 16'd0;
            cpu_rp <= 0;
            twr_pl <= 1'b0;
            rd_fin_pl <= 1'b0;
            ram_dout <= 8'hFF;
            tape_dout <= 8'h00;
            tape_rd_ack <= 1'b0;
            p1_rd <= 1'b0; p1_we <= 1'b0;
            p1_addr <= 23'd0; p1_wtbt <= 2'b00; p1_din <= 16'h00FF;
        end
        else begin
            twr_pl <= 1'b0;      // 1-cycle pulses by default
            rd_fin_pl <= 1'b0;

            case (est)
                E_IDLE: if (~cal_busy) begin
                    if (cpu_valid) begin
                        eop <= cpu_head;
                        e_tape <= 1'b0;
                        cpu_rp <= cpu_rp + 3'd1;
                        est <= E_RDY;
                    end
                    // tape rd/wr are levels held until the ack clears them,
                    // so waiting here for a free slot loses nothing
                    else if (tape_act) begin
                        eop    <= {tape_wr, {2'b11, tape_addr[20:0]}, tape_din};
                        e_tape <= 1'b1;
                        est <= E_RDY;
                    end
                end

                // wait idle, then present the request for one cycle
                E_RDY: if (~cal_busy) begin
                    if (p1_ready) begin
                        p1_rd   <= ~eop[31];
                        p1_we   <= eop[31];
                        p1_addr <= eop[30:8];
                        p1_wtbt <= 2'b00;
                        p1_din  <= {8'h00, eop[7:0]};
                        etmo    <= 16'd0;
                        est     <= E_LOW;
                    end
                    else if (etmo == OP_TMO) est <= E_IDLE; // stuck: drop op
                    else etmo <= etmo + 16'd1;
                end

                // request caught: ready drops 1..2 cycles after the strobe
                E_LOW: begin
                    p1_rd <= 1'b0;
                    p1_we <= 1'b0;
                    if (~cal_busy) begin
                        if (!p1_ready) begin
                            etmo <= 16'd0;
                            est  <= E_HIGH;
                        end
                        else if (etmo == 16'd31) est <= E_IDLE; // strobe missed
                        else etmo <= etmo + 16'd1;
                    end
                end

                // wait for the frame to finish and the result to assemble
                E_HIGH: if (~cal_busy) begin
                    if (p1_ready) begin
                        if (e_tape) begin
                            tape_dout <= p1_dout[7:0];
                            if (eop[31]) twr_pl <= 1'b1;
                            else tape_rd_ack <= ~tape_rd_ack;
                        end
                        else if (!eop[31]) begin
                            ram_dout <= p1_dout[7:0];
                            rd_fin_pl <= 1'b1;
                        end
                        etmo <= 16'd0;
                        est   <= E_IDLE;
                    end
                    else if (etmo == OP_TMO) est <= E_IDLE; // frame lost: drop
                    else etmo <= etmo + 16'd1;
                end

                default: est <= E_IDLE;
            endcase
        end
    end

    // ======================================================================
    // chip 2 scheduler: mirror writes (fifo) + video 16-bit word reads
    // ======================================================================
    reg [1:0]  est2;
    reg [33:0] eop2;
    reg [15:0] etmo2;

    // ---- video word tracking ----
    wire [22:0] vram_live = vram_addr;
    reg  [22:0] vram_out_addr;
    reg  [15:0] vram_buf;
    reg         vram_valid;
    reg         vram_dirty;
    reg         vram_inflight;
    reg         boot_loadq;

    assign vram_dout = vram_buf;
    // a fresh fetch is wanted when the live address moved off the buffered
    // word, the buffer is empty, a mirrored write changed the displayed
    // word, or boot_load has just ended
    wire vram_miss = (vram_live != vram_out_addr) | ~vram_valid | vram_dirty;

    always @(posedge clk) begin
        if (init) begin
            est2 <= E_IDLE;
            eop2 <= 34'd0;
            etmo2 <= 16'd0;
            v_rp <= 0;
            p2_rd <= 1'b0; p2_we <= 1'b0;
            p2_addr <= 23'd0; p2_wtbt <= 2'b00; p2_din <= 16'hFFFF;
            vram_out_addr <= 23'h7FFFFF;
            vram_buf <= 16'h0;
            vram_valid <= 1'b0;
            vram_dirty <= 1'b0;
            vram_inflight <= 1'b0;
            boot_loadq <= 1'b0;
        end
        else begin
            // a loader window starting or ending invalidates the buffered
            // word, so the first post-boot fetch is a fresh one
            if (boot_load != boot_loadq) begin
                boot_loadq    <= boot_load;
                vram_valid    <= 1'b0;
                vram_dirty    <= 1'b0;
                vram_inflight <= 1'b0;
                vram_out_addr <= 23'h7FFFFF;
            end

            case (est2)
                E_IDLE: if (~cal_busy) begin
                    if (v_valid) begin
                        eop2 <= v_head;
                        v_rp <= v_rp + 3'd1;
                        est2 <= E_RDY;
                    end
                    else if (vram_miss & ~boot_load & ~vram_inflight & ~v_full
                             & (vram_addr[22:16] == 7'd2)) begin
                        eop2 <= {1'b0, 1'b1, 1'b0, 8'h00, {2'b00, vram_live[20:0]}};
                        vram_inflight <= 1'b1;
                        est2 <= E_RDY;
                    end
                end

                E_RDY: if (~cal_busy) begin
                    if (p2_ready) begin
                        p2_rd   <= eop2[32];
                        p2_we   <= eop2[31];
                        // video = word read; mirror = 8-bit write
                        p2_wtbt <= eop2[32] ? 2'b11 : 2'b00;
                        p2_addr <= eop2[22:0];
                        p2_din  <= {8'h00, eop2[30:23]};
                        etmo2   <= 16'd0;
                        est2    <= E_LOW;
                    end
                    else if (etmo2 == OP_TMO) est2 <= E_IDLE; // stuck: drop op
                    else etmo2 <= etmo2 + 16'd1;
                end

                E_LOW: begin
                    p2_rd <= 1'b0;
                    p2_we <= 1'b0;
                    if (~cal_busy) begin
                        if (!p2_ready) begin
                            etmo2 <= 16'd0;
                            est2  <= E_HIGH;
                        end
                        else if (etmo2 == 16'd31) est2 <= E_IDLE; // missed
                        else etmo2 <= etmo2 + 16'd1;
                    end
                end

                E_HIGH: if (~cal_busy) begin
                    if (p2_ready) begin
                        if (eop2[32]) begin
                            // video word read landed
                            vram_out_addr <= eop2[22:0];
                            vram_buf      <= p2_dout;
                            vram_valid    <= 1'b1;
                            vram_dirty    <= 1'b0;
                            vram_inflight <= 1'b0;
                        end
                        else if (eop2[22:1] == vram_out_addr[22:1]) begin
                            // mirrored write on the displayed word
                            vram_dirty <= 1'b1;
                        end
                        etmo2 <= 16'd0;
                        est2  <= E_IDLE;
                    end
                    else if (etmo2 == OP_TMO) est2 <= E_IDLE; // frame lost
                    else etmo2 <= etmo2 + 16'd1;
                end

                default: est2 <= E_IDLE;
            endcase
        end
    end

endmodule
