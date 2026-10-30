`timescale 1ns/1ps

// tb_ram_bridge -- ram_bridge x2 chips + psram_cal3 + psram_dps + behavioural
// PLL phase model + real APS6404L models, in the 128 MHz Amstrad configuration.
//
// Exercises:
//   1. boot calibration (shared capture clock, DPS stepping through the sim PLL)
//   2. boot-load streaming writes incl. base-window mirrors (loader rate)
//   3. CPU byte write / read on chip 1 (base window, ROM area, bank 1, tape)
//   4. video word fetches from chip 2 through the holding buffer
//   5. write-to-displayed-word coherence (mirror -> re-read)
//   6. tape write + read
//   7. latency measurements (oe edge -> ram_dout valid, in clk cycles)
//
// Ground truth: the chip models' memory arrays (hierarchical peeks) plus a
// local reference array kept by the tb itself.
//
// Build/run (from the Amstrad_MiSTer directory):
//   mkdir -p sim/vvp
//   iverilog -g2012 -o sim/vvp/tb_ram_bridge.vvp \
//        rtl/ram_bridge.sv rtl/psram.sv rtl/psram_cal3.sv rtl/psram_dps.sv \
//        sim/aps6404l_model.sv sim/tb_ram_bridge.sv
//   vvp sim/vvp/tb_ram_bridge.vvp
//
// VCD=1 adds a waveform dump for debugging (sim/vcd/tb_ram_bridge.vcd).

`ifndef T_NS
  `define T_NS 7.8125          // 128 MHz
`endif
`ifndef TACLK_NS
  `define TACLK_NS 3.5         // mid-real device tACLK
`endif
`ifndef DFWD_NS
  `define DFWD_NS 1.0          // fabric -> pad
`endif
`ifndef DBACK_NS
  `define DBACK_NS 1.0         // pad -> fabric
`endif
`ifndef DSKEW_NS
  `define DSKEW_NS 0.3
`endif
`ifndef CHIP2_EXTRA_NS
  `define CHIP2_EXTRA_NS 0.4   // chip 2 pad path a bit different
`endif
`ifndef PHI0_NS
  `define PHI0_NS 1.1719       // window-start phase, ~0.15 T at 128 MHz
`endif
`ifndef STEP_NS
  `define STEP_NS 0.1953125    // 1/8 VCO (640 MHz); S=20 steps per half period
`endif

// ------------------------------------------------------------------ PLL model
module pll_dps_model #(
    parameter real        PHI0 = `PHI0_NS,
    parameter real        STEP = `STEP_NS,
    parameter [4:0]       CNT  = 5'd2
) (
    input  wire       clk,
    input  wire       scanclk,
    input  wire       phase_en,
    input  wire       updn,
    input  wire [4:0] cntsel,
    output reg        phase_done,
    output wire       clk_cap
);
    real phi;
    reg  cap_r;
    initial begin phi = PHI0; phase_done = 1'b1; cap_r = 1'b0; end
    always @(clk) cap_r <= #(phi) clk;          // transport delay keeps the 50 % duty cycle
    assign clk_cap = cap_r;

    always @(posedge phase_en) begin            // one phase step per low->high transition
        @(posedge scanclk);
        phase_done = 1'b0;
        if (cntsel == CNT) phi = phi + (updn ? STEP : -STEP);
        #25.0;
        phase_done = 1'b1;
    end
endmodule

// ------------------------------------------- one chip's pads + memory model
module ps_pads #(
    parameter real      DSKEW = 0.3,
    parameter real      TACLK = 3.5,
    parameter integer   DEPTH = 1 << 23
) (
    input  wire       sclk_f,
    input  wire       ce_n_f,
    input  wire [3:0] sio_out_f,
    input  wire [3:0] sio_oe_f,
    output wire [3:0] sio_in_b
);
    reg        sclk_d  = 1'b0;
    reg        ce_d    = 1'b1;
    reg [3:0]  sio_fwd = 4'bzzzz;
    tri  [3:0] sio_dev;
    reg [3:0]  sio_back = 4'h0;

    always @(sclk_f)                 sclk_d  <= #(`DFWD_NS)         sclk_f;
    always @(ce_n_f)                 ce_d    <= #(`DFWD_NS + DSKEW) ce_n_f;
    always @(sio_out_f or sio_oe_f)  sio_fwd <= #(`DFWD_NS + DSKEW) ((sio_oe_f != 4'h0) ? sio_out_f : 4'bzzzz);
    assign sio_dev = sio_fwd;
    always @(sio_dev)                sio_back <= #(`DBACK_NS) sio_dev;

    assign sio_in_b = sio_back;

    aps6404l_model #(
        .DEPTH(DEPTH), .TACLK_NS(TACLK), .TOH_NS(0.0), .WAIT_EB(5)
    ) u_mem (.sclk(sclk_d), .ce_n(ce_d), .sio(sio_dev));
endmodule

// ------------------------------------------------------------------- testbench
module tb_ram_bridge;

    reg clk = 1'b0;
    reg clk_50 = 1'b0;
    always #(`T_NS/2)  clk    = ~clk;
    always #10.0       clk_50 = ~clk_50;

    wire clk_cap;

    reg init = 1'b1;
    `ifdef VCD
    initial $dumpfile("sim/vcd/tb_ram_bridge.vcd");
    initial $dumpvars(0, tb_ram_bridge);
    `endif

    // ---------------------------------------------------- host stimulus regs
    reg  [22:0] addr_r  = 23'd0;
    reg  [1:0]  bank_r  = 2'b00;
    reg  [7:0]  cpu_din = 8'h00;
    reg         oe_r    = 1'b0;
    reg         we_r    = 1'b0;
    reg  [22:0] vram_r  = 23'd0;
    reg  [22:0] taddr_r = 23'd0;
    reg  [7:0]  tdi_r   = 8'h00;
    reg         t_wr_r  = 1'b0;
    reg         t_rd_r  = 1'b0;

    wire        p1_sclk, p2_sclk;
    wire        p1_ce_n, p2_ce_n;
    wire [3:0]  p1_sio_out, p1_sio_oe, p2_sio_out, p2_sio_oe;
    wire [3:0]  p1_sio_in, p2_sio_in;
    wire        pll_pen, pll_upd, pll_phdone;
    wire [4:0]  pll_cntsel;

    wire [63:0] cal_status;
    wire        cal_done, cal_fail, cal_busy;

    wire [7:0]  cpu_dout;
    wire [15:0] vram_dout;
    wire [7:0]  tape_dout;
    wire        tape_wr_ack, tape_rd_ack;

    `ifdef DBG
    always @(posedge clk) begin
        if (dut.p1_we === 1'b1 && dut.p1_ready === 1'b1)
            $display("%0t DBG issue WR addr=%h din=%h wtbt=%b fifo_cnt=%0d wp=%0d rp=%0d",
                     $time, dut.p1_addr, dut.p1_din, dut.p1_wtbt, dut.cpu_cnt, dut.cpu_wp, dut.cpu_rp);
        if (dut.p1_rd === 1'b1 && dut.p1_ready === 1'b1)
            $display("%0t DBG issue RD addr=%h fifo_cnt=%0d", $time, dut.p1_addr, dut.cpu_cnt);
        if (dut.p2_we === 1'b1 && dut.p2_ready === 1'b1)
            $display("%0t DBG issue MIRROR addr=%h din=%h", $time, dut.p2_addr, dut.p2_din);
        if (dut.psram1.framer === 1'b1 && dut.psram1.fop === 2'd2 && dut.psram1.fpcnt === 7'd0)
            $display("%0t DBG PS1 WR frame start addr=%h wd0=%h", $time, dut.psram1.raddr, dut.psram1.wd0);
    end
    `endif

    pll_dps_model #(
        .PHI0(`PHI0_NS), .STEP(`STEP_NS), .CNT(5'd2)
    ) u_pllmo (
        .clk(clk), .scanclk(clk_50),
        .phase_en(pll_pen), .updn(pll_upd), .cntsel(pll_cntsel),
        .phase_done(pll_phdone), .clk_cap(clk_cap)
    );

    ram_bridge #(
        .S(20),
        .CAL_BASE(23'h2F0000),
        .OP_TMO(16384),
        .DEPTH(4),
        .PWRUP_CYCLES(2000),
        .RST_CYCLES(400),
        .MID_CYCLES(20),
        .CAL_WORDS(4),
        .CAL_PASSES(1)
    ) dut (
        .init(init),
        .clk(clk),
        .clk_cap(clk_cap),
        .scanclk(clk_50),
        .pll_phase_done(pll_phdone),
        .pll_phase_en(pll_pen),
        .pll_updn(pll_upd),
        .pll_cntsel(pll_cntsel),
        .cal_done(cal_done),
        .cal_fail(cal_fail),
        .cal_busy(cal_busy),
        .cal_status(cal_status),

        // chip 1 (U1) pads
        .PSRAM_SCLK(p1_sclk),
        .PSRAM_CE_N(p1_ce_n),
        .PSRAM_SIO_OUT(p1_sio_out),
        .PSRAM_SIO_OE(p1_sio_oe),
        .PSRAM_SIO_IN(p1_sio_in),

        // chip 2 (U2) pads
        .PSRAM2_SCLK(p2_sclk),
        .PSRAM2_CE_N(p2_ce_n),
        .PSRAM2_SIO_OUT(p2_sio_out),
        .PSRAM2_SIO_OE(p2_sio_oe),
        .PSRAM2_SIO_IN(p2_sio_in),

        .bank(bank_r),
        .din(cpu_din),
        .dout(cpu_dout),
        .addr(addr_r),
        .oe(oe_r),
        .we(we_r),

        .vram_dout(vram_dout),
        .vram_addr(vram_r),

        .tape_addr(taddr_r),
        .tape_din(tdi_r),
        .tape_dout(tape_dout),
        .tape_wr(t_wr_r),
        .tape_wr_ack(tape_wr_ack),
        .tape_rd(t_rd_r),
        .tape_rd_ack(tape_rd_ack)
    );

    ps_pads #(.DSKEW(`DSKEW_NS), .TACLK(`TACLK_NS), .DEPTH(1 << 23)) c1 (
        .sclk_f(p1_sclk), .ce_n_f(p1_ce_n), .sio_out_f(p1_sio_out),
        .sio_oe_f(p1_sio_oe), .sio_in_b(p1_sio_in)
    );
    ps_pads #(.DSKEW(`DSKEW_NS + `CHIP2_EXTRA_NS), .TACLK(`TACLK_NS), .DEPTH(1 << 23)) c2 (
        .sclk_f(p2_sclk), .ce_n_f(p2_ce_n), .sio_out_f(p2_sio_out),
        .sio_oe_f(p2_sio_oe), .sio_in_b(p2_sio_in)
    );

    // ---------------------------------------------------- helpers
    integer errors = 0;
    integer passes = 0;
    reg [7:0] ref_base [0:65535];
    reg [7:0] got;
    reg [15:0] wgot;

    function [7:0] peek1(input [22:0] a);
        peek1 = c1.u_mem.mem[a];
    endfunction
    function [7:0] peek2(input [22:0] a);
        peek2 = c2.u_mem.mem[a];
    endfunction

    task tick(input integer k);
        begin repeat (k) @(posedge clk); end
    endtask

    task chk(input ok, input string msg);
        begin
            if (ok === 1'b1) passes = passes + 1;
            else begin
                errors = errors + 1;
                $display("%0t TB FAIL: %0s", $time, msg);
            end
        end
    endtask

    // host byte write pulse
    task wr8(input [22:0] a, input [1:0] bk, input [7:0] d);
        begin
            @(negedge clk);
            addr_r = a; bank_r = bk; cpu_din = d; we_r = 1; oe_r = 0;
            tick(2);
            @(negedge clk);
            we_r = 0;
        end
    endtask

    // host byte read: oe high, wait for the chip1 engine to pick the read
    // up (est leaves idle) and return to idle (that return is the result
    // assembly edge), sample the result register; latency in cycles from
    // the read edge to the idle return
    task rd8(input [22:0] a, input [1:0] bk, output [7:0] res, output integer lat);
        integer n;
        reg busy_seen;
        begin
            @(negedge clk);
            addr_r = a; bank_r = bk; oe_r = 1; we_r = 0; cpu_din = 8'hFF;
            n = 0;
            busy_seen = 0;
            while (n < 520 && (busy_seen == 0 || dut.est !== 2'd0)) begin
                @(posedge clk); n = n + 1;
                if (dut.est !== 2'd0) busy_seen = 1;
            end
            lat = n;
            res = dut.ram_dout;
            @(negedge clk); oe_r = 0;
            tick(4);
        end
    endtask

    // write turnaround: capture edge -> chip1 engine back to idle
    task wr8lat(input [22:0] a, input [1:0] bk, input [7:0] d, output integer lat);
        integer n;
        begin
            @(negedge clk);
            addr_r = a; bank_r = bk; cpu_din = d; we_r = 1; oe_r = 0;
            tick(2);
            @(negedge clk);
            we_r = 0;
            n = 0;
            while ((dut.est !== 2'd0 || dut.est2 !== 2'd0) && n < 200) begin
                @(posedge clk); n = n + 1;
            end
            lat = n;
        end
    endtask

    // drive a crtc word address and wait for the holding buffer to settle
    task vfetch(input [14:0] W);
        begin
            @(negedge clk);
            vram_r = {2'b10, W, 1'b0};
            tick(70);
        end
    endtask

    // download-style drive with the loader-invisible mirror, throttle-side
    function [15:0] vword(input [14:0] W);
        vword = {ref_base[2*W + 3'b1], ref_base[2*W]};
    endfunction

    // ---------------------------------------------------- body
    integer lat_min = 300, lat_max = 0, lat, i, k;
    integer wlat_min = 300, wlat_max = 0;
    reg [22:0] a;
    reg [14:0] W;

    initial begin
        $display("tb_ram_bridge: start (T=%0.3f ns)", `T_NS);

        for (i = 0; i < 65536; i = i + 1) ref_base[i] = 8'h00;

        // ------------------------------------------- 1) calibration
        tick(200);
        init = 0;
        i = 0;
        while (cal_done !== 1'b1 && i < 4000000) begin #1000; i = i + 1; end
        chk(cal_done === 1'b1, "cal_done never asserted");
        tick(6);
        $display("cal_status=%h  c*=%0d s*=%0d eye=%0d margin=%0d det_tog=%0d fail=%b",
                 cal_status, cal_status[32:30], cal_status[39:33],
                 cal_status[9:0], cal_status[49:40], cal_status[62:55], cal_fail);
        chk(cal_fail === 1'b0, "calibration: no common eye");

        // ------------------------------------------- 2) boot download:
        // 64 word-lane pairs at loader-like pace into the base window plus
        // 8 ROM-window writes (chip 1 only)
        for (k = 0; k < 64; k = k + 1) begin
            i = ((k * 37) & 16'h07FE);              // even base offset
            got = (k * 29 + 7) & 8'hFF;
            wr8(23'h020000 + i, 2'b00, got);
            ref_base[i] = got;
            tick(8);
            got = (k * 67 + 11) & 8'hFF;
            wr8(23'h020000 + i + 1, 2'b00, got);
            ref_base[i + 1] = got;
            tick(30);                               // ~ loader rate
        end
        for (k = 0; k < 8; k = k + 1) begin
            wr8(23'h008000 + k, 2'b00, (~k) & 8'hFF);
            tick(30);
        end
        tick(200);
        tick(40);

        // ------------------------------------------- 3) mirror + memory truth
        tick(600);                                   // flush any in-flight mirrors
        for (k = 0; k < 64; k = k + 1) begin
            i = ((k * 37) & 16'h07FE);
            chk(peek1(23'h020000 + i) === ref_base[i],
                $sformatf("boot write chip1 @%08h: got %h want %h", 23'h020000+i, peek1(23'h020000+i), ref_base[i]));
            chk(peek2(23'h020000 + i) === ref_base[i],
                $sformatf("boot MIRROR chip2 @%08h: got %h want %h", 23'h020000+i, peek2(23'h020000+i), ref_base[i]));
            chk(peek1(23'h020000 + i + 1) === ref_base[i + 1],
                $sformatf("boot odd chip1 @%08h mismatch", 23'h020000+i+1));
            chk(peek2(23'h020000 + i + 1) === ref_base[i + 1],
                $sformatf("boot odd MIRROR chip2 @%08h mismatch", 23'h020000+i+1));
        end
        for (k = 0; k < 8; k = k + 1) begin
            // ROM window is chip-1-only: must NOT reach chip 2
            chk(peek1(23'h008000 + k) === ((~k) & 8'hFF),
                $sformatf("rom write chip1 @%08h", 23'h008000+k));
            chk(peek2(23'h008000 + k) === 8'h00,
                $sformatf("rom write leaked into chip2 @%08h", 23'h008000+k));
        end

        // ------------------------------------------- 4) CPU reads of base + ROM
        for (k = 0; k < 6; k = k + 1) begin
            i = ((k * 37 + 3) & 16'h07FF);
            rd8(23'h020000 + i, 2'b00, got, lat);
            if (lat < lat_min) lat_min = lat;
            if (lat > lat_max) lat_max = lat;
            chk(got === ref_base[i],
                $sformatf("cpu read base @%08h: got %h want %h", 23'h020000+i, got, ref_base[i]));
        end

        // ------------------------------------------- 5) cpu write/read round trips
        // base window again (value changed by a later write)
        for (k = 0; k < 4; k = k + 1) begin
            i = ((k * 731 + 5) & 16'h07FF);
            got = (k * 41 + 1) & 8'hFF;
            wr8(23'h020000 + i, 2'b00, got);
            ref_base[i] = got;
            tick(30);
            rd8(23'h020000 + i, 2'b00, got, lat);
            if (lat < lat_min) lat_min = lat;
            if (lat > lat_max) lat_max = lat;
            chk(got === ref_base[i],
                $sformatf("cpu w/r base @%08h: got %h want %h", 23'h020000+i, got, ref_base[i]));
        end
        // bank 1 (model region on chip 1)
        for (k = 0; k < 4; k = k + 1) begin
            i = (k * 131 + 7);
            got = (k * 53 + 17) & 8'hFF;
            wr8(23'h001000 + i, 2'b01, got);
            tick(30);
            rd8(23'h001000 + i, 2'b01, got, lat);
            chk(peek1(23'h201000 + i) === ((k * 53 + 17) & 8'hFF),
                $sformatf("bank1 write @%08h: mem=%h want %h", 23'h201000+i, peek1(23'h201000+i), (k*53+17)&8'hFF));
            chk(got === ((k * 53 + 17) & 8'hFF),
                $sformatf("bank1 read @%08h: got %h", 23'h201000+i, got));
        end

        // ------------------------------------------- 6) video word fetches
        W = 14'd137;
        for (k = 0; k < 6; k = k + 1) begin
            W = 137 + k * 511;
            vfetch(W);
            wgot = dut.vram_buf;
            chk(wgot === vword(W),
                $sformatf("video word @%h: got %h want %h", {2'b10, W, 1'b0}, wgot, vword(W)));
            chk(vram_dout === dut.vram_buf, $sformatf("vram_dout wire @%h", W));
        end
        chk(dut.vram_out_addr === {2'b00, vram_r[20:0]}, "vram_out_addr not tracking live address");
        chk(dut.vram_valid === 1'b1, "vram buffer not marked valid after a fetch");
        chk(dut.vram_dirty === 1'b0, "vram buffer flagged dirty unexpectedly");

        // ------------------------------------------- 7) write-to-displayed-word coherence
        W = 14'd777;
        vfetch(W);
        got  = 8'hA5;                      // new byte, even lane
        i    = 2 * W;
        wr8(23'h020000 + i, 2'b00, got);
        ref_base[i] = got;
        tick(90);
        chk(peek2(23'h020000 + i) === got, "coherence: mirror write missing on chip2");
        chk(dut.vram_buf === vword(W),
            $sformatf("coherence: displayed word not refreshed: got %h want %h", dut.vram_buf, vword(W)));        // odd lane of the same word
        got = 8'h3C;
        i   = 2 * W + 1;
        wr8(23'h020000 + i, 2'b00, got);
        ref_base[i] = got;
        tick(90);
        chk(dut.vram_buf === vword(W), "coherence: odd-lane write did not refresh the word");

        // ------------------------------------------- 8) read-back of the video-checked bytes
        rd8(23'h020000 + 2*777,     2'b00, got, lat);
        chk(got === 8'hA5, $sformatf("cpu reads back video byte (even): %h", got));
        rd8(23'h020000 + 2*777 + 1, 2'b00, got, lat);
        chk(got === 8'h3C, $sformatf("cpu reads back video byte (odd): %h", got));

        // ------------------------------------------- 9) tape write + read
        taddr_r = 23'h000010; tdi_r = 8'h77; t_wr_r = 1;
        tick(4); t_wr_r = 0;
        i = 0;
        while (tape_wr_ack !== 1'b1 && i < 600) begin @(posedge clk); i = i + 1; end
        chk(tape_wr_ack === 1'b1, "tape write ack never seen");
        tick(100);
        chk(peek1(23'h600010) === 8'h77,
            $sformatf("tape write: mem=%h want 77", peek1(23'h600010)));

        t_rd_r = 1;
        tick(4); t_rd_r = 0;
        i = 0;
        while (tape_dout !== 8'h77 && i < 600) begin @(posedge clk); i = i + 1; end
        chk(tape_dout === 8'h77, $sformatf("tape read: got %h want 77", tape_dout));
        tick(50);

        // ------------------------------------------- 10) write turnaround + summary
        begin
            integer wl;
            wr8lat(23'h021234, 2'b00, 8'h5A, wl);
            if (wl < wlat_min) wlat_min = wl;
            if (wl > wlat_max) wlat_max = wl;
            got = 8'h5A; ref_base[16'h1234] = got;
            tick(30);
            wr8lat(23'h021235, 2'b00, 8'hA6, wl);
            if (wl < wlat_min) wlat_min = wl;
            if (wl > wlat_max) wlat_max = wl;
            ref_base[16'h1235] = 8'hA6;
            tick(60);
            rd8(23'h021234, 2'b00, got, lat);
            chk(got === 8'h5A, $sformatf("post-timing read-even got %h", got));
            rd8(23'h021235, 2'b00, got, lat);
            chk(got === 8'hA6, $sformatf("post-timing read-odd got %h", got));
            // mirror side wrote both bytes: verify chip2 at the mapped address
            chk(peek2(23'h021234) === 8'h5A && peek2(23'h021235) === 8'hA6,
                "post-timing: chip2 mirror bytes");
            $display("write turnaround: min %0d  max %0d clk cycles = %0.1f .. %0.1f ns",
                     wlat_min, wlat_max, wlat_min * `T_NS, wlat_max * `T_NS);
        end

        $display("cpu read latency: min %0d  max %0d clk cycles = %0.1f .. %0.1f ns",
                 lat_min, lat_max, lat_min * `T_NS, lat_max * `T_NS);

        $display("");
        $display("tb_ram_bridge: DONE.  %0d checks passed, %0d errors.", passes, errors);
        if (errors == 0) $display("TB PASS");
        else             $display("TB FAIL");
        $finish;
    end

endmodule
