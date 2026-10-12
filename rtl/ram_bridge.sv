`timescale 1ns/1ps

module ram_bridge(
    output PSRAM_SCLK,
    output PSRAM_CE_N,
    output [3:0] PSRAM_SIO_OUT,
    output [3:0] PSRAM_SIO_OE,
    input [3:0] PSRAM_SIO_IN,

    // cpu/chipset interface
    input init,
    input clk,        // host clock (clk_ram, 128 MHz)
    input clkref,     // CPU clock enable (~8 MHz), not used

    input [1:0] bank,
    input [7:0] din,
    output [7:0] dout,
    input [22:0] addr,
    input oe,
    input we,

    output [15:0] vram_dout,
    input [22:0] vram_addr,
    input [1:0] vram_bank,

    input [22:0] tape_addr,
    input [7:0] tape_din,
    output reg [7:0] tape_dout,
    input tape_wr,
    output reg tape_wr_ack,
    input tape_rd,
    output reg tape_rd_ack,
    input boot_load,   // ROM/DAN/SNA streaming: suppress VRAM reads
                       // so loader writes are serviced immediately

    // high unless a CPU op is ongoing
    output rdy
);

    localparam [0:0] SRC_CPU  = 1'b0;
    localparam [0:0] SRC_TAPE = 1'b1;

    assign PSRAM_SCLK = clk;

    wire rst_n = ~init;

    wire b_ready;
    wire [15:0] b_dout;
    reg ce_n_p;
    reg [3:0] sio_out_p, sio_oe_p;
    reg [22:0] br_addr;
    reg [7:0] br_din;
    reg [1:0] br_wtbt;
    reg br_rd, br_we;

    psram #(
        .PWRUP_CYCLES(30000),
        .RST_CYCLES(4000),
        .MID_CYCLES(100)
    ) psram (
        .clk(clk),
        .reset(~rst_n),
        .init(init),

        .ce_n(ce_n_p),
        .sio_out(sio_out_p),
        .sio_oe(sio_oe_p),
        .sio_in(PSRAM_SIO_IN),

        .addr(br_addr),
        .wtbt(br_wtbt),
        .rd(br_rd),
        .we(br_we),
        .din({8'h00, br_din}),
        .dout(b_dout),

        .ready(b_ready)
    );

    assign PSRAM_CE_N = ce_n_p;
    assign PSRAM_SIO_OUT = sio_out_p;
    assign PSRAM_SIO_OE = sio_oe_p;
    
    base_ram base_ram(
        .clock(clk),
        
        .address_a(bram_addr),
        .data_a(bram_din),
        .wren_a(bram_we),
        .q_a(bram_dout),
        
        .address_b(vram_addr[15:1]),
        .wren_b(1'b0),
        .q_b(vram_dout)
    );

    reg [15:0] bram_addr;
    reg [7:0] bram_din;
    reg [7:0] bram_dout;
    reg bram_we;
    wire is_base_ram /* synthesis keep */ = addr[22:16] == 7'd2; // The base RAM (64K), shaved with video
    always @(posedge clk) begin
        reg last_we = 1'b0;
        reg last_oe = 1'b0;
        last_we <= we;
        last_oe <= oe;
        bram_we <= 1'b0;
        if (is_base_ram && ((~last_oe & oe) || (~last_we & we))) begin
            bram_addr <= addr[15:0];
            bram_we <= we;
            if (we) bram_din <= din;
        end
    end
    // assign dout = (oe | we) ? ram_dout : 8'hFF;

    assign dout = (oe | we) ? is_base_ram ? bram_dout[7:0] : ram_dout : 8'hFF;


    // ======================================================================
    // host side
    // ======================================================================
    localparam [1:0] H_IDLE = 2'd0, H_BUSY = 2'd1;
    reg [1:0] hstate = H_IDLE;

    // CPU bus capture: remember every edge so no access is dropped
    reg old_oe = 1'b0, old_we = 1'b0, old_tape = 1'b0;
    reg cpu_armed = 1'b0;
    reg [24:0] cpu_addr;
    reg cpu_wr;
    reg [7:0] cpu_din_l;
    reg cpu_rd_pending = 1'b0;

    // tape request latch
    reg tape_armed = 1'b0;
    reg [24:0] tape_addr_l;
    reg tape_wr_l;
    reg [7:0] tape_din_l;

    reg [7:0] ram_dout;

    // outgoing request / in-flight bookkeeping (stable as long as H_BUSY)
    reg req_t = 1'b0;
    reg [22:0] req_addr;
    reg [7:0] req_din;
    reg req_we;
    reg [1:0] req_wtbt;
    reg [1:0] req_src;

    assign rdy = ~cpu_rd_pending;

    always @(posedge clk) begin
        old_oe <= oe;
        old_we <= we;
        old_tape <= tape_rd | tape_wr;

        tape_wr_ack <= 1'b0;

        if (init) begin
            hstate <= H_IDLE;
            req_t <= 1'b0;
            req_addr <= 23'd0;
            req_din <= 8'd0;
            req_we <= 1'b0;
            req_wtbt <= 2'b00;
            req_src <= SRC_CPU;
            cpu_armed <= 1'b0;
            tape_armed <= 1'b0;
            tape_wr_ack <= 1'b0;
            tape_rd_ack <= 1'b0;
            cpu_rd_pending <= 1'b0;
        end
        else begin
            // CPU bus capture. During a boot/DAN/SNA download the loader is the
            // only legitimate source (the CPU is held in reset), so reads are
            // not armed — this also stops pre-download stragglers from sneaking
            // a read into the window right at the ioctl edge.
            if (boot_load) begin
                if(~old_we & we) begin
                    cpu_armed <= 1'b1;
                    cpu_addr <= {bank, addr};
                    cpu_wr <= 1'b1;
                    cpu_din_l <= din;
                end
            end
            else if ((~old_oe & oe) | (~old_we & we)) begin
                cpu_armed <= 1'b1;
                cpu_addr <= {bank, addr};
                cpu_wr <= we;
                cpu_din_l <= din;
                if(~we) cpu_rd_pending <= 1'b1;
            end

            // tape capture
            if ((tape_rd | tape_wr) & ~old_tape) begin
                tape_armed <= 1'b1;
                tape_addr_l <= {2'b11, tape_addr};
                tape_wr_l <= tape_wr;
                tape_din_l <= tape_din;
            end

            case (hstate)
                H_IDLE: begin
                    if (cpu_armed & cpu_wr) begin
                        req_addr <= {cpu_addr[24:23], cpu_addr[20:0]};
                        req_din <= cpu_din_l;
                        req_we <= 1'b1;
                        req_wtbt <= 2'b00;
                        req_src <= SRC_CPU;
                        cpu_armed <= 1'b0;
                        hstate <= H_BUSY;
                        req_t <= ~req_t;
                    end
                    else if (cpu_armed) begin
                        req_addr <= {cpu_addr[24:23], cpu_addr[20:0]};
                        req_din  <= cpu_din_l;
                        req_we   <= 1'b0;
                        req_wtbt <= 2'b00;
                        req_src  <= SRC_CPU;
                        cpu_armed<= 1'b0;
                        hstate   <= H_BUSY;
                        req_t    <= ~req_t;
                    end
                    else if (tape_armed) begin
                        req_addr <= {tape_addr_l[24:23], tape_addr_l[20:0]};
                        req_din  <= tape_din_l;
                        req_we   <= tape_wr_l;
                        req_wtbt <= 2'b00;
                        req_src  <= SRC_TAPE;
                        tape_armed<= 1'b0;
                        hstate    <= H_BUSY;
                        req_t     <= ~req_t;
                    end
                end

                H_BUSY: begin
                    if (ack_t == req_t) begin
                        case (req_src)
                            SRC_CPU: begin
                                if (~req_we) begin
                                    ram_dout <= rd_data[7:0];
                                    cpu_rd_pending <= 1'b0;
                                end
                            end
                            SRC_TAPE: begin
                                tape_dout <= rd_data[7:0];
                                if (req_we) tape_wr_ack <= 1'b1;
                                else tape_rd_ack <= ~tape_rd_ack;
                            end
                        endcase
                        hstate <= H_IDLE;
                    end
                end
            endcase
        end
    end

    // ======================================================================
    // bridge side
    // ======================================================================
    localparam W_IDLE = 3'd0, W_ISSUE = 3'd1, W_WAIT0 = 3'd2, W_WAIT = 3'd3, W_ACK = 3'd4;
    reg [2:0] st = W_IDLE;

    reg req_t_p = 1'b0;                // last processed req_t value
    reg br_op_we;
    reg [15:0] rd_data;
    reg ack_t = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            st <= W_IDLE;
            req_t_p <= 1'b0;
            br_op_we <= 1'b0;
            br_addr <= 23'd0;
            br_din <= 8'h00;
            br_wtbt <= 2'b00;
            br_rd <= 1'b0;
            br_we <= 1'b0;
            rd_data <= 16'h0000;
            ack_t <= 1'b0;
        end
        else begin
            br_rd <= 1'b0;      // strobes are single-cycle by default
            br_we <= 1'b0;

            case (st)
                W_IDLE: begin
                    if (req_t != req_t_p) begin
                        req_t_p  <= req_t;
                        br_addr  <= req_addr;
                        br_din   <= req_din;
                        br_wtbt  <= req_wtbt;
                        br_op_we <= req_we;
                        st <= W_ISSUE;
                    end
                end

                W_ISSUE: begin
                    // wait for the bridge to be idle, then raise the strobe
                    if (b_ready) begin
                        if (br_op_we) br_we <= 1'b1;
                        else br_rd <= 1'b1;
                        st <= W_WAIT0;
                    end
                end
                
                W_WAIT0: begin
                    if (~b_ready) begin
                        st <= W_WAIT;
                    end
                end
                
                W_WAIT: begin
                    if (b_ready) begin
                        rd_data <= b_dout;
                        st <= W_ACK;
                    end
                end

                W_ACK: begin
                    ack_t <= ~ack_t;
                    st <= W_IDLE;
                end

                default:;
            endcase
        end
    end


endmodule
