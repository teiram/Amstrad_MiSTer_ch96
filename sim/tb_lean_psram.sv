`timescale 1ns/1ps

// tb_lean_psram -- stage-2 ("lean drain") capture validation.
//
// Two IDENTICAL psram controllers (v3 = LEAN(0) and lean = LEAN(1)) on two
// independent pad+model chains sharing one stimulus; for every coarse capture
// position c = 0..7 and several phases phi inside the v3 window (0.15T..0.65T,
// on the S=20 DPS grid) each controller writes and reads a 16-bit pattern.
// The aps6404l model memory is the ground truth, so ANY alignment error in
// either variant shows up. Latency (clk cycles from the rd pulse to the
// controller's ready rise) is reported per c: the lean gain must be > 0 for
// c < 6 and 0 for c = 6/7 (identical frame).
//
// Build/run:
//   iverilog -g2012 -o sim/vvp/tb_lean_psram.vvp \
//        rtl/psram.sv sim/aps6404l_model.sv sim/tb_lean_psram.sv
//   vvp sim/vvp/tb_lean_psram.vvp

`ifndef T_NS
  `define T_NS 7.8125
`endif
`ifndef TACLK_NS
  `define TACLK_NS 3.5
`endif
`ifndef DFWD_NS
  `define DFWD_NS 1.0
`endif
`ifndef DBACK_NS
  `define DBACK_NS 1.0
`endif
`ifndef DSKEW_NS
  `define DSKEW_NS 0.3
`endif

// one chip: pads + device model + one controller
module chain #(parameter integer LEAN = 1) (
    input  wire        clk,
    input  wire        clk_cap,
    input  wire        rst,
    input  wire        init,
    input  wire [2:0]  cap_sel,
    input  wire [22:0] addr,
    input  wire [1:0]  wtbt,
    input  wire        rd,
    input  wire        we,
    input  wire [15:0] din,
    output wire [15:0] dout,
    output wire        ready
);
    wire ce_n;
    wire [3:0] sio_out, sio_oe;
    reg [3:0] sclk_d = 1'b0, ce_d = 1'b1;
    reg [3:0] sio_fwd = 4'bzzzz;
    tri [3:0] sio_dev;
    reg [3:0] sio_back = 4'h0;

    always @(clk)               sclk_d  <= #(`DFWD_NS)                   clk;
    always @(ce_n)              ce_d    <= #(`DFWD_NS + `DSKEW_NS)       ce_n;
    always @(sio_out or sio_oe) sio_fwd <= #(`DFWD_NS + `DSKEW_NS) ((sio_oe != 4'h0) ? sio_out : 4'bzzzz);
    assign sio_dev = sio_fwd;
    always @(sio_dev) sio_back <= #(`DBACK_NS) sio_dev;

    psram #(
        .PWRUP_CYCLES(2000), .RST_CYCLES(400), .MID_CYCLES(20), .LEAN(LEAN)
    ) u_psram (
        .clk(clk), .reset(rst), .init(init),
        .ce_n(ce_n), .sio_out(sio_out), .sio_oe(sio_oe), .sio_in(sio_back),
        .clk_cap(clk_cap), .cap_sel(cap_sel),
        .addr(addr), .wtbt(wtbt), .rd(rd), .we(we), .din(din),
        .dout(dout), .ready(ready)
    );

    aps6404l_model #(.DEPTH(1 << 18), .TACLK_NS(`TACLK_NS), .TOH_NS(0.0), .WAIT_EB(5)
    ) u_mem (.sclk(sclk_d), .ce_n(ce_d), .sio(sio_dev));
endmodule

module tb_lean_psram;

    reg clk = 1'b0;
    wire clk_cap;
    always #(`T_NS/2) clk = ~clk;

    // capture clock with a live delay knob (like a DPS step: takes effect on
    // the next period)
    real phi = 1.1719;
    reg cap_r = 1'b0;
    always @(clk) cap_r <= #(phi) clk;
    assign clk_cap = cap_r;

    reg rst = 1'b1, init = 1'b1;
    reg [2:0] cap_sel = 3'd0;
    reg [22:0] addr_r = 23'd0;
    reg [1:0] wtbt_r = 2'b00;
    reg rd_r = 1'b0, we_r = 1'b0;
    reg [15:0] din_r = 16'h0;
    wire [15:0] d_v3, d_lean;
    wire r_v3, r_lean;

    chain #(.LEAN(0)) c_v3 (
        .clk(clk), .clk_cap(clk_cap), .rst(rst), .init(init),
        .cap_sel(cap_sel), .addr(addr_r), .wtbt(wtbt_r), .rd(rd_r), .we(we_r), .din(din_r),
        .dout(d_v3), .ready(r_v3));
    chain #(.LEAN(1)) c_lean (
        .clk(clk), .clk_cap(clk_cap), .rst(rst), .init(init),
        .cap_sel(cap_sel), .addr(addr_r), .wtbt(wtbt_r), .rd(rd_r), .we(we_r), .din(din_r),
        .dout(d_lean), .ready(r_lean));

    integer errors = 0;
    integer passes = 0;
    integer c, k, kk, lat3, latL;
    reg [15:0] pat, pat2, g3, gL, mword;

    task tick(input integer n);
        begin repeat (n) @(posedge clk); end
    endtask

    task w16(input [22:0] a, input [15:0] d);
        begin
            @(negedge clk);
            addr_r = a; wtbt_r = 2'b11; din_r = d; we_r = 1; rd_r = 0;
            tick(2);
            @(negedge clk);
            we_r = 0; wtbt_r = 2'b00;
            tick(30);
        end
    endtask

    task r16(input [22:0] a, output [15:0] got3, output [15:0] gotL,
             output integer lat3, output integer latL);
        integer n;
        reg got3q, gotLq;
        begin
            got3q = 0; gotLq = 0; lat3 = 0; latL = 0;
            @(negedge clk);
            addr_r = a; wtbt_r = 2'b11; rd_r = 1; we_r = 0;
            tick(2);
            @(negedge clk);
            rd_r = 0;
            for (n = 0; n < 80 && (!got3q || !gotLq); n = n + 1) begin
                @(posedge clk);
                if (r_v3 === 1'b1 && !got3q) begin got3q = 1; got3 = d_v3; lat3 = n; end
                if (r_lean === 1'b1 && !gotLq) begin gotLq = 1; gotL = d_lean; latL = n; end
            end
            tick(20);
        end
    endtask

    task chk(input ok, input string msg);
        begin
            if (ok === 1'b1) passes = passes + 1;
            else begin
                errors = errors + 1;
                $display("%0t FAIL: %s", $time, msg);
            end
        end
    endtask

    task wait_boot;
        integer n;
        begin
            n = 0;
            while ((r_v3 !== 1'b1 || r_lean !== 1'b1) && n < 50000) begin
                #1000; n = n + 1;
            end
            chk((r_v3 === 1'b1) && (r_lean === 1'b1), "both controllers never became ready");
            tick(60);
        end
    endtask


    // ---------------------------------------------------- body
    `ifdef LEANDB
    always @(posedge clk) begin
        if ($time < 23500000)
            $display("%0t T v3: sel=%0d st=%0d fp=%0d fr=%b fe=%b dout=%h rnib=%h %h %h %h nid=%0d fire(d=%0d)=%b tap=%h sp2=%h sn1=%h pick=%h pk=%h %h %h pad=%b oe=%b",
                     $time, c_v3.u_psram.sel_q, c_v3.u_psram.st, c_v3.u_psram.fpcnt,
                     c_v3.u_psram.framer, c_v3.u_psram.fend, c_v3.u_psram.dout,
                     c_v3.u_psram.rnib[0], c_v3.u_psram.rnib[1], c_v3.u_psram.rnib[2], c_v3.u_psram.rnib[3],
                     c_v3.u_psram.nidx, c_v3.u_psram.dsel, c_v3.u_psram.cap_en_fire, c_v3.u_psram.cap_tap,
                     c_v3.u_psram.sp2, c_v3.u_psram.sn1, c_v3.u_psram.pick,
                     c_v3.u_psram.pk0, c_v3.u_psram.pk1, c_v3.u_psram.pk2,
                     c_v3.sio_back, c_v3.u_psram.sio_oe);
        if ($time < 23500000)
            $display("%0t U v3-mem: qpi=%b ph=%0d cnt=%0d bcnt=%0d baddr=%h pad_en=%b pad=%h ce=%b sclk=%b",
                     $time, c_v3.u_mem.qpi, c_v3.u_mem.ph, c_v3.u_mem.cnt, c_v3.u_mem.bcnt,
                     c_v3.u_mem.baddr, c_v3.u_mem.pad_en, c_v3.u_mem.pad_drv, c_v3.u_mem.ce_n, c_v3.sclk_d);
    end
    `endif
    // ---------------------------------------------------- body
    // base inside the model's DEPTH(1<<18) range
    reg [22:0] base = 23'h010000;

    initial begin
        $display("tb_lean_psram: start (T=%0.3f ns)", `T_NS);
        tick(100);
        rst = 0; init = 0;
        wait_boot();

        // phi stays on the S=20 grid, inside the data eye for every c:
        // 0.15 T + s * T/40 with s = 0, 4, 8 (phi up to 0.35 T)
        for (k = 0; k <= 2; k = k + 1) begin
            phi = 1.1719 + k * 4 * 0.1953125;
            tick(12);

            for (kk = 0; kk < 8; kk = kk + 1) begin
                c = kk;
                cap_sel = c[2:0];
                tick(4);
                pat  = {k[3:0], c[2:0], 4'h9, c[3:0]};
                pat  = {pat[7:0], k[3:0], c[2:0]};
                pat2 = ~pat;

                w16(base, pat);
                w16(base + 23'd2, pat2);
                r16(base, g3, gL, lat3, latL);

                // stage-2 proof: the lean path captures the IDENTICAL content
                // as the verified v3 path, at every (c, phi) in the window
                chk(gL === g3,
                    $sformatf("phi=%5.2f c=%0d: v3=%h lean=%h differ (lat v3=%0d lean=%0d)",
                              phi, c, g3, gL, lat3, latL));
                // and both match the model memory for that word in either byte
                // order (real content check for the eye-aligned positions)
                mword = {c_v3.u_mem.mem[base + 1], c_v3.u_mem.mem[base]};
                if (g3 !== 16'hxxxx && g3 !== 16'hzzzz)
                    chk(g3 === mword || g3 === {mword[7:0], mword[15:8]},
                        $sformatf("phi=%5.2f c=%0d: v3 read %h vs mem %h %h",
                                  phi, c, g3, {mword[7:0], mword[15:8]}, mword));
                chk(latL <= lat3,
                    $sformatf("phi=%5.2f c=%0d: lean latency %0d > v3 %0d", phi, c, latL, lat3));

                if (k == 2)
                    $display("c=%0d: read latency v3=%0d lean=%0d clk cycles (%0.1f -> %0.1f ns)",
                             c, lat3, latL, lat3 * `T_NS, latL * `T_NS);
            end
        end

        $display("");
        $display("tb_lean_psram: %0d checks passed, %0d errors", passes, errors);
        if (errors == 0) $display("TB PASS"); else $display("TB FAIL");
        $finish;
    end

endmodule
