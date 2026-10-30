# PSRAM v3 switch-over notes (dynamic-phase build)

What changed on 2026-10-30 (working tree, NOT committed — revert with
`backup_20261030_125354_psram_v3/`). Stage 2 (lean drain) is IN this build;
the same-day cleanup then simplified the bridge (see below):
* `rtl/ram_bridge.sv` – drops the old single-controller era carry-overs:
  no `boot_load` special-casing (video is on the dedicated chip 2 and
  cannot collide with loader traffic; display staleness during a download
  self-heals via dirty-marking / address changes), no unused `vram_bank`
  port, no unused `rdy`/`cpu_rd_pending`/`rd_fin` bookkeeping (nothing
  consumes a bridge readiness; the Z80's timing is GA40010-driven).
* `rtl/psram.sv` – the calibrated v3 QPI controller, plus `LEAN=1` passed
  from the bridge: the nibble
  write always comes from `pick` and the write-enable fire depth is
  `2 + cap_sel/2`, with the same value used for the frame drain. The pad
  capture instants, `rd_ext` (device CE hold), and the calibration window
  are untouched, so the capture behaviour is bit-identical to v3 -- only
  the fabric-side wait shrinks (5 -> 2..5 cycles depending on c*).
* sim evidence: `sim/tb_lean_psram.sv` (v3-vs-lean A/B on twin pad chains)
  shows byte-equal reads at every (c, phi) in the window and read-frame
  gains of 3/3/2/2/1/1/0/0 cycles for c = 0..7; `sim/tb_ram_bridge.sh`
  passes 318/318 with the calibrated position and measures the read
  turnaround 242 -> 219 ns (at c*=1; on the board it depends on the c*
  the calibration finds).
* `rtl/psram_cal3.sv`, `rtl/psram_dps.sv` – verbatim copies of the v3
  shared calibrator + DPS stepper.
* `rtl/ram_bridge.sv` – rewritten: two independent controllers
  (U1 = LOANIO 33/19/17/34 / CLK54 / CE48 = CPU+loader+tape;
   U2 = LOANIO 29/27/53/30 / CLK28 / CE23 = base-64K shadow for video),
  direct-issue scheduler (no req/ack toggle), 4-deep request FIFOs,
  byte mirror of every base-window write to U2, word (wtbt=11) video
  fetches into a holding buffer. No BRAM RAM is used any more; the old
  `base_ram` BRAM is removed from `files.qip` (still on disk).
* `Amstrad.sv` – pll instance now expects three outputs
  (C0 64 MHz, C1 128 MHz, C2 128 MHz +DPS) plus the DPS ports
  `phase_en/updn/cntsel[4:0]/phase_done`, `ioctl_wait` and `reset` are
  held until cal_done (2FF synced), and both chips' pads are wired.

PLL (you regenerate with the wizard; port names as in pll96 of
psram-qspi-test-3):
* outclk_0: 64 MHz, 0 deg (clk_sys)
* outclk_1: 128 MHz, 0 deg (clk_ram + both PSRAM SCLK pads)
* outclk_2: 128 MHz, static phase 1172 ps (= 0.15 T = 6 fine taps on the
  640 MHz VCO; 54 deg of the 128 MHz period) - capture clock, DPS counter
  must map to `cntsel = 2` (DPS_CNTL default already 5'd2)
* VCO 640 MHz (50 MHz ref, M=12 + frac 0.8) => S = 4*Fvco/Fout = 20 phase
  steps per half period (fine step 195.3 ps); the bridge defaults `S(20)`.
  outclk_2's static phase is 1172 ps = 0.15 T = 6 fine taps (the v3 window
  start); after the sweep the DPS moves it up to 0.65 T.
* SDC note (Amstrad_ch96.sdc): the C2-origin timing paths (pp/pn/clk_det
  capture retimings into the 128 MHz clk domain) are false-pathed. This is
  the v3 design principle, not a shortcut: TimeQuest cannot evaluate a
  phase-transfer that DPS moves through the sweep (the v3 rig's own SDC
  handled this identically by the loan-io pad arcs being absent); the
  margins live in the capture window construction (>=0.3 T setup / >=0.15 T
  hold inside the chosen sweep region) and correctness is content-verified
  at boot by psram_cal3's pattern check over both chips (cal_status bits:
  eye/len1/len2/det_tog).
* DPS "scanclk" is the 50 MHz reference (CLK_50M).

Bring-up:
1. Fit and build (RBF). Watch the PLL Summary: VCO ~512 MHz, three C
   counters, C2 phase as above.
2. On the board watch `ram_bridge|cal|status[63:0]` (SignalTap file
   `output_files/psram.stp` is configured) - bit map in
   `rtl/psram_cal3.sv` (eye/len1/len2/c*/s*/margin/det_tog...).
   `cal_fail` = no shared eye: if it fails, cal3 falls back to DEF_C(3)
   and boots anyway.
3. Calibration takes ~4-6 ms at boot; the core reset is held until
   cal_done. Re-calibration happens only when the PLL re-locks (init).

Sim: `sim/tb_ram_bridge.sh` (318 checks; reports read ~31 clk = 242 ns
and write ~15 clk = 117 ns turnaround at 128 MHz). The sim's clk_cap is
a behavioural PLL model - on the board the real DPS grid is what cal3
measures, the sim only validates the logic shapes.
