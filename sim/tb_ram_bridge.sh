#!/bin/sh
# Build and run the ram_bridge v3 testbench (menu: iverilog -g2012).
#   ./sim/tb_ram_bridge.sh          # fast run
#   DBG=1 ./sim/tb_ram_bridge.sh    # + engine/issue tracing
#   VCD=1 ./sim/tb_ram_bridge.sh    # + waveform (sim/vcd/tb_ram_bridge.vcd)
#
# The bridge simulates at reduced power-up/calibration workloads (PWRUP 2000,
# CAL_WORDS 4, CAL_PASSES 1) so the whole thing fits in ~2.5 ms of sim time.
# Board values (30000/16/4) are the module defaults and stay untouched.

set -e
cd "$(dirname "$0")/.."
mkdir -p sim/vvp sim/vcd

FLAGS=""
[ -n "$DBG" ] && FLAGS="$FLAGS -DDBG"
[ -n "$VCD" ] && FLAGS="$FLAGS -DVCD"

iverilog -g2012 $FLAGS -o sim/vvp/tb_ram_bridge.vvp \
    rtl/ram_bridge.sv rtl/psram.sv rtl/psram_cal3.sv rtl/psram_dps.sv \
    sim/aps6404l_model.sv sim/tb_ram_bridge.sv

vvp sim/vvp/tb_ram_bridge.vvp
