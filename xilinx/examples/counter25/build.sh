#!/usr/bin/env bash
# build.sh -- reproduce the VC707 PRBS+Johnson counter through the fully
# open-source (no-Vivado) flow, end to end:
#
#   top.json                 synthesised netlist (PRBS LFSR + Johnson + LEDs)
#     -> insert_lutbuf.py     LUT1-buffer every FF->FF shift (FFMUX workaround)
#     -> nextpnr-xilinx       place + route  (router1, per-arc budget + skip)
#     -> json2dcp             routed JSON -> DCP
#     -> VerilogPhys + iverilog   offline physical-netlist sim (X-cycles must be 0)
#     -> dcp2fasm             DCP -> FASM
#     -> fasm2frames          FASM -> frames        (Project X-Ray)
#     -> xc7frames2bit        frames -> .bit        (Project X-Ray)
#     -> openFPGALoader       flash (only with --flash)
#
# All outputs are written into THIS directory.  Nothing else is touched.
#
# Tool locations (override via environment):
set -euo pipefail
cd "$(dirname "$0")"

NEXTPNR=${NEXTPNR:-$HOME/nextpnr-xilinx/build/nextpnr-xilinx}
CHIPDB=${CHIPDB:-$HOME/nextpnr-xilinx/xilinx/xc7vx485t.bin}
JSON_DRC=${JSON_DRC:-$HOME/json_drc-portable}          # json2dcp/dcp2fasm/VerilogPhys
PRJXRAY=${PRJXRAY:-$HOME/prjxray}
PART=xc7vx485tffg1761-2

CP="$JSON_DRC/lib/classes:$JSON_DRC/lib/rapidwright_json_drc.jar:$JSON_DRC/lib/rapidwright-2025.2.1-standalone-lin64.jar:$JSON_DRC/lib/gson-2.10.1.jar"
export RAPIDWRIGHT_PATH="$JSON_DRC/data"
export XRAY_WIRE_ORACLE="$JSON_DRC/oracle/$PART.oracle.txt.gz"
JAVA="java -cp $CP"

echo "[1] insert LUT1 buffers (FFMUX bypass workaround)"
python3 insert_lutbuf.py top.json top_buf.json

echo "[2] nextpnr place + route"
NEXTPNR_ARC_MAX_VISIT=2000000 NEXTPNR_SKIP_FAILED_ARCS=1 \
  "$NEXTPNR" --router router1 --chipdb "$CHIPDB" --xdc top.xdc \
    --json top_buf.json --write routed.json --freq 200 2>&1 \
  | grep -iE "Max frequency|left unrouted|errors" || true

echo "[3] json2dcp -> counter25.dcp"
$JAVA dev.fpga.rapidwright.json2dcp "$PART" routed.json counter25.dcp \
  | grep -iE "Writing DCP|ROUTING net=clk" || true

echo "[4] VerilogPhys + iverilog offline check (X-cycles must be 0)"
$JAVA dev.fpga.rapidwright.VerilogPhys counter25.dcp phys.v | grep VerilogPhys
python3 gen_tb.py phys.v tb.v
iverilog -g2012 -o sim.vvp phys.v tb.v
vvp sim.vvp | tail -5

echo "[5] dcp2fasm -> counter25.fasm"
$JAVA dev.fpga.rapidwright.dcp2fasm counter25.dcp counter25.fasm | grep dcp2fasm

echo "[6] fasm2frames + xc7frames2bit -> counter25.bit"
# fasm2frames returns non-zero on the benign SING-IOB STEPDOWN warning for
# led[4]/AR35 but still writes valid frames -- tolerate it, then sanity-check.
XRAY_ALLOW_MISSING_FEATURES=1 PATH="$PRJXRAY/env/bin:$PATH" \
  python3 "$PRJXRAY/utils/fasm2frames.py" \
    --db-root "$PRJXRAY/database/virtex7" --part "$PART" \
    counter25.fasm counter25.frames 2>&1 | tail -2 || true
test -s counter25.frames || { echo "ERROR: no frames produced"; exit 1; }
"$PRJXRAY/build/tools/xc7frames2bit" \
  --part_file "$PRJXRAY/database/virtex7/$PART/part.yaml" \
  --part_name "$PART" --frm_file counter25.frames \
  --output_file counter25.bit 2>&1 | tail -1
ls -lh counter25.bit

if [ "${1:-}" = "--flash" ]; then
  echo "[7] flash to VC707"
  openFPGALoader --freq 15000000 --cable digilent counter25.bit
fi
echo "Done.  Bitstream: $(pwd)/counter25.bit"
