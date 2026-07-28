# counter25 — VC707 PRBS + Johnson LED counter (fully open-source flow)

A self-checking example that takes a 25-bit PRBS prescaler driving an 8-bit
Johnson counter all the way to a **VC707** (`xc7vx485tffg1761-2`) bitstream
**without Vivado** — nextpnr-xilinx for place & route, Project X-Ray for bitgen,
RapidWright (via the `json_drc-portable` helpers) for the JSON↔DCP↔FASM hops
and an offline physical-netlist simulation that catches the routing defect this
example exists to work around.

Confirmed working on hardware: the 8 LEDs walk a Johnson (twisted-ring) pattern
at ~6 steps/second; `rst` (CPU_RESET, AV40) clears to 0x00.

## Design (`top.v`)

```
200 MHz LVDS sysclk → IBUFDS → BUFG → clk
  prbs   : 25-bit LFSR  x^25 + x^3 + 1   (fb = prbs[24]^prbs[2])
  tick   : prbs == 1                      (once per 2^25-1 cycles ≈ 6 Hz)
  johnson: 8-bit twisted ring, advances on tick → led[7:0]
```

`top.json` is the synthesised netlist (32×FDRE + FDSE + LUTs + IBUFDS/BUFG/OBUFs)
and is the input to the open flow; it is checked in so the example reproduces
without a synthesis front-end.

## The fix this example demonstrates: `insert_lutbuf.py`

nextpnr-xilinx **cannot route the flip-flop input-MUX (`xFFMUX`) route-thru for a
direct FF→FF shift** — where a FF's `D` is fed from another FF's `Q` through the
SLICE `AX`/`BX` bypass input. Such an arc explores essentially forever; when the
per-arc budget skips it, the FF's D-input MUX is left unbound, the `D` pin floats
(`X` in simulation) and on silicon the LFSR corrupts — the classic *"stuck at
0x43"* freeze.

FFs whose `D` comes from a `LUT.O6` route fine (the `FFMUX.O6` site-PIP is used,
no bypass). So `insert_lutbuf.py` inserts a **LUT1 buffer** (`O = I0`,
`INIT="10"`) into every FF→FF shift path, making **every** flip-flop LUT-paired.
nextpnr then routes the whole design with **0 skipped arcs**, and the physical
netlist simulates with **0 undriven (X) nodes**.

(The companion nextpnr knobs `NEXTPNR_ARC_MAX_VISIT` / `NEXTPNR_SKIP_FAILED_ARCS`
in `common/router1.{h,cc}` bound the per-arc A* search and skip-and-log a failed
arc instead of aborting; with the LUT-buffer transform applied nothing is
actually skipped.)

## Reproduce

```sh
./build.sh            # build counter25.bit (writes only into this directory)
./build.sh --flash    # build, then flash to a VC707 over Digilent JTAG
```

`build.sh` honours these environment overrides (defaults shown):

| var        | default                                  | what                                   |
|------------|------------------------------------------|----------------------------------------|
| `NEXTPNR`  | `$HOME/nextpnr-xilinx/build/nextpnr-xilinx` | the router built from this tree      |
| `CHIPDB`   | `$HOME/nextpnr-xilinx/xilinx/xc7vx485t.bin` | chipdb (`bbasm` from the `.bba`)      |
| `JSON_DRC` | `$HOME/json_drc-portable`                | RapidWright helpers `json2dcp` / `dcp2fasm` / `VerilogPhys` |
| `PRJXRAY`  | `$HOME/prjxray`                          | `fasm2frames.py`, `xc7frames2bit`, virtex7 db |

### Pipeline

1. `insert_lutbuf.py top.json top_buf.json` — LUT1-buffer every FF→FF shift.
2. `nextpnr-xilinx --router router1 … --json top_buf.json --write routed.json`
   — place + route (PASS @ ~496 MHz, 0 unrouted).
3. `json2dcp` → `counter25.dcp`.
4. `VerilogPhys` → `phys.v`; `gen_tb.py` + `iverilog`/`vvp` → **X-cycles must be 0**
   (the regression that proves the bypass-FFMUX defect is gone — the LFSR walks
   `2→4→9→12→24→…` cleanly).
5. `dcp2fasm` → `counter25.fasm` (38 LUTs, 33 FFs, 1 BUFG, 1279 pips).
6. `fasm2frames` + `xc7frames2bit` → `counter25.bit`.

`fasm2frames` prints a benign `frame_clear: invalid word address 102 …STEPDOWN`
warning for the `led[4]`/AR35 SING IOB and exits non-zero, but still writes valid
frames; `build.sh` tolerates this and sanity-checks the frame file.

## Files

| file                | role                                             |
|---------------------|--------------------------------------------------|
| `top.v`             | design source (reference)                        |
| `top.xdc`           | VC707 pin constraints                            |
| `top.json`          | synthesised netlist — input to the flow          |
| `insert_lutbuf.py`  | the FFMUX-bypass workaround (LUT1 buffers)        |
| `gen_tb.py`         | builds the iverilog TB from `VerilogPhys` probes |
| `build.sh`          | full reproduce script                            |

Generated (not checked in): `top_buf.json`, `routed.json`, `counter25.{dcp,fasm,frames,bit}`, `phys.v`, `tb.v`, `sim.vvp`.

## Note on the clock path

This example brings the 200 MHz clock in via the simplified
`RIOI_…_I0.RIOI_IBUF0` IBUF path. It works here; if a faster/marginal design
freezes despite a clean offline sim, the clock-IO needs the full ISERDES-master
input path (separate from the FFMUX fix above).
