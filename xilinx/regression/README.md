# xc7 regression cases

One directory per fixed bug. Each design **failed before its patch** and builds after it.

| Case | Guards | Failure before the fix |
|---|---|---|
| `clock-srcc-bufg` | #110 | clock-buffer preplace BFS gave up at 50,000 visited pips; the SRCC-pin-to-BUFG path spans 75,492 wires on xc7a200t, so the clock read as unroutable |
| `bram-sdp-unused-port` | #112 | a width configuration bit was emitted for the unused port of an SDP BRAM and conflicted with the used one, so no bitstream was produced at all |
| `bufg-fabric-driven` | #111 | the placer aborted instead of pre-placing a BUFG driven from the fabric, so any design that divides or gates a clock in logic and re-buffers it failed to place |
| `config-primitive-startupe2` | #113 | the single-site configuration primitives had no pre-placement, so instantiating `STARTUPE2` failed to place |
| `iddr-four-iff-flops` | #115 | only Q1/Q2 of the four-flop IFF were initialised; on silicon the outputs then read Q1=0, Q2=1 despite both being programmed INIT=0 |

## Running

```bash
CHIPDB=/path/to/xc7a200t.bin xilinx/regression/run.sh
```

Needs `yosys` and `nextpnr-xilinx` on `PATH`. **No hardware and no prjxray** — the pass
criterion stops at the FASM, which is where all of these bugs bit.

## Why the pass criterion checks content, not just exit status

A case passes when a **non-empty** `.fasm` is produced. Every bug guarded here stopped the
flow outright, so there is no partial-credit case.

The size check is not decoration. A target that exists but is truncated is exactly how a
failing stage reports success — the same defect class as
[openXC7/demo-projects#13](https://github.com/openXC7/demo-projects/pull/13), where a partial
`.frames` yielded a normal-looking 9.7 MB bitstream that flashed, reported `done 1`, and left
the board silent. An existence check whose negative is unobservable carries no information.

## Two kinds of criterion

Most cases pass on **a non-empty `.fasm`**, because the bug stopped the flow outright.

`iddr-four-iff-flops` is the other kind: #115 changed *which bits are emitted*, not whether the
flow completes, so "it built" would have passed before the fix too. That case carries an
`expect.txt` of regexes which must all appear in the FASM:

```
ILOGIC_Y[01]\.IFF\.ZINIT_Q1
ILOGIC_Y[01]\.IFF\.ZINIT_Q2
ILOGIC_Y[01]\.IFF\.ZINIT_Q3
ILOGIC_Y[01]\.IFF\.ZINIT_Q4
```

Any case may add an `expect.txt`; it is checked after the size check.

## Still missing: #109

`set_multicycle_path` being parsed and silently dropped **does not stop the build**, so neither
criterion above catches it. A test needs the constraint's *effect* to be observable, which is
what #117 adds — once that is in, the case becomes "the warning is absent when the selector
resolves, present when it does not".

## Adding a case

A directory with `top.v` and `top.xdc`, plus a row above naming the patch and the failure.
Keep the I/O minimal: `bram-sdp-unused-port` drives its address and data from internal
counters, so it needs three pins and almost no constraints, which leaves the feature under
test as the only interesting thing in the design.

Constraints use `xc7a200tfbg484-2` pins (ALINX AX7203), resolved from `prjxray-db`
`package_pins.csv`. Another part needs its own `.xdc`.
