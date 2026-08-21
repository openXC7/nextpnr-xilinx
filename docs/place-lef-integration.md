# Integrating place_lef into nextpnr

Branch `place-lef-integration`.  Working notes; the scope below is evidence,
not a plan sketch -- every claim cites the line it came from.

## Why routethru comes first

`place_lef_core.ml:2940` materialises identity `LUT1` cells (`INIT=10`) purely
because nextpnr cannot route a signal *through* an unused LUT:

    S[k] LUT-driven -> stamp the driver at <site>/<slot>6LUT
    S[k] FF/ext     -> identity LUT1 (INIT=10) at the 6LUT, rewire S

That is ~971 cells per ethmin build (`Srt` 701, `DIgnd` 576, `DIvcc` 37,
`DIrt` 274).  Add routethru and those cells, `carry_stamp.py` (501 lines) and
its OCaml port (~300 lines) all stop being necessary.  So routethru DELETES
code; every other piece of the integration ADDS it.  Do it first.

## The trap: routethru looks implemented, and is not

`grep routethru` finds plenty, which is misleading.  What actually exists:

| location | what it is |
|---|---|
| `xilinx/arch.h:160` | `PIP_LUT_ROUTETHRU = 5` enum constant |
| `xilinx/arch.h:1230` | availability logic in `usp_pip_hard_unavail` |
| `xilinx/arch.h:1478` | a flat 300 ps delay in `getPipDelay` |
| `xilinx/python/nextpnr_structs.py:12` | the same constant on the Python side |

What does NOT exist: **any pip is ever given that type**.  Enumerating the
constructed types in `xilinx/python/*.py` gives

    3 CONST_DRIVER   1 LUT_PERMUTATION   1 SITE_ENTRANCE
    1 SITE_EXIT      1 SITE_INTERNAL     2 TILE_ROUTING

`LUT_ROUTETHRU` is absent.  The routing graph therefore contains **zero**
routethru arcs, and the two `arch.h` consumers are unreachable dead code
inherited from upstream.  `place_lef_core.ml`'s comment is exactly right.

Second independent gap, `xilinx/fasm.cc:360`:

    if (pd.flags != PIP_TILE_ROUTING)
        return;

So even if the arcs existed and the router used one, the FASM writer would drop
it silently -- no bits, no warning.  A LUT routethru is not a routing pip in the
bitstream: it needs the LUT `INIT` programmed to a pass-through of the specific
input used, plus the output-select bits.  Left unwritten, the LUT sits at
`INIT=0` and the signal dies.  That failure is invisible until the board is dark.

## Scope, therefore, is three parts and not one

The earlier "moderate" estimate assumed the arcs existed and only the FIXMEs
needed fixing.  They do not exist.

**(a) chipdb -- generate the arcs.**  `nextpnr_structs.py:460` is the template:
`LUT_PERMUTATION` already walks every LUT input site-wire (`A1`..`D6`) and emits
a pseudo-pip per input, encoding `("ABCDEFGH".index(letter) << 8) | ((j-1) << 4)
| (i-1)`, plus `(4 << 8)` when `s.rel_xy()[0] == 1` to distinguish the tile's
second SLICE.  That `>> 8` field is the same `eight` slot index `arch.h:1231`
reads back, so the encodings already agree.  A routethru pip is the same walk
with `from_wire` = the LUT input tile wire and `to_wire` = the LUT *output*.
  - OPEN QUESTION, size: 8 LUTs x 6 inputs per CLB across every CLB tile is a
    large number of new arcs.  Measure the .bin growth and the routing-time cost
    before committing to the full cross-product; restricting to O6 may be enough
    for the CARRY4 `S`/`DI` cases that motivate this.

**(b) `arch.h` -- make availability correct rather than conservative.**  Two
blanket FIXMEs currently reject whole classes:

    if (eight == 0) return true;   // FIXME: conflict with ground
    if (dest & 0x1) return true;   // FIXME: routethru to MUX

and the 300 ps delay at :1478 is a placeholder, not a LUT delay from the SDF.
  - Also unhandled: two nets routing through the *same* LUT via different input
    pips are distinct pips, so both look available.  Needs a per-LUT
    reservation, not just "is a cell bound here".

**(c) `fasm.cc` -- emit the bits.**  For each used routethru pip: the
pass-through `INIT` for the input actually taken, and the output/used bits.
This is the part that decides whether the bitstream works.

## Measured starting point for placement convergence (2026-08-20)

`make placement-ab` in the demos repo compares
    A  sv2v -> yosys -> place_lef -> nextpnr (route only)
    B  sv2v -> yosys -> nextpnr's own placer
cell-by-cell on johnson.  Both sides are emitted by `nextpnr --write`, so a
mismatch is a real placement difference and not a formatting one.

    join 100% (85/85)     after structurally matching packer-created cells
    exact BEL agreement   24.7% (21/85)
    median site distance  85,  p90 158,  max 258

The 24.7% is entirely the constraints file.  Every agreeing cell is I/O pinned
by top.xdc -- PAD x11, IOB18_OUTBUF_DCIEN x8, IOB18M/IOB18_INBUF_DCIEN x1 each
-- and every placeable cell disagrees: SLICE_FFX x36, SLICE_LUTX x16,
SELMUX2_1 x9, VCC, GND, BUFGCTRL.  **Logic agreement is 0/64.**

Two consequences for this work:

1. There is no incremental path.  The placers agree on nothing they choose
   themselves, so tuning weights cannot walk 0% upwards in any meaningful way.
   Convergence means putting place_lef's placement DECISIONS inside nextpnr,
   after which the number should jump to ~100% in one step.  A small rise is
   noise, not progress.
2. The RELATIVE structure already agrees even though the absolute positions do
   not: a MUXF7 cluster's lut0/mux7/mux8 land on one slice on both sides
   (A `SLICE_X121Y110`, B `SLICE_X124Y125`).  Packing is not the disagreement;
   site selection is.

Note the join required structural matching to reach 100%: both flows
materialise 9 constant drivers for the MUXF7 inputs, and they name them in
disjoint spaces (`<path>.mux7_const_I0` vs `$PACKER_GND_NET$LUT$10`).  Once the
enhanced placer packs for itself, place_lef's names cease to exist, so name
equality is permanently wrong for packer-created cells; they are paired by what
they DRIVE.  Their inputs are deliberately excluded from the signature --
place_lef emits INIT=00 and wires A1 to an arbitrary net, which pulls unrelated
logic in and stops the match.

## Ordering note

(c) is the one with a silent failure mode, so it should land with an assertion
that every used routethru pip produced bits -- the existing "Unprocessed
route-thru" warning at `fasm.cc:434` covers a different pip class and would not
fire here.
