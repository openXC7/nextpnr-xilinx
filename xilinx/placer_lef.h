/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  placer_lef: place_lef.exe's placement passes, ported into nextpnr one at a
 *  time.  See placer_lef.cc for the pass inventory and the porting protocol.
 */

#ifndef PLACER_LEF_H
#define PLACER_LEF_H

#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

extern bool placer_lef(Context *ctx);

// The place_lef transplant.  Runs from customRewriteJson, BEFORE the netlist is
// parsed: its packer recognises LUT6/FDRE/CARRY4/MUXF7, which Arch::pack() would
// have replaced with SLICE_LUTX/SLICE_FFX.  Packs, runs the netlist prepasses,
// places, carry-stamps, and returns the path of the fully stamped netlist for
// nextpnr to parse (empty string on failure).
extern std::string place_lef_transplant(Context *ctx, const std::string &json_path);

NEXTPNR_NAMESPACE_END

#endif
