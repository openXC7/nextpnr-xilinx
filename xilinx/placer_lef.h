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

// The place_lef transplant's recognition pre-pass.  Must run on the RAW
// netlist (from customAfterLoad), before nextpnr's packer rewrites LUT6/FDRE
// into SLICE_LUTX/SLICE_FFX -- pack_to_lef recognises the primitives.
extern bool place_lef_prepass(Context *ctx, const std::string &json_path);

NEXTPNR_NAMESPACE_END

#endif
