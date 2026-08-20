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

NEXTPNR_NAMESPACE_END

#endif
