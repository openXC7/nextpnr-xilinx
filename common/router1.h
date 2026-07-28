/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2018  Clifford Wolf <clifford@symbioticeda.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef ROUTER1_H
#define ROUTER1_H

#include "log.h"
#include "nextpnr.h"
NEXTPNR_NAMESPACE_BEGIN

struct Router1Cfg
{
    Router1Cfg(Context *ctx);

    int maxIterCnt;
    bool cleanupReroute;
    bool fullCleanupReroute;
    bool useEstimate;
    // Per-arc A* node budget.  An unroutable sink otherwise explores the whole
    // chip (effective hang).  When exceeded the arc fails; with skipFailedArcs
    // it is left unrouted (to be finished by an external router) instead of
    // aborting the whole route.  Default INT_MAX = original unbounded behaviour.
    int arcMaxVisitCnt;
    bool skipFailedArcs;
    // When set, route_const_arc() routes GND/VCC sinks using ONLY free wires (no
    // ripup pass), so the constant net never tears up an already-placed signal to
    // grab a shorter path -- it takes a longer free path instead.  Used by the
    // hybrid flow (router2 signals first, then router1 const) to keep the signal
    // routing/timing intact.  Env: NEXTPNR_GND_NO_RIPUP.
    bool constNoRipup;
    delay_t wireRipupPenalty;
    delay_t netRipupPenalty;
    delay_t reuseBonus;
    delay_t estimatePrecision;
};

extern bool router1(Context *ctx, const Router1Cfg &cfg);

NEXTPNR_NAMESPACE_END

#endif // ROUTER1_H
