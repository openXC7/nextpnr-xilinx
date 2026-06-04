#!/usr/bin/env python3
# insert_lutbuf.py -- work around the xc7 bypass-FFMUX routing defect.
#
# nextpnr-xilinx cannot route the flip-flop input-MUX (xFFMUX) "route-thru"
# for a *direct* FF->FF shift (D fed from another FF's Q via the SLICE AX/BX
# bypass input).  Such an arc explores effectively forever and, when skipped,
# leaves the FF's D-input MUX unbound -> the D pin floats (X in sim) -> the
# LFSR corrupts on silicon (the classic "stuck at 0x43" freeze).
#
# FFs whose D comes from a LUT.O6 route fine (FFMUX.O6 is set, no bypass).
# So: insert a LUT1 *buffer* (O = I0) into every FF->FF shift path, making
# EVERY flip-flop LUT-paired.  nextpnr then routes the whole design with 0
# skipped arcs and the physical netlist simulates clean (no X).
#
# Usage: insert_lutbuf.py in.json out.json
#
# LUT1 INIT is a 2-bit string, MSB-first (INIT[1]INIT[0]):
#   "01" = inverter (I0=0->1, I0=1->0)   "10" = buffer (I0=0->0, I0=1->1)

import json, sys
from collections import Counter

src, dst = sys.argv[1], sys.argv[2]
j = json.load(open(src))
m = next(iter(j['modules'].values()))
cells, nets = m['cells'], m.setdefault('netnames', {})

# bit -> type of the cell that drives it
drv = {}
for c in cells.values():
    for p, conn in c['connections'].items():
        if c.get('port_directions', {}).get(p) == 'output':
            for b in conn:
                if isinstance(b, int):
                    drv[b] = c['type']

maxbit = max(b for c in cells.values() for conn in c['connections'].values()
             for b in conn if isinstance(b, int))
nb = maxbit + 1

added, newcells = 0, {}
for cn, c in list(cells.items()):
    if c['type'] not in ('FDRE', 'FDSE', 'FDCE', 'FDPE'):
        continue
    dconn = c['connections'].get('D')
    if not dconn or not isinstance(dconn[0], int):
        continue
    db = dconn[0]
    if drv.get(db, '').startswith('LUT'):
        continue                       # already LUT-paired -> no bypass
    out = nb; nb += 1
    newcells['lutbuf_' + cn] = {
        'hide_name': 1, 'type': 'LUT1',
        'parameters': {'INIT': '10'},  # O = I0 (buffer)
        'attributes': {},
        'port_directions': {'I0': 'input', 'O': 'output'},
        'connections': {'I0': [db], 'O': [out]},
    }
    c['connections']['D'] = [out]
    nets['lutbuf_' + cn + '_o'] = {'hide_name': 1, 'bits': [out], 'attributes': {}}
    added += 1

cells.update(newcells)
json.dump(j, open(dst, 'w'))
print("inserted %d LUT1 buffers" % added)
print("cell counts:", dict(Counter(c['type'] for c in cells.values())))
