#!/usr/bin/env python3
"""Prepare 'NEW JOINTS KESHAV ASSEMBLY.step' for build_urdf.py.

That file's CAD frame has the rover along +-y and across x (z up). build_urdf.py expects the
01Design_25 convention: along x (bogie end at -x), across y. So every part is rotated +90 deg about
z: old = (-y_new, x_new, z_new). Duplicate parts (same name pattern, bbox and volume) and hardware
under 1 cm^3 are dropped. Each remaining part gets an explicit link, by name, from LINKS below;
sides are assigned by position, because the file mirrors parts with misleading names.

Writes OUT/parts.json (rows with an extra 'link' field) and OUT/parts_m/*.stl + index.json.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_urdf import read_stl, write_stl  # noqa: E402,I100

R = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # +90 deg about z
MIN_VOL_MM3 = 1000.0

# name (or name prefix) -> link kind. Side and wheel position are added from geometry.
LINKS = [
    ('base_link', 'base_link'), ('Component45', 'base_link'), ('Component46', 'base_link'),
    ('Component47', 'base_link'), ('connecting pin', 'base_link'),
    ('connecting rods', 'base_link'), ('90447A210', 'base_link'),
    ('Slotted Hex Head', 'base_link'),
    ('Component67', 'diff_bar'), ('Component73', 'diff_bar'),
    ('main joint', 'rocker'), ('BEARING SHAFT JOINT HOUSING', 'rocker'), ('SOLID001', 'rocker'),
    ('Component75', 'rocker'), ('Rod 3', 'rocker'), ('Rod bet', 'rocker'),
    ('Component103', 'rocker'), ('changed joint-differential', 'rocker'),
    ('Diff bearing', 'rocker'), ('Component72', 'rocker'), ('SOLID', 'rocker'),
    ('Rod 1', 'bogie'), ('Rod 2', 'bogie'), ('link2', 'bogie'), ('Component102', 'bogie'),
    ('Component126', 'bogie'), ('Component127', 'bogie'), ('Component130', 'bogie'),
    ('Component140', 'bogie'), ('Component141', 'bogie'), ('Component142', 'bogie'),
    ('Component143', 'bogie'), ('Component144', 'bogie'), ('Component145', 'bogie'),
    ('plates', 'bogie'),
    # motors, C-brackets and motor caps: travel with the leg that carries their wheel
    ('COMPOUND', 'motor'), ('Component81', 'motor'), ('Component001', 'motor'),
    ('Component002', 'motor'),
    ('Component85', 'wheel'), ('Component79', 'wheel'), ('Component80', 'wheel'),
]


def material_of(name, kind, vol_mm3):
    """Material per part (team: joints are PLA, rods carbon fibre). Others are ASSUMED."""
    n = name.strip()
    if n.startswith(('Rod ', 'connecting rods')):
        return 'cf_tube'
    if n.startswith(('90447A210', 'Slotted Hex Head', 'connecting pin')):
        return 'steel'
    if kind == 'wheel':
        return 'wheel'
    if kind == 'base_link' and n.startswith('base_link'):
        return 'body'
    if n.startswith('Component67'):                        # the differential bar (wood in renders)
        return 'wood'
    if n.startswith('COMPOUND') and vol_mm3 < 600e3:      # the 415 cm^3 block = drive motor
        return 'motor'
    return 'pla'                                           # joints, brackets, plates, caps, diff


def kind_of(name):
    n = name.strip()
    best = None
    for key, kind in LINKS:            # longest matching prefix wins ('SOLID001' before 'SOLID')
        if n.startswith(key) and (best is None or len(key) > len(best[0])):
            best = (key, kind)
    return best[1] if best else None


def main(src_json, src_dir, out):
    rows = json.load(open(src_json))
    index = json.load(open(os.path.join(src_dir, 'index.json')))
    files = {}
    for it in index:
        files.setdefault(it['label'], []).append(it['file'])
    used, seen, keep = {}, set(), []
    for r in rows:
        k = used.get(r['name'], 0)
        used[r['name']] = k + 1
        fn = files[r['name']][k]
        if r['volume_mm3'] < MIN_VOL_MM3:
            print('drop (small)   ', r['name'])
            continue
        sig = (tuple(round(v, 1) for v in r['bbox_mm']), round(r['volume_mm3'], 0))
        if sig in seen:
            print('drop (duplicate)', r['name'])
            continue
        seen.add(sig)
        kind = kind_of(r['name'])
        if kind is None:
            raise SystemExit(f'unclassified part: {r["name"]}')
        keep.append((r, fn, kind))
    os.makedirs(os.path.join(out, 'parts_m'), exist_ok=True)
    # rotate everything into the old convention
    out_rows, out_index, wheels = [], [], []
    for i, (r, fn, kind) in enumerate(keep):
        t = read_stl(os.path.join(src_dir, fn)).reshape(-1, 3) @ R.T
        lo, hi = t.min(0) * 1000, t.max(0) * 1000
        name = f'{r["name"].strip()} #{i}'
        nfn = f'p{i:03d}.stl'
        write_stl(os.path.join(out, 'parts_m', nfn), t.reshape(-1, 3, 3))
        row = {'name': name, 'volume_mm3': r['volume_mm3'], 'solids': r.get('solids', 1),
               'com_mm': list(np.asarray(r['com_mm']) @ R.T), 'parents': r.get('parents', []),
               'bbox_mm': [*lo, *hi], 'kind': kind,
               'material': material_of(r['name'], kind, r['volume_mm3'])}
        out_rows.append(row)
        out_index.append({'label': name, 'file': nfn})
        if kind == 'wheel':
            wheels.append((lo + hi) / 2)
    body = next(r for r in out_rows if r['name'].startswith('base_link'))
    y_mid = (body['bbox_mm'][1] + body['bbox_mm'][4]) / 2
    for row in out_rows:
        c = (np.array(row['bbox_mm'][:3]) + np.array(row['bbox_mm'][3:])) / 2
        side = 'left' if c[1] > y_mid else 'right'          # internal CAD sense (+y = 'left')
        kind = row.pop('kind')
        if kind in ('base_link', 'diff_bar'):
            row['link'] = kind
            continue
        if kind in ('wheel', 'motor'):
            same = [w for w in wheels if (w[1] > y_mid) == (side == 'left')]
            w = min(same, key=lambda w: abs(w[0] - c[0]))   # nearest wheel along the rover
            xs = sorted(x[0] for x in same)                  # rear(-x) .. front(+x) in CAD sense
            pos = {0: 'rear', 1: 'middle', 2: 'front'}[xs.index(w[0])]
            if kind == 'wheel':
                row['link'] = f'{side}_{pos}_wheel'
            else:
                row['link'] = f'{side}_rocker' if pos == 'front' else f'{side}_bogie'
            continue
        row['link'] = f'{side}_{kind}'
    json.dump(out_rows, open(os.path.join(out, 'parts.json'), 'w'), indent=1)
    json.dump(out_index, open(os.path.join(out, 'parts_m', 'index.json'), 'w'), indent=1)
    counts = {}
    for row in out_rows:
        counts[row['link']] = counts.get(row['link'], 0) + 1
    print(json.dumps(dict(sorted(counts.items())), indent=0))
    print('body centre y (old convention, mm):', round(y_mid, 1))


if __name__ == '__main__':
    main(*sys.argv[1:4])
