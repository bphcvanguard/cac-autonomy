"""Assign every exported part of 01Design_25 to a URDF link (by name AND position)."""
import json

import numpy as np

Y_MID = 280.5   # mm, rover centre plane; REP-103: +x forward (front wheel on the rocker), +y LEFT


def centre(r):
    b = r['bbox_mm']
    return np.array([(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2])


def is_wheel(r):
    b = r['bbox_mm']
    s = [b[3] - b[0], b[4] - b[1], b[5] - b[2]]
    return abs(s[0] - 220) < 3 and abs(s[1] - 60) < 3 and abs(s[2] - 220) < 3


def link_of(r):
    n, c = r['name'].strip(), centre(r)
    side = 'left' if c[1] > Y_MID else 'right'
    low = n.lower()
    if is_wheel(r):
        pos = 'front' if c[0] > 450 else ('middle' if c[0] > -100 else 'rear')
        return f'{side}_{pos}_wheel'
    if n in ('base_link', 'Component45', 'Component46', 'Component47', 'connecting pin',
             'connecting rods', 'connecting rods 2'):
        return 'base_link'
    if n in ('Component65', 'Component69', 'Component71'):
        return 'diff_bar'
    if (low.startswith('rod 3') or low.startswith('rod bet') or 'differential' in low
            or n in ('Component68', 'Component70', 'Component73', 'Component74', 'Component75',
                     'Component76 (1)', 'Diff bearing') or 'j2' in low):
        return f'{side}_rocker'
    if (low.startswith('rod 1') or low.startswith('rod 2') or 'plate 2' in low
            or n in ('Joint 2R', 'SOLID')):
        return f'{side}_bogie'
    # motors, motor caps and C-brackets travel with the leg that carries their wheel
    if 'c joint' in low or n.startswith('COMPOUND') or n.startswith('SOLID0'):
        return f'{side}_rocker' if c[0] > 450 else f'{side}_bogie'
    raise ValueError(f'unclassified part {n} at {c}')


if __name__ == '__main__':
    import sys
    rows = json.load(open(sys.argv[1]))
    groups = {}
    for r in rows:
        groups.setdefault(link_of(r), []).append(r)
    for k in sorted(groups):
        vol = sum(r['volume_mm3'] for r in groups[k]) / 1000
        names = ', '.join(r['name'][:18] for r in groups[k])
        print(f'{k:20s} {len(groups[k]):2d} parts  {vol:8.1f} cm3  ' + names)
