#!/usr/bin/env python3
"""Build the rover URDF from the per-part STL export of 01Design_25.step.

Inputs: parts.json (inspect_step.py) and parts_m/ (export_parts.py, STLs in metres, CAD frame).
CAD frame: x along the rover, y across (rocker pivots at y = 0.5 and 560.5 mm), z up, millimetres
in the STEP (converted to metres at export).

Zero pose = the physical standing pose on flat ground: rockers at their CAD angle (the
differential is neutral there), each bogie rotated about its pin until its two wheels line up
with the front wheel. On flat ground the body then sits pitched (see STANCE in the output).

Usage: build_urdf.py PARTS_JSON PARTS_DIR OUT_PKG_DIR [--front -x|+x] [--total-mass KG]
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from classify import centre, link_of  # noqa: E402,I100

MM = 0.001
WHEEL_R, WHEEL_W = 0.110, 0.060
ROCKER_PIVOT = {'right': np.array([385.1, 0.5, 169.0]) * MM,
                'left': np.array([385.1, 560.5, 169.0]) * MM}
BOGIE_PIVOT = {'right': np.array([-130.8, -22.0, -10.3]) * MM,
               'left': np.array([-137.4, 583.0, -4.1]) * MM}
DIFF_PIVOT = np.array([479.2, 280.5, 312.0]) * MM
# Rocker-side ball joint of each differential link and the diff-bar end it drives (CAD, mm).
DIFF_LINK_ON_ROCKER = np.array([452.0, -25.0, 295.0]) * MM
DIFF_BAR_END = np.array([474.0, -25.0, 316.0]) * MM

# ASSUMED effective densities (kg/m^3): solids in the CAD are idealised (e.g. the body is a solid
# block, wheels are solid), so these are "mass per CAD volume". Replace by --total-mass scaling.
DENSITY = {
    'base_link': 700.0,     # 6.4 L of CAD solid: aluminium frame + plates + electronics (air)
    'rocker': 1500.0,       # aluminium rods + 3D-printed brackets + front motor
    'bogie': 1500.0,        # rods + two motors and C-brackets
    'wheel': 450.0,         # 2.24 L solid wheel in CAD; printed/rubber wheel, mostly hollow
    'diff_bar': 1200.0,
}
# Visual colour per material tag (looks like the team's renders/photos), used when parts carry
# a material; otherwise COLOUR by link kind.
MAT_COLOUR = {'pla': (0.93, 0.93, 0.91, 1), 'cf_tube': (0.07, 0.07, 0.08, 1),
              'wheel': (0.11, 0.11, 0.12, 1), 'body': (0.78, 0.79, 0.80, 1),
              'steel': (0.55, 0.56, 0.58, 1), 'motor': (0.30, 0.31, 0.33, 1),
              'wood': (0.62, 0.47, 0.32, 1)}
COLOUR = {'base_link': (0.55, 0.57, 0.6, 1), 'rocker': (0.85, 0.45, 0.1, 1),
          'bogie': (0.16, 0.45, 0.75, 1), 'wheel': (0.12, 0.12, 0.12, 1),
          'diff_bar': (0.75, 0.2, 0.17, 1)}


def kind(link):
    for k in ('wheel', 'rocker', 'bogie', 'diff_bar'):
        if link.endswith(k) or link == k:
            return k
    return 'base_link'


def read_stl(fn):
    b = open(fn, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:400]:
        v = [list(map(float, ln.split()[1:4])) for ln in b.decode().splitlines()
             if ln.strip().startswith('vertex')]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack('<I', b[80:84])[0]
    dt = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    a = np.frombuffer(b[84:84 + n * 50], dtype=dt)
    return a['v'].reshape(-1, 3, 3).astype(float)


def write_stl(fn, tris):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.where(ln > 0, n / np.maximum(ln, 1e-30), 0)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['n'] = n
    rec['v'] = tris.reshape(-1, 9)
    with open(fn, 'wb') as f:
        f.write(b'cac_description rover mesh (metres)'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(rec.tobytes())


def mass_props(tris):
    """Volume, centroid and inertia (about the centroid, unit density) of a closed mesh."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    v6 = np.einsum('ij,ij->i', a, np.cross(b, c))          # 6 x signed tetra volume (origin apex)
    vol = v6.sum() / 6.0
    if abs(vol) < 1e-12:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    com = (v6[:, None] * (a + b + c)).sum(0) / (24.0 * vol)
    # second moments of the tetrahedra (origin, a, b, c)

    def sq(i, j):
        return (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j]
                + (a[:, i] + b[:, i] + c[:, i]) * (a[:, j] + b[:, j] + c[:, j])) / 2.0
    C = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            C[i, j] = (v6 * sq(i, j)).sum() / 60.0          # covariance about the origin
    if vol < 0:
        vol, C = -vol, -C
    C = C - vol * np.outer(com, com)                          # shift covariance to the centroid
    inertia = np.trace(C) * np.eye(3) - C
    return vol, com, inertia / vol                            # inertia per unit mass


def roty(p, ctr, t):
    R = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
    return (p - ctr) @ R.T + ctr


def solve_bogie_stance(wheel_c, side):
    """Bogie rotation (rad, about its pin) puts its two wheel centres on the front wheel line."""
    from scipy.optimize import brentq
    ws = sorted([w for w in wheel_c
                 if (w[1] > (ROCKER_PIVOT['left'][1] + ROCKER_PIVOT['right'][1]) / 2)
                 == (side == 'left')],
                key=lambda w: -w[0])
    F, M, R = ws
    J = BOGIE_PIVOT[side]

    def cross(t):
        m, r = roty(M, J, t), roty(R, J, t)
        return (m[0] - F[0]) * (r[2] - F[2]) - (m[2] - F[2]) * (r[0] - F[0])
    return brentq(cross, -0.6, 0.6)


# Effective densities (kg/m^3 of CAD solid volume) per material tag (prep_*.py). ASSUMED unless
# a model file overrides them.
MATERIAL_DENSITY = {
    'pla': 0.6 * 1240.0,           # printed parts: walls + infill ~60 % of solid PLA
    'cf_tube': 0.36 * 1550.0,      # CAD rods are solid 20 mm bars; a 20 x 16 mm tube is 36 %
    'steel': 7850.0,
    'motor': 1450.0,               # ~0.6 kg drive motor in a 415 cm^3 CAD block
    'wheel': 450.0,
    'body': 700.0,
    'wood': 600.0,                 # the differential bar (plywood-like, ASSUMED)
}


def load_model(path):
    """Override pivots (mm) and material densities from a model JSON."""
    m = json.load(open(path))
    for name, table in (('rocker_pivot', ROCKER_PIVOT), ('bogie_pivot', BOGIE_PIVOT)):
        for side in ('left', 'right'):
            if side in m.get(name, {}):
                table[side] = np.array(m[name][side], float) * MM
    g = globals()
    for key, var in (('diff_pivot', 'DIFF_PIVOT'), ('diff_link_on_rocker', 'DIFF_LINK_ON_ROCKER'),
                     ('diff_bar_end', 'DIFF_BAR_END')):
        if key in m:
            g[var] = np.array(m[key], float) * MM
    MATERIAL_DENSITY.update(m.get('density', {}))


def mirror_side(parts, source):
    """Replace every rocker/bogie/wheel part of the other side by a mirror image of `source`'s.

    The mirror plane is the body's centre plane (y of the base_link part's bounding-box centre).
    Reflection flips triangle winding, so each mirrored triangle is re-ordered to keep its normal
    pointing outwards (mass_props needs a positive volume). The body and the diff bar are kept.
    """
    other = 'right' if source == 'left' else 'left'
    body = [r for lk, _, r in parts
            if lk == 'base_link' and r['name'].strip().split(' #')[0] == 'base_link']
    y0 = centre(body[0])[1] * MM
    for piv in (ROCKER_PIVOT, BOGIE_PIVOT):      # the pivots move with the mirrored side
        piv[other] = piv[source] * np.array([1.0, -1.0, 1.0]) + np.array([0.0, 2 * y0, 0.0])
    keep = [(lk, t, r) for lk, t, r in parts if not lk.startswith(other + '_')]
    added = []
    for lk, t, r in parts:
        if not lk.startswith(source + '_'):
            continue
        m = t.copy()
        m[:, :, 1] = 2 * y0 - m[:, :, 1]
        m = m[:, ::-1, :]                                  # restore outward winding
        b = list(r['bbox_mm'])
        r2 = dict(r, name=r['name'] + ' (mirrored)',
                  bbox_mm=[b[0], 2 * y0 / MM - b[4], b[2], b[3], 2 * y0 / MM - b[1], b[5]])
        added.append((other + lk[len(source):], m, r2))
    return keep + added


# Wheel friction for driving in gz-sim 8 DART. None = no tags: isotropic, so the sand's friction
# (DART takes the lower mu) sets the grip. Measured in the cac_sim arena with the new-joints rover
# (wheelbase 0.89 m, track 0.65 m): isotropic wheels turn 360.3-360.5 of 360 deg in place with
# odometry exact at the 0.647 m track. The OLD geometry (1.16 m on a 0.57 m track) could only turn
# with anisotropic wheels: {'mu1': 0.2, 'mu2': 1.0, 'fdir1': '0 0 1'} (fdir1 = axle).
SIM_WHEEL_FRICTION = None


def sim_variant(urdf_text, lock):
    """rover_sim.urdf: the same robot for DRIVING in gz-sim 8 on DART.

    DART ignores URDF mimic joints, so a free differential lets the body tip over. Here the rocker
    joints and the diff bar are fixed (the bogies still articulate) and the wheels get the
    anisotropic friction that lets the skid-steer turn. Everything else is byte-identical.
    """
    import re
    out = urdf_text
    for name in lock:
        pat = re.compile(r'(<joint name="%s" type=")revolute(">.*?</joint>)' % re.escape(name),
                         re.S)
        m = pat.search(out)
        assert m, name
        body = re.sub(r'<axis[^>]*/>|<limit[^>]*/>|<dynamics[^>]*/>|<mimic[^>]*/>', '', m.group(2))
        out = out[:m.start()] + m.group(1) + 'fixed' + body + out[m.end():]
    gz = []
    for wheel in re.findall(r'<link name="([a-z_]+_wheel)">', out) if SIM_WHEEL_FRICTION else []:
        f = SIM_WHEEL_FRICTION
        gz.append(f'  <gazebo reference="{wheel}"><mu1>{f["mu1"]}</mu1><mu2>{f["mu2"]}</mu2>'
                  f'<fdir1>{f["fdir1"]}</fdir1></gazebo>')
    if gz:
        out = out.replace('</robot>', '\n'.join(gz) + '\n</robot>')
    note = ('<!-- rover_sim.urdf: DRIVING variant for gz-sim 8 DART. Rockers and diff bar are'
            ' FIXED\n     (DART ignores mimic joints; a free differential tips the body over);'
            ' bogies still\n     articulate. Wheel friction: see SIM_WHEEL_FRICTION in'
            ' tools/build_urdf.py.\n     Use rover.urdf for RViz and suspension work. -->\n')
    return out.replace('<robot ', note + '<robot ', 1)


def fmt(v):
    return ' '.join(f'{x:.6g}' for x in v)


MIN_COLLISION_VOL = 15e-6    # m^3: parts smaller than 15 cm^3 (pins, bolts) get no collision box


def obb(points):
    """Oriented bounding box via PCA: (centre, rotation with box axes as columns, size)."""
    c0 = points.mean(0)
    _, vecs = np.linalg.eigh(np.cov((points - c0).T))
    if np.linalg.det(vecs) < 0:
        vecs[:, 0] = -vecs[:, 0]
    loc = (points - c0) @ vecs
    lo, hi = loc.min(0), loc.max(0)
    return c0 + vecs @ ((lo + hi) / 2), vecs, hi - lo


def rpy_of(rot):
    return (math.atan2(rot[2, 1], rot[2, 2]), math.asin(max(-1.0, min(1.0, -rot[2, 0]))),
            math.atan2(rot[1, 0], rot[0, 0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('parts_json')
    ap.add_argument('parts_dir')
    ap.add_argument('out_pkg')
    ap.add_argument('--front', choices=('-x', '+x'), default='-x',
                    help='which CAD end is the front (default -x: wheel/rod 1 end)')
    ap.add_argument('--total-mass', type=float, default=None, help='scale all masses to this (kg)')
    ap.add_argument('--model', default=None,
                    help="JSON with pivots (mm, this tool's CAD convention) and material "
                         'densities for a CAD file other than 01Design_25')
    ap.add_argument('--mirror-from', choices=('left', 'right'), default=None,
                    help='rebuild the other side as a mirror image of this CAD side '
                         '(internal CAD sense: left = CAD +y). Use when one side has newer joints')
    a = ap.parse_args()

    if a.model:
        load_model(a.model)
    rows = json.load(open(a.parts_json))
    index = json.load(open(os.path.join(a.parts_dir, 'index.json')))
    files = {}
    for it in index:
        files.setdefault(it['label'], []).append(it['file'])
    # pair parts.json rows with exported files (labels repeat; keep order of appearance)
    used = {}
    parts = []
    for r in rows:
        lbl = r['name']
        k = used.get(lbl, 0)
        used[lbl] = k + 1
        tris = read_stl(os.path.join(a.parts_dir, files[lbl][k]))
        parts.append((r['link'] if 'link' in r else link_of(r), tris, r))
    if a.mirror_from:
        parts = mirror_side(parts, a.mirror_from)

    wheel_c = [centre(r) * MM for (lk, _, r) in parts if lk.endswith('wheel')]
    beta = {s: solve_bogie_stance(wheel_c, s) for s in ('left', 'right')}

    # Link frames in the CAD frame (metres), after the bogie stance rotation.
    frames = {'base_link': (ROCKER_PIVOT['left'] + ROCKER_PIVOT['right']) / 2,
              'diff_bar': DIFF_PIVOT}
    for s in ('left', 'right'):
        frames[f'{s}_rocker'] = ROCKER_PIVOT[s]
        frames[f'{s}_bogie'] = BOGIE_PIVOT[s]
    link_tris, link_mats = {}, {}
    for lk, tris, r in parts:
        side = lk.split('_')[0]
        if lk.endswith('bogie') or lk in (f'{side}_middle_wheel', f'{side}_rear_wheel'):
            tris = roty(tris.reshape(-1, 3), BOGIE_PIVOT[side], beta[side]).reshape(-1, 3, 3)
        link_tris.setdefault(lk, []).append(tris)
        link_mats.setdefault(lk, []).append(r.get('material'))
        if lk.endswith('wheel'):
            frames[lk] = tris.reshape(-1, 3).min(0) / 2 + tris.reshape(-1, 3).max(0) / 2
    # Base-frame orientation: front=-x means the robot's +x is CAD -x and +y is CAD -y.
    flip = np.diag([-1.0, -1.0, 1.0]) if a.front == '-x' else np.eye(3)

    def to_base(p):          # CAD point (m) -> vector in the robot's axes
        return p @ flip.T

    # Mass properties per link (in the link frame, robot axes).
    props = {}
    for lk, tl in link_tris.items():
        m_tot, mom, I_tot = 0.0, np.zeros(3), np.zeros((3, 3))
        items = []
        for t in tl:
            vol, com, I_unit = mass_props(t)
            mat = link_mats[lk][len(items)]
            m = vol * (MATERIAL_DENSITY[mat] if mat else DENSITY[kind(lk)])
            items.append((m, com, I_unit))
            m_tot += m
            mom += m * com
        com_link = mom / m_tot
        for m, com, I_unit in items:
            d = com - com_link
            I_tot += m * I_unit + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        props[lk] = [m_tot, to_base(com_link - frames[lk]), flip @ I_tot @ flip.T]
    if a.total_mass:
        k = a.total_mass / sum(p[0] for p in props.values())
        for p in props.values():
            p[0] *= k
            p[2] = p[2] * k

    # Meshes (link frame, robot axes).
    mesh_dir = os.path.join(a.out_pkg, 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    vis_parts = {}           # link -> [(material, mesh file stem)] for per-material visuals
    for lk, tl in link_tris.items():
        t = np.concatenate(tl).reshape(-1, 3)
        write_stl(os.path.join(mesh_dir, f'{lk}.stl.tmp'),
                  to_base(t - frames[lk]).reshape(-1, 3, 3))
        mats = link_mats.get(lk, [None] * len(tl))
        groups = {}
        for tri, mat in zip(tl, mats):
            groups.setdefault(mat, []).append(tri)
        if len(groups) > 1 and None not in groups:
            vis_parts[lk] = []
            for mat, g in sorted(groups.items()):
                t = np.concatenate(g).reshape(-1, 3)
                write_stl(os.path.join(mesh_dir, f'{lk}__{mat}.stl.tmp'),
                          to_base(t - frames[lk]).reshape(-1, 3, 3))
                vis_parts[lk].append((mat, f'{lk}__{mat}'))
        elif None not in groups:
            vis_parts[lk] = [(next(iter(groups)), lk)]

    # Differential gear ratio from the linkage geometry: a rocker rotation q moves its ball joint
    # along x by z_lever * q; the bar end at lateral lever L turns the bar by that / L.
    z_lever = (DIFF_LINK_ON_ROCKER - ROCKER_PIVOT['right'])[2]
    lat = abs(DIFF_BAR_END[1] - DIFF_PIVOT[1])
    ratio = z_lever / lat
    # sign of the bar end's robot y for the master rocker
    dsign = -1.0 if a.front == '+x' else 1.0

    # Stance: ground under the wheels; base_footprint below base_link, x along the ground.
    wc = {k: frames[k] for k in frames if k.endswith('wheel')}
    fr = np.mean([wc[k] for k in wc if 'front' in k], 0)
    rr = np.mean([wc[k] for k in wc if 'rear' in k], 0)
    pitch_cad = math.atan2(rr[2] - fr[2], fr[0] - rr[0])        # wheel line slope in the CAD frame
    # rotate the CAD frame by pitch_cad about y so the wheel line is horizontal; ground = axle - r
    # Ry(-pitch_cad): brings the higher (rear-in-CAD) end down so the wheel line is horizontal
    Rg = np.array([[math.cos(pitch_cad), 0, -math.sin(pitch_cad)], [0, 1, 0],
                   [math.sin(pitch_cad), 0, math.cos(pitch_cad)]])
    axle_z = (Rg @ fr)[2]
    base_g = Rg @ frames['base_link']
    height_above_ground = base_g[2] - (axle_z - WHEEL_R)
    all_pts = np.concatenate([np.concatenate(tl).reshape(-1, 3)
                              for tl in link_tris.values()]) @ Rg.T
    ground = axle_z - WHEEL_R
    extent = all_pts.max(0) - all_pts.min(0)
    # base_link orientation in the ground frame is Rg = Ry(-pitch_cad) in CAD axes; flipping x
    # and y (front = -x) reverses the y axis, so the same tilt is +pitch_cad about the robot's y.
    stance_pitch_robot = -pitch_cad if a.front == '+x' else pitch_cad

    swap = {'left': 'right', 'right': 'left', 'front': 'rear', 'rear': 'front'}

    def pub(name):           # internal (CAD-sense) link name -> robot-sense name
        if a.front == '+x':
            return name
        return '_'.join(swap.get(t, t) for t in name.split('_'))

    def joint_xyz(child, parent):
        return to_base(frames[child] - frames[parent])

    L = []
    w = L.append
    w('<?xml version="1.0"?>')
    w('<!-- GENERATED by tools/build_urdf.py from 01Design_25.step (old Vanguard rover).'
      ' Do not edit')
    w('     by hand: change the generator and re-run. Units: metres, kg.'
      ' REP-103: x forward, y left, z up.')
    mass_note = (f'scaled to {a.total_mass} kg' if a.total_mass else
                 'ASSUMED densities (see generator); rescale with the total-mass option')
    w(f'     Front = CAD {a.front}. Masses: {mass_note}.')
    w('     Zero pose = standing on flat ground: rockers at the CAD angle'
      ' (differential neutral), bogies')
    w(f'     rotated {math.degrees(beta["left"]):+.2f} / {math.degrees(beta["right"]):+.2f}'
      ' deg (left/right) from the CAD. On flat ground the body')
    w(f'     pitches {math.degrees(stance_pitch_robot):+.2f} deg about y'
      ' (base_footprint -> base_link). -->')
    w('<robot name="vanguard_rover">')
    for k, rgba in COLOUR.items():
        w(f'  <material name="{k}"><color rgba="{fmt(rgba)}"/></material>')
    for k, rgba in MAT_COLOUR.items():       # separate names: 'wheel' exists in both tables
        w(f'  <material name="mat_{k}"><color rgba="{fmt(rgba)}"/></material>')
    w('  <link name="base_footprint"/>')
    w('  <joint name="base_footprint_joint" type="fixed">')
    w('    <parent link="base_footprint"/><child link="base_link"/>')
    w(f'    <origin xyz="0 0 {height_above_ground:.6g}"'
      f' rpy="0 {stance_pitch_robot:.6g} 0"/>')
    w('  </joint>')
    order = ['base_link', 'diff_bar', 'left_rocker', 'right_rocker', 'left_bogie',
             'right_bogie'] + \
        [f'{s}_{p}_wheel' for s in ('left', 'right') for p in ('front', 'middle', 'rear')]
    for lk in order:
        m, com, inr = props[lk]
        w(f'  <link name="{pub(lk)}">')
        w(f'    <inertial><origin xyz="{fmt(com)}" rpy="0 0 0"/><mass value="{m:.6g}"/>')
        w(f'      <inertia ixx="{inr[0, 0]:.6g}" ixy="{inr[0, 1]:.6g}" ixz="{inr[0, 2]:.6g}"'
          f' iyy="{inr[1, 1]:.6g}" iyz="{inr[1, 2]:.6g}" izz="{inr[2, 2]:.6g}"/></inertial>')
        mesh = ('<geometry><mesh filename="package://cac_description/meshes/'
                f'{pub(lk)}.stl"/></geometry>')
        if lk in vis_parts:
            for mat, stem in vis_parts[lk]:
                base, _, mat_sfx = stem.partition('__')
                pstem = pub(base) + ('__' + mat_sfx if mat_sfx else '')
                w('    <visual><geometry><mesh filename="package://cac_description/meshes/'
                  f'{pstem}.stl"/></geometry><material name="mat_{mat}"/></visual>')
        else:
            w(f'    <visual>{mesh}<material name="{kind(lk)}"/></visual>')
        if kind(lk) == 'wheel':
            w(f'    <collision><origin xyz="0 0 0" rpy="{math.pi / 2:.6g} 0 0"/>'
              f'<geometry><cylinder radius="{WHEEL_R}" length="{WHEEL_W}"/></geometry>'
              '</collision>')
        else:
            # one oriented box per solid part: cheap for the physics engine (full meshes ran the
            # arena at 0.05x real time) but still tight around every rod, plate and motor
            for t in link_tris[lk]:
                vol = abs(mass_props(t)[0])
                if vol < MIN_COLLISION_VOL:
                    continue
                pts = to_base(t.reshape(-1, 3) - frames[lk])
                c, rot, size = obb(pts)
                w(f'    <collision><origin xyz="{fmt(c)}" rpy="{fmt(rpy_of(rot))}"/>'
                  f'<geometry><box size="{fmt(np.maximum(size, 0.002))}"/></geometry></collision>')
        w('  </link>')

    def joint(name, jtype, parent, child, axis, extra=''):
        w(f'  <joint name="{pub(name)}" type="{jtype}">')
        w(f'    <parent link="{pub(parent)}"/><child link="{pub(child)}"/>')
        w(f'    <origin xyz="{fmt(joint_xyz(child, parent))}" rpy="0 0 0"/>'
          f'<axis xyz="{axis}"/>{extra}')
        w('  </joint>')
    lim = '<limit lower="-0.6" upper="0.6" effort="500" velocity="2"/><dynamics damping="0.5"/>'
    joint('left_rocker_joint', 'revolute', 'base_link', 'left_rocker', '0 1 0', lim)
    # differential: the rockers turn opposite ways relative to the body
    joint('right_rocker_joint', 'revolute', 'base_link', 'right_rocker', '0 1 0',
          lim + f'<mimic joint="{pub("left_rocker_joint")}" multiplier="-1" offset="0"/>')
    # diff bar about z; sign: left rocker +q moves its ball joint along robot +x by z_lever*q,
    # which turns the bar end at +y by -(z_lever*q)/lat about +z
    joint('diff_bar_joint', 'revolute', 'base_link', 'diff_bar', '0 0 1',
          '<limit lower="-0.5" upper="0.5" effort="100" velocity="2"/>'
          f'<mimic joint="{pub("left_rocker_joint")}"'
          f' multiplier="{dsign * ratio:.6g}" offset="0"/>')
    blim = '<limit lower="-0.8" upper="0.8" effort="500" velocity="2"/><dynamics damping="0.5"/>'
    for s in ('left', 'right'):
        joint(f'{s}_bogie_joint', 'revolute', f'{s}_rocker', f'{s}_bogie', '0 1 0', blim)
        for p, parent in (('front', f'{s}_rocker'), ('middle', f'{s}_bogie'),
                          ('rear', f'{s}_bogie')):
            joint(f'{s}_{p}_wheel_joint', 'continuous', parent, f'{s}_{p}_wheel', '0 1 0',
                  '<limit effort="20" velocity="20"/><dynamics damping="0.05"/>')
    w('</robot>')
    os.makedirs(os.path.join(a.out_pkg, 'urdf'), exist_ok=True)
    with open(os.path.join(a.out_pkg, 'urdf', 'rover.urdf'), 'w') as f:
        f.write('\n'.join(L) + '\n')
    with open(os.path.join(a.out_pkg, 'urdf', 'rover_sim.urdf'), 'w') as f:
        locked = [pub('left_rocker_joint'), pub('right_rocker_joint'), 'diff_bar_joint']
        f.write(sim_variant('\n'.join(L) + '\n', locked))

    for lk in link_tris:
        os.replace(os.path.join(mesh_dir, f'{lk}.stl.tmp'),
                   os.path.join(mesh_dir, f'{pub(lk)}.stl'))
        for mat, stem in vis_parts.get(lk, []):
            if '__' in stem:
                os.replace(os.path.join(mesh_dir, f'{stem}.stl.tmp'),
                           os.path.join(mesh_dir, f'{pub(lk)}__{mat}.stl'))
    summary = {
        'front': a.front, 'total_mass_kg': round(sum(p[0] for p in props.values()), 3),
        'masses_kg': {pub(k): round(props[k][0], 3) for k in order},
        'bogie_stance_deg': {k: round(math.degrees(v), 3) for k, v in beta.items()},
        'stance_pitch_deg': round(math.degrees(stance_pitch_robot), 3),
        'base_link_height_m': round(height_above_ground, 4),
        'diff_ratio': round(ratio, 4),
        'extent_in_stance_m': [round(x, 4) for x in extent],
        'top_above_ground_m': round(all_pts[:, 2].max() - ground, 4),
        'wheel_centres_robot_m': {pub(k): [round(x, 4) for x in to_base(v - frames['base_link'])]
                                  for k, v in wc.items()},
    }
    json.dump(summary, open(os.path.join(a.out_pkg, 'urdf', 'build_summary.json'), 'w'), indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
