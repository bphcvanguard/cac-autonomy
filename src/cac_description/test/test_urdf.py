"""URDF sanity: strict XML, meshes exist, wheels on the ground at zero pose, valid inertias."""
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest

PKG = Path(__file__).resolve().parent.parent
URDF = PKG / 'urdf' / 'rover.urdf'


@pytest.fixture(scope='module')
def robot():
    return ET.parse(URDF).getroot()          # strict parser: catches '--' in comments etc.


def _rpy(r, p, y):
    cx, sx, cy, sy, cz, sz = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                     [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                     [-sy, cy * sx, cy * cx]])


def _zero_pose(robot):
    tf = {'base_footprint': np.eye(4)}
    joints = robot.findall('joint')
    while len(tf) < len(robot.findall('link')):
        for j in joints:
            p, c = j.find('parent').get('link'), j.find('child').get('link')
            if p in tf and c not in tf:
                o = j.find('origin')
                m = np.eye(4)
                m[:3, :3] = _rpy(*map(float, o.get('rpy').split()))
                m[:3, 3] = list(map(float, o.get('xyz').split()))
                tf[c] = tf[p] @ m
    return tf


def _stl(path):
    b = path.read_bytes()
    n = struct.unpack('<I', b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50],
                      dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3).astype(float)


def test_meshes_exist(robot):
    for m in robot.iter('mesh'):
        f = m.get('filename').replace('package://cac_description/', '')
        assert (PKG / f).is_file(), f


def test_six_wheels_touch_flat_ground_at_zero_pose(robot):
    tf = _zero_pose(robot)
    wheels = [ln for ln in robot.findall('link') if ln.get('name').endswith('_wheel')]
    assert len(wheels) == 6
    for ln in wheels:
        mesh = ln.find('visual/geometry/mesh').get('filename')
        f = mesh.replace('package://cac_description/', '')
        v = _stl(PKG / f) @ tf[ln.get('name')][:3, :3].T + tf[ln.get('name')][:3, 3]
        assert abs(v[:, 2].min()) < 0.002, ln.get('name')


def test_inertias_physical(robot):
    for ln in robot.findall('link'):
        i = ln.find('inertial')
        if i is None:
            continue
        assert float(i.find('mass').get('value')) > 0
        a = {k: float(v) for k, v in i.find('inertia').attrib.items()}
        m = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']],
                      [a['ixz'], a['iyz'], a['izz']]])
        ev = np.linalg.eigvalsh(m)
        assert ev.min() > 0, ln.get('name')
        assert ev[0] + ev[1] >= ev[2] * (1 - 1e-6), ln.get('name')   # triangle inequality


def test_mimic_targets_exist(robot):
    names = {j.get('name') for j in robot.findall('joint')}
    mimics = [m.get('joint') for m in robot.iter('mimic')]
    assert len(mimics) == 2
    assert all(m in names for m in mimics)


def test_frame_conventions(robot):
    """REP-103: front wheels at +x, left wheels at +y."""
    tf = _zero_pose(robot)
    assert tf['left_front_wheel'][0, 3] > tf['left_rear_wheel'][0, 3]
    assert tf['left_front_wheel'][1, 3] > 0 > tf['right_front_wheel'][1, 3]


SIM = PKG / 'urdf' / 'rover_sim.urdf'


def test_sim_variant_only_locks_the_differential():
    """rover_sim.urdf = rover.urdf with rockers + diff bar fixed and wheel friction added."""
    full, sim = ET.parse(URDF).getroot(), ET.parse(SIM).getroot()
    jf = {j.get('name'): j for j in full.findall('joint')}
    js = {j.get('name'): j for j in sim.findall('joint')}
    assert jf.keys() == js.keys()
    locked = {n for n, j in js.items() if j.get('type') == 'fixed'} - \
        {n for n, j in jf.items() if j.get('type') == 'fixed'}
    assert locked == {'left_rocker_joint', 'right_rocker_joint', 'diff_bar_joint'}
    assert not list(sim.iter('mimic'))
    for n in jf:                     # same frames everywhere
        assert jf[n].find('origin').attrib == js[n].find('origin').attrib, n
    lf = {ln.get('name'): ET.tostring(ln) for ln in full.findall('link')}
    ls = {ln.get('name'): ET.tostring(ln) for ln in sim.findall('link')}
    assert lf == ls                  # identical geometry, inertia, collisions


def test_sim_variant_wheel_friction():
    """Wheel friction tags, if any, cover every wheel (none = isotropic, the sand decides)."""
    sim = ET.parse(SIM).getroot()
    g = {e.get('reference'): e for e in sim.findall('gazebo')}
    wheels = [ln.get('name') for ln in sim.findall('link') if ln.get('name').endswith('_wheel')]
    assert not g or sorted(g) == sorted(wheels)


@pytest.mark.parametrize('path', [URDF, SIM])
def test_material_names_unique(path):
    """Reject duplicate material names: urdfdom (gz sdf, RViz) refuses such a URDF."""
    names = [m.get('name') for m in ET.parse(path).getroot().findall('material')]
    assert len(names) == len(set(names))


def _mimic(robot, joint):
    return next(float(m.get('multiplier')) for j in robot.findall('joint')
                if j.get('name') == joint for m in j.findall('mimic'))


def test_mimic_multipliers_and_signs(robot):
    """Rockers turn opposite ways (left = -1 x right); the diff bar follows the right rocker."""
    mimics = {j.get('name'): j.find('mimic') for j in robot.findall('joint')
              if j.find('mimic') is not None}
    assert set(mimics) == {'left_rocker_joint', 'diff_bar_joint'}
    assert mimics['left_rocker_joint'].get('joint') == 'right_rocker_joint'
    assert mimics['diff_bar_joint'].get('joint') == 'right_rocker_joint'
    assert _mimic(robot, 'left_rocker_joint') == pytest.approx(-1.0)
    assert 0.35 <= _mimic(robot, 'diff_bar_joint') <= 0.5


def test_left_right_symmetry(robot):
    tf = _zero_pose(robot)
    links = {ln.get('name'): ln for ln in robot.findall('link')}
    for name, ln in links.items():
        if not name.startswith('left_'):
            continue
        other = links['right_' + name[len('left_'):]]
        m1 = float(ln.find('inertial/mass').get('value'))
        m2 = float(other.find('inertial/mass').get('value'))
        assert m1 == pytest.approx(m2, abs=1e-3), name
        a, b = tf[name][:3, 3], tf['right_' + name[len('left_'):]][:3, 3]
        assert a[0] == pytest.approx(b[0], abs=1e-3), name
        assert a[1] == pytest.approx(-b[1], abs=1e-3), name
        assert a[2] == pytest.approx(b[2], abs=1e-3), name


def _boxes(ln):
    """Corners of every collision box of a link, in the link frame."""
    out = []
    for c in ln.findall('collision'):
        b = c.find('geometry/box')
        if b is None:
            continue
        o = c.find('origin')
        rot = _rpy(*map(float, o.get('rpy').split()))
        half = np.array(b.get('size').split(), float) / 2
        k = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        out.append((k * half) @ rot.T + np.array(o.get('xyz').split(), float))
    return out


def _visual_pts(ln):
    return np.vstack([_stl(PKG / m.get('filename').replace('package://cac_description/', ''))
                      for m in ln.findall('visual/geometry/mesh')])


STRUCT = ['base_link', 'diff_bar', 'left_rocker', 'right_rocker', 'left_bogie', 'right_bogie']


def test_collision_boxes_inside_visual_bounds(robot):
    links = {ln.get('name'): ln for ln in robot.findall('link')}
    for name in STRUCT:
        v = _visual_pts(links[name])
        lo, hi = v.min(0) - 0.005, v.max(0) + 0.005
        for corners in _boxes(links[name]):
            assert (corners.min(0) >= lo).all() and (corners.max(0) <= hi).all(), name


def test_collision_envelope_not_wider_or_lower_than_visual(robot):
    tf = _zero_pose(robot)
    links = {ln.get('name'): ln for ln in robot.findall('link')}
    col, vis = [], []
    for name in STRUCT:
        rot, p = tf[name][:3, :3], tf[name][:3, 3]
        col += [c @ rot.T + p for c in _boxes(links[name])]
        vis.append(_visual_pts(links[name]) @ rot.T + p)
        v_low = vis[-1][:, 2].min()
        c_low = min((c @ rot.T + p)[:, 2].min() for c in _boxes(links[name]))
        assert c_low >= v_low - 0.005, name
    col, vis = np.vstack(col), np.vstack(vis)
    assert np.ptp(col[:, 1]) <= np.ptp(vis[:, 1]) + 0.010
    assert col[:, 2].min() >= vis[:, 2].min() - 0.005


def test_readme_states_build_summary_mass():
    import json
    total = json.loads((PKG / 'urdf' / 'build_summary.json').read_text())['total_mass_kg']
    assert f'{total:.1f} kg' in (PKG / 'README.md').read_text()
