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
