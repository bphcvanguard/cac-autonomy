"""Layout module tests (no ROS)."""
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cac_sim_gen import layout as L  # noqa: E402, I100
from cac_sim_gen.config import load_config, load_layout  # noqa: E402
from cac_sim_gen.png import write_png  # noqa: E402
import numpy as np  # noqa: E402, I100, I201
import pytest  # noqa: E402, I201


@pytest.fixture(scope='module')
def cfg():
    return load_config()


@pytest.fixture(scope='module')
def layouts(cfg):
    return [L.random_layout(cfg, s) for s in range(200)]


def test_200_seeds_pass_strict(cfg, layouts):
    for lo in layouts:
        assert L.check_layout(cfg, lo, strict=True) == [], lo['name']
        assert L.path_exists(cfg, lo)[0]


def test_determinism(cfg):
    assert L.random_layout(cfg, 7) == L.random_layout(cfg, 7)
    assert L.random_layout(cfg, 7) != L.random_layout(cfg, 8)
    assert L.random_layout(cfg, 7)['name'] == 'seed_0007'


def test_start_pose_stream_independent(cfg):
    a = L.random_layout(cfg, 3)['start_pose']
    b = L.random_start_pose(cfg, np.random.default_rng([3, 3]))
    assert a == b


def test_nothing_in_start_or_construction_rims_included(cfg, layouts):
    z = cfg['zones']
    for lo in layouts:
        for (kind, _, x, y, r), e in zip(L._items(lo), L._extents(cfg, lo)):
            assert L._circle_rect_dist(x, y, z['start']) >= e
            assert L._circle_rect_dist(x, y, z['construction']) >= e


def test_no_straddling_zone_boundary(cfg, layouts):
    x5 = cfg['zones']['excavation'][0]
    for lo in layouts:
        for (kind, _, x, y, r), e in zip(L._items(lo), L._extents(cfg, lo)):
            assert x + e <= x5 or x - e >= x5


def test_straddle_reported(cfg):
    lo = L.random_layout(cfg, 4)
    lo = dict(lo, craters=lo['craters'] + [
        {'position': [4.95, 4.4], 'diameter': 0.4, 'depth': 0.2, 'rim': 0.03}])
    assert any('straddles' in v for v in L.check_layout(cfg, lo, strict=True, paths=False))


def test_slalom_required(cfg, layouts):
    for lo in layouts:
        assert L._straight_corridor(cfg, lo) is None
    lo = dict(L.random_layout(cfg, 4), boulders=[], craters=[])
    assert L._straight_corridor(cfg, lo) is not None
    assert any('straight' in v for v in L.check_layout(cfg, lo))


def test_se2_agrees_with_path_and_is_stricter(cfg, layouts):
    for lo in layouts[:40]:
        ok, path = L.se2_path_exists(cfg, lo)
        assert ok
        assert path[-1][0] >= cfg['zones']['excavation'][0] - 0.75 - 0.1
    for gap, expect in ((1.3, True), (0.8, False)):
        ys = [2.5 - gap / 2 - 0.2 - 0.4 * k for k in range(7)]
        ys += [2.5 + gap / 2 + 0.2 + 0.4 * k for k in range(7)]
        wall = [{'position': [2.9, y], 'diameter': 0.4, 'height': 0.2}
                for y in ys if 0.1 < y < 4.9]
        lo = {'name': 'gap', 'seed': None, 'boulders': wall, 'craters': [],
              'start_pose': {'x': 1.0, 'y': 1.0, 'yaw_deg': 0.0}}
        # only the gap may decide: park the column out of the way
        far = dict(cfg, column=dict(cfg['column'], centre=[7.5, 1.0]))
        assert L.se2_path_exists(far, lo)[0] == expect


def test_construction_boulders_option(cfg):
    import copy
    c2 = copy.deepcopy(cfg)
    c2['boulders']['in_construction_zone'] = [1, 2]
    z = c2['zones']['construction']
    for s in range(20):
        lo = L.random_layout(c2, s)
        n = sum(1 for k, _, x, y, _r in L._items(lo) if L._zone_of(c2, x, y) == 'construction')
        assert 1 <= n <= 2
        assert L.truth(c2, lo)['obstacles_in_construction_zone'] == n
        for (k, _, x, y, r), e in zip(L._items(lo), L._extents(cfg, lo)):
            if L._zone_of(c2, x, y) == 'construction':
                assert z[0] <= x - e and x + e <= z[2] and z[1] <= y - e and y + e <= z[3]


def test_start_footprint_inside_zone(cfg, layouts):
    x0, y0, x1, y1 = cfg['zones']['start']
    m = cfg['randomiser']['start_pose_margin']
    yaws = []
    for lo in layouts:
        yaws.append(lo['start_pose']['yaw_deg'])
        for cx, cy in L._footprint_corners(cfg, lo['start_pose']):
            assert x0 + m - 1e-9 <= cx <= x1 - m + 1e-9
            assert y0 + m - 1e-9 <= cy <= y1 - m + 1e-9
    assert min(yaws) < -90 and max(yaws) > 90


def test_schematic(cfg):
    # The digitised schematic must be physically valid (no overlaps, column clear, start pose ok).
    # Strict mode still flags that the drawing only has 2 obstacle-zone craters (rules: >= 3).
    s = load_layout('schematic')
    assert L.check_layout(cfg, s, strict=False) == []
    strict = L.check_layout(cfg, s, strict=True)
    assert any('2 craters in the obstacle zone' in v for v in strict)


def test_blocked_layout_has_no_path(cfg):
    lo = {'name': 'wall', 'seed': None,
          'boulders': [{'position': [3.5, 0.45 + 0.4 * k], 'diameter': 0.4, 'height': 0.2}
                       for k in range(12)],
          'craters': [], 'start_pose': {'x': 1.0, 'y': 1.0, 'yaw_deg': 0.0}}
    ok, path = L.path_exists(cfg, lo)
    assert not ok and path == []
    assert any('no path' in v for v in L.check_layout(cfg, lo, strict=False))
    assert not L.se2_path_exists(cfg, lo)[0]


def test_path_endpoints_and_inflate(cfg):
    lo = L.random_layout(cfg, 3)
    ok, path = L.path_exists(cfg, lo)
    assert ok
    x0, y0, x1, y1 = cfg['zones']['start']
    assert x0 <= path[0][0] <= x1 and y0 <= path[0][1] <= y1
    assert path[-1][0] >= cfg['zones']['excavation'][0]
    # a huge robot cannot pass
    assert not L.path_exists(cfg, lo, inflate=2.0)[0]


def test_hard_problems(cfg):
    lo = L.random_layout(cfg, 5)
    bad = dict(lo, boulders=lo['boulders'] + [dict(lo['boulders'][0])])
    assert any('overlaps' in v for v in L.check_layout(cfg, bad, strict=False))
    bad = dict(lo, start_pose={'x': 1.9, 'y': 1.0, 'yaw_deg': 0.0})
    assert any('start footprint' in v for v in L.check_layout(cfg, bad, strict=False))


def test_truth_and_map(cfg, tmp_path):
    import yaml
    lo = L.random_layout(cfg, 2)
    t = L.truth(cfg, lo)
    yaml.safe_dump(t)
    assert t['path_exists']['exists']
    assert len(t['obstacles']) == len(lo['boulders']) + len(lo['craters'])
    out = tmp_path / 'map.png'
    L.render_map(cfg, lo, str(out), path=t['path_exists']['path'])
    assert out.read_bytes()[:8] == b'\x89PNG\r\n\x1a\n'


def test_write_png(tmp_path):
    p = tmp_path / 'a.png'
    write_png(p, np.zeros((4, 6, 3), np.uint8))
    assert p.read_bytes()[:4] == b'\x89PNG'


def test_yaw_range(cfg):
    rng = np.random.default_rng(0)
    for _ in range(100):
        p = L.random_start_pose(cfg, rng)
        assert -180 <= p['yaw_deg'] < 180 and math.isfinite(p['x'])
