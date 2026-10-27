"""Layout module tests (no ROS)."""
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cac_sim_gen import layout as L  # noqa: E402, I100
from cac_sim_gen.config import load_config, load_layout  # noqa: E402
from cac_sim_gen.geometry import write_png  # noqa: E402
from cac_sim_gen.markers import post_circles  # noqa: E402
import numpy as np  # noqa: E402, I100, I201
import pytest  # noqa: E402, I201


@pytest.fixture(scope='module')
def cfg():
    return load_config()


@pytest.fixture(scope='module')
def layouts(cfg):
    # 60 seeds on every PR keeps CI quick; set CAC_SIM_SEEDS=500 (or more) for a full sweep
    # after changing the randomiser or arena.yaml limits.
    return [L.random_layout(cfg, s) for s in range(int(os.environ.get('CAC_SIM_SEEDS', 60)))]


def test_seeds_pass_strict(cfg, layouts):
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


def test_posts_exist_in_default_config(cfg):
    posts = post_circles(cfg)
    assert posts and all(0 < r <= 0.02 for *_, r in posts)


def test_posts_are_obstacles_in_every_path_check(cfg, layouts):
    """No seed's truth path (disc or SE(2)) touches a marker post, with or without --markers."""
    posts = [(x, y, r) for _, x, y, r in post_circles(cfg)]
    inflate = L._default_inflate(cfg)
    for lo in layouts[:60]:
        t = L.truth(cfg, lo)
        pts = t['path_exists']['path']
        for a, b in zip(pts[:-1], pts[1:]):
            for k in range(21):
                x, y = a[0] + (b[0] - a[0]) * k / 20, a[1] + (b[1] - a[1]) * k / 20
                for px, py, pr in posts:
                    assert math.hypot(x - px, y - py) >= pr + inflate - 0.05, lo['name']
        for x, y, yaw in t['se2_path']:
            for px, py, pr in posts:
                assert not L._box_hits_circle(cfg, {'x': x, 'y': y, 'yaw_deg': yaw}, px, py, pr), \
                    lo['name']


def test_post_blocks_path_check(cfg):
    """A layout whose only gap is plugged by a post has no path; truth lists posts if asked."""
    import copy
    c2 = copy.deepcopy(cfg)
    ex = c2['zones']['excavation'][0]
    c2['markers']['list'] = [{'id': y, 'position': [ex - 0.5, y / 10, 0.35], 'yaw_deg': 0}
                             for y in range(0, 51, 2)]
    lo = {'name': 'x', 'seed': None, 'boulders': [], 'craters': [],
          'start_pose': {'x': 1.0, 'y': 1.0, 'yaw_deg': 0.0}}
    assert not L.path_exists(c2, lo)[0]
    assert any('no path' in v for v in L.check_layout(c2, lo, strict=False))
    assert L.path_exists(cfg, lo)[0]
    assert L.truth(cfg, lo)['marker_posts'] == []
    assert len(L.truth(cfg, lo, markers=True)['marker_posts']) == len(post_circles(cfg))
    assert L.truth(cfg, lo)['paths_include_marker_posts']


def test_obstacle_overlapping_post_is_hard_error(cfg):
    _, px, py, _ = post_circles(cfg)[0]
    lo = {'name': 'x', 'seed': None, 'craters': [],
          'boulders': [{'position': [px, py], 'diameter': 0.3, 'height': 0.2}],
          'start_pose': {'x': 1.0, 'y': 1.0, 'yaw_deg': 0.0}}
    assert any('post of marker' in v for v in L.check_layout(cfg, lo, strict=False, paths=False))


def test_margins_come_from_config_and_are_enforced(cfg):
    import copy
    rc = cfg['randomiser']
    assert rc['start_zone_margin'] == 0.10 and rc['construction_zone_margin'] == 0.05
    sx = cfg['zones']['start'][2]
    lo = {'name': 'x', 'seed': None, 'craters': [],
          'start_pose': {'x': 1.0, 'y': 1.0, 'yaw_deg': 0},
          'boulders': [{'position': [sx + 0.15 + 0.04, 1.0], 'diameter': 0.3, 'height': 0.2}]}
    msgs = L.check_layout(cfg, lo, strict=True, paths=False)
    assert any('closer than 0.1 m to the start zone' in m for m in msgs)
    lo['boulders'][0]['position'][0] += 0.07            # 0.11 m clear: passes the start margin
    assert not any('start zone' in m for m in L.check_layout(cfg, lo, strict=True, paths=False))
    c2 = copy.deepcopy(cfg)
    c2['randomiser']['start_zone_margin'] = 0.3         # the knob is honoured by the checker
    assert any('start zone' in m for m in L.check_layout(c2, lo, strict=True, paths=False))
    # and by the randomiser: every seed keeps its extent + margin from both zones
    z = c2['zones']
    c2['randomiser']['construction_zone_margin'] = 0.2
    for s in range(15):
        lay = L.random_layout(c2, s)
        for (_, _, x, y, _r), e in zip(L._items(lay), L._extents(c2, lay)):
            assert L._circle_rect_dist(x, y, z['start']) >= e + 0.3 - 1e-6
            assert L._circle_rect_dist(x, y, z['construction']) >= e + 0.2 - 1e-6


def test_cm_rounds_half_up_consistently():
    assert L._cm(0.825) == 83
    assert L._cm(0.625) == 63
    assert L._cm(0.615) == 62
    assert L._cm(0.0) == 0


def test_strict_applies_min_gap_to_marker_posts(cfg):
    pid, px, py, pr = post_circles(cfg)[0]
    lo = {'name': 't', 'seed': None, 'craters': [], 'start_pose': {'x': 1, 'y': 1, 'yaw_deg': 0},
          'boulders': [{'position': [px + 0.05 + pr + 0.2 + 0.0, py + 0.9], 'diameter': 0.4,
                        'height': 0.2}]}
    near = dict(lo, boulders=[{'position': [px + 0.3, py], 'diameter': 0.4, 'height': 0.2}])
    assert any('post of marker' in m for m in L.check_layout(cfg, near, paths=False))
    assert not any('post of marker' in m for m in L.check_layout(cfg, lo, paths=False))
