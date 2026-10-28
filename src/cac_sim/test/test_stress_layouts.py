"""Hand-made stress layouts (layouts/*.yaml, except the organiser's schematic): no ROS."""
from collections import deque
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cac_sim_gen import layout as L  # noqa: E402, I100
from cac_sim_gen.config import load_config, load_layout  # noqa: E402
import pytest  # noqa: E402, I201

LAYOUT_DIR = Path(__file__).resolve().parent.parent / 'layouts'
STRESS = ['gate', 'column_pinch', 'crater_field', 'busy_dig', 'start_trap']


@pytest.fixture(scope='module')
def cfg():
    return load_config()


def _reach(cfg, lo, src, goal, inflate=None, res=0.05):
    """Can a round robot (default: half width + clearance) drive from src to goal(x, y)."""
    blocked, xs, ys = L._blocked_grid(cfg, lo, inflate or L._default_inflate(cfg), res)
    nx, ny = blocked.shape
    s = (int(src[0] / res), int(src[1] / res))
    seen, todo = {s}, deque([s])
    while todo:
        i, j = todo.popleft()
        if goal(xs[i], ys[j]):
            return True
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < nx and 0 <= b < ny and not blocked[a, b] and (a, b) not in seen:
                seen.add((a, b))
                todo.append((a, b))
    return False


def test_all_layout_files_present():
    names = {p.stem for p in LAYOUT_DIR.glob('*.yaml')}
    assert set(STRESS) | {'schematic'} <= names


@pytest.mark.parametrize('name', STRESS)
def test_stress_layout_is_legal(cfg, name):
    lo = load_layout(LAYOUT_DIR / f'{name}.yaml')
    assert lo['name'] == name and lo['seed'] is None
    assert L.check_layout(cfg, lo, strict=False) == []
    assert L.se2_path_exists(cfg, lo)[0]
    text = (LAYOUT_DIR / f'{name}.yaml').read_text()
    assert text.startswith('# STRESS LAYOUT')
    # strict: everything except the gate's intended straight corridor
    issues = L.check_layout(cfg, lo, strict=True)
    if name == 'gate':
        assert len(issues) == 1 and 'straight eastward corridor' in issues[0]
    else:
        assert issues == []


def _gate_with_gap(lo, gap):
    """Copy of the gate with the upper half of the barrier shifted so the gap is `gap` m."""
    out = {k: (list(v) if isinstance(v, list) else v) for k, v in lo.items()}
    bs = [dict(b, position=list(b['position'])) for b in lo['boulders']]
    cur, lower, upper = _gate_gap(lo)
    for b in bs:
        if b['position'][0] == 3.0 and b['position'][1] > lower + 0.5 * cur:
            b['position'][1] += gap - cur
    out['boulders'] = bs
    return out


def _gate_gap(lo):
    bar = [b for b in lo['boulders'] if b['position'][0] == 3.0]
    ys = sorted((b['position'][1], b['diameter'] / 2) for b in bar)
    # the widest edge-to-edge spacing in the barrier is the gap
    gaps = [(ys[i + 1][0] - ys[i + 1][1] - (ys[i][0] + ys[i][1]), ys[i][0] + ys[i][1],
             ys[i + 1][0] - ys[i + 1][1]) for i in range(len(ys) - 1)]
    return max(gaps)


def test_gate_gap_is_near_the_se2_minimum(cfg):
    lo = load_layout(LAYOUT_DIR / 'gate.yaml')
    gap, _, _ = _gate_gap(lo)
    assert not any(b['position'][0] == 3.0 and abs(b['position'][1] - 1.6) < 0.2
                   for b in lo['boulders'])
    lo_, hi_ = 0.5, gap + 0.5
    assert L.se2_path_exists(cfg, _gate_with_gap(lo, hi_))[0]
    assert not L.se2_path_exists(cfg, _gate_with_gap(lo, lo_))[0]
    while hi_ - lo_ > 0.01:
        mid = (lo_ + hi_) / 2
        if L.se2_path_exists(cfg, _gate_with_gap(lo, mid))[0]:
            hi_ = mid
        else:
            lo_ = mid
    assert hi_ <= gap <= hi_ + 0.10, f'gate gap {gap:.3f} m, SE(2) minimum {hi_:.3f} m'


def test_crater_field_is_shallow_and_on_the_travel_route(cfg):
    lo = load_layout(LAYOUT_DIR / 'crater_field.yaml')
    route = cfg['randomiser']['routes']['travel']
    near = []
    for c in lo['craters']:
        assert 0.10 <= c.get('depth_ratio', 0) <= 0.12
        x, y = c['position']
        if x < cfg['zones']['excavation'][0]:
            near.append(min(_seg_dist(x, y, route[i], route[i + 1])
                            for i in range(len(route) - 1)))
    assert len(near) >= 6 and all(d <= 1.0 for d in near)
    assert sum(1 for b in lo['boulders'] if b['position'][0] < 5) <= 4


def _seg_dist(x, y, p, q):
    dx, dy = q[0] - p[0], q[1] - p[1]
    t = max(0.0, min(1.0, ((x - p[0]) * dx + (y - p[1]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(x - (p[0] + t * dx), y - (p[1] + t * dy))


def test_column_pinch_has_one_lane(cfg):
    lo = load_layout(LAYOUT_DIR / 'column_pinch.yaml')
    x_exc = cfg['zones']['excavation'][0] + 0.3
    src = (1.0, 1.4)
    assert _reach(cfg, lo, src, lambda x, y: x > x_exc)
    # a boulder anywhere in the lane between the column and the crater closes the way
    for y in (3.0, 3.4, 3.8):
        shut = dict(lo, boulders=lo['boulders'] + [
            {'position': [4.4, y], 'diameter': 0.35, 'height': 0.25}])
        assert not _reach(cfg, shut, src, lambda x, y: x > x_exc)


def test_busy_dig_is_full_and_haul_is_drivable(cfg):
    lo = load_layout(LAYOUT_DIR / 'busy_dig.yaml')
    ex = cfg['zones']['excavation']
    items = [it for it in L._items(lo) if it[2] >= ex[0]]
    nb = sum(1 for it in items if it[0] == 'boulder')
    nc = sum(1 for it in items if it[0] == 'crater')
    assert nb == cfg['boulders']['in_excavation_zone'][1]
    assert nc == cfg['craters']['in_excavation_zone'][1]
    haul = cfg['randomiser']['routes']['haul']
    dig = haul[0]
    for kind, _, x, y, r in items:
        assert math.hypot(x - dig[0], y - dig[1]) - r >= 0.5, (kind, x, y)
    target = cfg['berm']['target_centre']
    assert _reach(cfg, lo, dig, lambda x, y: math.hypot(x - target[0], y - target[1]) < 0.3)


def _turn_before_leaving(path, zone):
    """Largest heading change (deg) before the path leaves the start zone."""
    start = path[0][2]
    best = 0.0
    for x, y, yaw in path:
        best = max(best, abs((yaw - start + 180.0) % 360.0 - 180.0))
        if x > zone[2] or y > zone[3]:
            break
    return best


def test_start_trap_forces_a_turn(cfg):
    lo = load_layout(LAYOUT_DIR / 'start_trap.yaml')
    sp = lo['start_pose']
    assert sp['yaw_deg'] == 180
    sm = cfg['randomiser']['start_zone_margin']
    z = cfg['zones']['start']
    near = [it for it in L._items(lo) if it[2] < z[2] + 1.0 and it[3] < z[3] + 1.0]
    assert len(near) >= 5
    for kind, _, x, y, r in near:      # as close as the rules allow, never closer
        d = L._circle_rect_dist(x, y, z) - r
        assert sm - 1e-6 <= d <= sm + 0.16
    ok, path = L.se2_path_exists(cfg, lo)
    assert ok
    assert _turn_before_leaving(path, z) >= 67.5   # the route must turn inside the start zone
    # negative control: without the east column and the south boulders the robot can reverse out
    # eastwards, and the same check must then fail
    open_east = dict(lo, boulders=[b for b in lo['boulders']
                                   if b['position'][1] > 2.0 or b['position'][0] > 5])
    ok2, path2 = L.se2_path_exists(cfg, open_east)
    assert ok2 and _turn_before_leaving(path2, z) < 67.5
    # the north gap is the only way out: one more boulder in it shuts the zone
    shut = dict(lo, boulders=lo['boulders'] + [
        {'position': [1.3, 2.3], 'diameter': 0.4, 'height': 0.25}])
    assert not L.se2_path_exists(cfg, shut)[0]
    assert not _reach(cfg, shut, (1.0, 1.0), lambda x, y: x > 2.6, inflate=0.2)
