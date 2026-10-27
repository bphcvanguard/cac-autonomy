"""Layouts: seeded randomiser, rule checker, path-exists check, ground truth and top-down map.

A layout is a plain dict (see DESIGN.md). Everything here is deterministic for a given seed.
RNG streams: obstacles = default_rng([seed, 2]), start pose = default_rng([seed, 3]).
Marker posts (markers.post_circles) are obstacles in every path check, with or without --markers.
"""
import heapq
import math

import numpy as np

from .geometry import crater_params, write_png
from .markers import (marker_list, marker_placement, post_circles,
                      wall_mounted)

OBSTACLE_STREAM = 2
START_STREAM = 3
EPS = 1e-6
MAX_ATTEMPTS = 600
POSE_REDRAW_EVERY = 100       # hard start poses (facing away, low y) get a fresh pose
SLALOM_BIAS = 0.7             # probability an obstacle-zone obstacle is drawn from the band
YAW_BINS = 16
PLACE_TRIES = 200


# ----------------------------------------------------------------------------- helpers

def _footprint_corners(cfg, pose):
    length, width = cfg['randomiser']['robot_footprint']
    yaw = math.radians(pose['yaw_deg'])
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        lx, ly = sx * length / 2, sy * width / 2
        pts.append((pose['x'] + c * lx - s * ly, pose['y'] + s * lx + c * ly))
    return pts


def _in_rect(x, y, rect):
    return rect[0] <= x < rect[2] and rect[1] <= y < rect[3]


def _circle_rect_dist(cx, cy, rect):
    """Distance from a point to a rectangle (0 inside)."""
    dx = max(rect[0] - cx, 0.0, cx - rect[2])
    dy = max(rect[1] - cy, 0.0, cy - rect[3])
    return math.hypot(dx, dy)


def _extents(cfg, layout):
    """Full footprint radius per item (boulder r, crater geometry.outer_radius), _items order."""
    seed = layout.get('seed')
    out = [b['diameter'] / 2 for b in layout.get('boulders', [])]
    out += [crater_params(cfg, layout, i, seed)['outer_radius']
            for i in range(len(layout.get('craters', [])))]
    return out


def _items(layout):
    """All obstacles as (kind, index, x, y, radius)."""
    out = []
    for i, b in enumerate(layout.get('boulders', [])):
        out.append(('boulder', i, b['position'][0], b['position'][1], b['diameter'] / 2))
    for i, c in enumerate(layout.get('craters', [])):
        out.append(('crater', i, c['position'][0], c['position'][1], c['diameter'] / 2))
    return out


def _column(cfg):
    """
    Return the column as a circle (x, y, r).

    A box column uses its circumscribed circle: conservative, so every obstacle gap and path
    check stays valid for the real square.
    """
    col = cfg['column']
    return col['centre'][0], col['centre'][1], col['radius']


def _posts(cfg):
    """Marker collision posts of the configured preset as circles (x, y, r)."""
    return [(x, y, r) for _, x, y, r in post_circles(cfg)]


def _circles(cfg, layout):
    """Every fixed obstacle for path checks: items, column, marker posts, as (x, y, r)."""
    return [(x, y, r) for _, _, x, y, r in _items(layout)] + [_column(cfg)] + _posts(cfg)


def _zone_of(cfg, x, y):
    z = cfg['zones']
    if _in_rect(x, y, z['construction']):
        return 'construction'
    if _in_rect(x, y, z['start']):
        return 'start'
    if _in_rect(x, y, z['obstacle']):
        return 'obstacle'
    if _in_rect(x, y, z['excavation']):
        return 'excavation'
    return 'outside'


def _name(kind, i):
    return f'{kind} {i}'


# ----------------------------------------------------------------------------- start pose

def _pose_interval(cfg, yaw_deg):
    """Return the allowed (x_lo, x_hi, y_lo, y_hi) robot-centre box for a rotated footprint."""
    length, width = cfg['randomiser']['robot_footprint']
    m = cfg['randomiser']['start_pose_margin']
    x0, y0, x1, y1 = cfg['zones']['start']
    yaw = math.radians(yaw_deg)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    hx = c * length / 2 + s * width / 2
    hy = s * length / 2 + c * width / 2
    return x0 + m + hx, x1 - m - hx, y0 + m + hy, y1 - m - hy


def random_start_pose(cfg, rng):
    """Yaw uniform in [-180, 180); position uniform over where the rotated footprint fits.

    The footprint (L along heading) must lie inside the start zone shrunk by start_pose_margin.
    Position is sampled given the yaw; a yaw with no feasible position is resampled. Poses from
    which the footprint (+ clearance) cannot even turn out of the start zone in an empty arena
    are rejected too.
    """
    for _ in range(1000):
        yaw = float(rng.uniform(-180.0, 180.0))
        xl, xh, yl, yh = _pose_interval(cfg, yaw)
        if xh - xl < 0.004 or yh - yl < 0.004:
            continue
        # keep 1 mm inside so rounding to mm cannot push the footprint out
        x = float(rng.uniform(xl + 0.001, xh - 0.001))
        y = float(rng.uniform(yl + 0.001, yh - 0.001))
        pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(yaw, 2)}
        # the rover must be able to leave the start zone even with nothing in the arena: a pose
        # jammed against a wall with the clearance margin cannot turn (needs a 3-point turn)
        empty = {'boulders': [], 'craters': [], 'start_pose': pose}
        if se2_path_exists(cfg, empty)[0]:
            return pose
    raise RuntimeError('robot footprint does not fit in the start zone at any yaw')


# ----------------------------------------------------------------------------- randomiser

def _sample_obstacle(cfg, rng, kind, zone, placed, fixed):
    """Draw one obstacle in `zone` ('obstacle'|'excavation'|'construction') that clears all.

    Items lie fully inside their zone (crater rim included), clear the start zone by
    start_zone_margin, the construction zone by construction_zone_margin, and the other items,
    the column and the marker posts (`fixed`) by min_gap_between_obstacles.
    """
    rc = cfg['randomiser']
    gap, off = rc['min_gap_between_obstacles'], rc['keep_off_walls']
    w, h = cfg['arena']['size']
    spec = cfg['boulders'] if kind == 'boulder' else cfg['craters']
    z = cfg['zones']
    slack = 0.002
    for _ in range(PLACE_TRIES):
        d = float(rng.uniform(*spec['diameter']))
        r = d / 2
        item = {'diameter': round(d, 3)}
        if kind == 'boulder':
            lo, hi = spec['height']
            hi = min(hi, spec.get('max_height_ratio', 10.0) * d)   # no pillar-shaped rocks
            item['height'] = round(float(rng.uniform(lo, max(lo, hi))), 3)
        # crater depth and rim are drawn by geometry.crater_params from the crater's own stream
        d = item['diameter']
        r = d / 2
        if kind == 'crater':
            tmp = {'craters': [dict(item, position=[0, 0])]}
            ext = crater_params(cfg, tmp, 0, 0)['outer_radius'] + 0.002
        else:
            ext = r + 0.002
        ylo, yhi = off + r, h - off - r
        # Always draw the same numbers in the same order (one stream): which branch is used must
        # not change how many draws follow.
        u_route, u_slalom = float(rng.random()), float(rng.random())
        t_route, n_route = float(rng.random()), float(rng.uniform(-1, 1))
        u_seg = float(rng.random())
        if zone == 'obstacle':
            xlo, xhi = off + r, z['excavation'][0] - ext
            if u_slalom < SLALOM_BIAS:
                band = rc.get('slalom_band', [2.0, 5.0])
                xlo, xhi = max(xlo, band[0] + ext), min(xhi, band[1] - ext)
        elif zone == 'excavation':
            xlo, xhi = z['excavation'][0] + ext, w - off - r
        else:
            xlo, xhi = z['construction'][0] + ext, w - off - r
            ylo = max(ylo, z['construction'][1] + ext)
            yhi = min(yhi, z['construction'][3] - ext)
        if xhi <= xlo or yhi <= ylo:
            continue
        x, y = float(rng.uniform(xlo, xhi)), float(rng.uniform(ylo, yhi))
        segs = _route_segments(cfg, zone)
        if segs and u_route < rc.get('route_bias', 0.0):
            (ax, ay), (bx, by) = segs[min(int(u_seg * len(segs)), len(segs) - 1)]
            ln = math.hypot(bx - ax, by - ay)
            off_n = n_route * rc.get('route_band', 0.5)
            x = ax + t_route * (bx - ax) - off_n * (by - ay) / ln
            y = ay + t_route * (by - ay) + off_n * (bx - ax) / ln
            if not (xlo <= x <= xhi and ylo <= y <= yhi):
                continue
        if _circle_rect_dist(x, y, z['start']) < ext + rc['start_zone_margin']:
            continue
        cm = rc['construction_zone_margin']
        if zone != 'construction' and _circle_rect_dist(x, y, z['construction']) < ext + cm:
            continue
        ok = True
        for (px, py, pr) in placed + fixed:
            if math.hypot(x - px, y - py) - r - pr < gap + slack:
                ok = False
                break
        if not ok:
            continue
        item['position'] = [round(x, 3), round(y, 3)]
        return item
    return None


def _route_segments(cfg, zone):
    """Route segments (from randomiser.routes) whose midpoint lies in `zone`."""
    out = []
    for pts in cfg['randomiser'].get('routes', {}).values():
        for a, b in zip(pts[:-1], pts[1:]):
            if _zone_of(cfg, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2) == zone:
                out.append((tuple(a), tuple(b)))
    return out


def _one_attempt(cfg, rng):
    b, c = cfg['boulders'], cfg['craters']
    n = {
        ('boulder', 'obstacle'): int(rng.integers(b['min_in_obstacle_zone'],
                                                  b['max_in_obstacle_zone'] + 1)),
        ('crater', 'obstacle'): int(rng.integers(c['min_in_obstacle_zone'],
                                                 c['max_in_obstacle_zone'] + 1)),
        ('boulder', 'excavation'): int(rng.integers(b['in_excavation_zone'][0],
                                                    b['in_excavation_zone'][1] + 1)),
        ('crater', 'excavation'): int(rng.integers(c['in_excavation_zone'][0],
                                                   c['in_excavation_zone'][1] + 1)),
    }
    lo, hi = b.get('in_construction_zone', [0, 0])
    n[('boulder', 'construction')] = int(rng.integers(lo, hi + 1))
    n[('crater', 'construction')] = 0
    jobs = []
    for zone in ('obstacle', 'excavation', 'construction'):
        seq = ['boulder'] * n[('boulder', zone)] + ['crater'] * n[('crater', zone)]
        order = rng.permutation(len(seq))
        jobs += [(seq[i], zone) for i in order]
    fixed = [_column(cfg)] + _posts(cfg)
    placed, boulders, craters = [], [], []
    for kind, zone in jobs:
        item = _sample_obstacle(cfg, rng, kind, zone, placed, fixed)
        if item is None:
            return None
        placed.append((item['position'][0], item['position'][1], item['diameter'] / 2))
        (boulders if kind == 'boulder' else craters).append(item)
    return boulders, craters


def _reason(issue):
    if 'straight' in issue:
        return 'no_slalom'
    if 'SE(2)' in issue:
        return 'se2_path'
    if 'no path' in issue:
        return 'disc_path'
    return 'rules'


def random_layout(cfg, seed, stats=None):
    """Seeded random layout that passes check_layout(strict=True), paths included.

    `stats` (optional dict) receives {'attempts': n, 'reasons': {reason: rejects}}; reasons are
    placement / rules / no_slalom / disc_path / se2_path.
    """
    rng = np.random.default_rng([seed, OBSTACLE_STREAM])
    pose_rng = np.random.default_rng([seed, START_STREAM])
    pose = random_start_pose(cfg, pose_rng)
    reasons = {}

    def reject(why):
        reasons[why] = reasons.get(why, 0) + 1

    for attempt in range(1, MAX_ATTEMPTS + 1):
        if attempt % POSE_REDRAW_EVERY == 0:
            pose = random_start_pose(cfg, pose_rng)
            reject('pose_redraw')
        res = _one_attempt(cfg, rng)
        if res is None:
            reject('placement')
            continue
        layout = {'name': f'seed_{seed:04d}', 'seed': int(seed), 'boulders': res[0],
                  'craters': res[1], 'start_pose': dict(pose)}
        issues = check_layout(cfg, layout, strict=True, paths=False)
        if issues:
            reject(_reason(issues[0]))
            continue
        issues = _path_issues(cfg, layout)
        if issues:
            reject(_reason(issues[0]))
            continue
        if stats is not None:
            stats.update(attempts=attempt, reasons=reasons)
        return layout
    raise RuntimeError(f'no valid layout for seed {seed} in {MAX_ATTEMPTS} attempts')


# ----------------------------------------------------------------------------- disc path check

def _blocked_grid(cfg, layout, inflate, res):
    w, h = cfg['arena']['size']
    nx, ny = int(round(w / res)), int(round(h / res))
    xs = (np.arange(nx) + 0.5) * res
    ys = (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys, indexing='ij')           # (nx, ny)
    blocked = (gx < inflate) | (gx > w - inflate) | (gy < inflate) | (gy > h - inflate)
    circles = _circles(cfg, layout)
    for x, y, r in circles:
        blocked |= (gx - x) ** 2 + (gy - y) ** 2 < (r + inflate) ** 2
    return blocked, xs, ys


def _default_inflate(cfg):
    rc = cfg['randomiser']
    return rc['robot_footprint'][1] / 2 + rc['clearance']


def _plan(cfg, layout, inflate, res):
    blocked, xs, ys = _blocked_grid(cfg, layout, inflate, res)
    nx, ny = blocked.shape
    sx0, sy0, sx1, sy1 = cfg['zones']['start']
    gx_min = cfg['zones']['excavation'][0]
    free = ~blocked
    starts = [(i, j) for i in range(nx) for j in range(ny)
              if free[i, j] and sx0 <= xs[i] < sx1 and sy0 <= ys[j] < sy1]
    if not starts:
        return False, [], blocked, xs, ys
    free_l = free.tolist()
    inf = float('inf')
    dist = [[inf] * ny for _ in range(nx)]
    prev = {}
    heap = []
    for (i, j) in starts:
        dist[i][j] = 0.0
        heap.append((max(0.0, gx_min - xs[i]), 0.0, i, j))
    heapq.heapify(heap)
    nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    goal = None
    while heap:
        _, d, i, j = heapq.heappop(heap)
        if d > dist[i][j]:
            continue
        if xs[i] >= gx_min:
            goal = (i, j)
            break
        for di, dj, cst in nbrs:
            a, b = i + di, j + dj
            if not (0 <= a < nx and 0 <= b < ny) or not free_l[a][b]:
                continue
            if di and dj and not (free_l[i + di][j] and free_l[i][j + dj]):
                continue
            nd = d + cst * res
            if nd < dist[a][b]:
                dist[a][b] = nd
                prev[(a, b)] = (i, j)
                heapq.heappush(heap, (nd + max(0.0, gx_min - xs[a]), nd, a, b))
    if goal is None:
        return False, [], blocked, xs, ys
    cells = [goal]
    while cells[-1] in prev:
        cells.append(prev[cells[-1]])
    cells.reverse()
    pts = [(float(xs[i]), float(ys[j])) for i, j in cells]
    return True, _shortcut(pts, blocked, res), blocked, xs, ys


def _seg_free(p, q, blocked, res):
    n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / (res * 0.5)) + 1)
    nx, ny = blocked.shape
    for k in range(n + 1):
        t = k / n
        i = int((p[0] + (q[0] - p[0]) * t) / res)
        j = int((p[1] + (q[1] - p[1]) * t) / res)
        if not (0 <= i < nx and 0 <= j < ny) or blocked[i, j]:
            return False
    return True


def _shortcut(pts, blocked, res):
    """Greedy line-of-sight shortcutting of the grid path into a short polyline."""
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not _seg_free(pts[i], pts[j], blocked, res):
            j -= 1
        out.append(pts[j])
        i = j
    return [(round(x, 3), round(y, 3)) for x, y in out]


def path_exists(cfg, layout, inflate=None, resolution=0.05):
    """Check disc reachability: can a round robot go from the start to the excavation zone.

    Occupancy grid at `resolution` (5 cm). Boulders, craters, the column and the walls are
    inflated by `inflate` metres, default = robot half-width + clearance (inscribed-radius
    check, a fast prefilter). It ignores the 1.5 m length: use se2_path_exists for the real
    footprint that has to turn. Pass inflate=0.95 for a conservative turning-disc variant.
    Returns (ok, polyline of (x, y)).
    """
    if inflate is None:
        inflate = _default_inflate(cfg)
    ok, path, _, _, _ = _plan(cfg, layout, inflate, resolution)
    return ok, path


def _bottleneck_width(cfg, layout, res=0.05):
    """Widest disc diameter that still passes from start to excavation (bisection)."""
    lo, hi = 0.0, 1.5
    for _ in range(6):
        mid = (lo + hi) / 2
        if _plan(cfg, layout, mid, res)[0]:
            lo = mid
        else:
            hi = mid
    return 2 * lo


# ----------------------------------------------------------------------------- SE(2) check

def _box_circle_blocked(gx, gy, cx, cy, r, c, s, a, b):
    """Return True where a robot box (half extents a, b, yaw cos/sin c, s) hits the circle."""
    dx, dy = cx - gx, cy - gy
    u = c * dx + s * dy
    v = -s * dx + c * dy
    return np.hypot(np.maximum(np.abs(u) - a, 0.0), np.maximum(np.abs(v) - b, 0.0)) < r


def _se2_blocked(cfg, layout, yaw_bins, res):
    """Configuration-space obstacle (yaw_bins, nx, ny) at `res`, conservative (any-blocked)."""
    length, width = cfg['randomiser']['robot_footprint']
    clr = cfg['randomiser']['clearance']
    a, b = length / 2 + clr, width / 2 + clr
    w, h = cfg['arena']['size']
    fs = 0.05
    fnx, fny = int(round(w / fs)), int(round(h / fs))
    fx = (np.arange(fnx) + 0.5) * fs
    fy = (np.arange(fny) + 0.5) * fs
    reach = math.hypot(a, b)
    circles = _circles(cfg, layout)
    half = yaw_bins // 2
    out = np.zeros((yaw_bins, fnx, fny), bool)
    for k in range(half):                       # a rectangle is symmetric under 180 deg
        th = 2 * math.pi * k / yaw_bins
        c, s = math.cos(th), math.sin(th)
        hx, hy = abs(c) * a + abs(s) * b, abs(s) * a + abs(c) * b
        m = np.zeros((fnx, fny), bool)
        m |= (fx < hx)[:, None] | (fx > w - hx)[:, None]
        m |= (fy < hy)[None, :] | (fy > h - hy)[None, :]
        for cx, cy, r in circles:
            R = r + reach
            i0, i1 = max(int((cx - R) / fs), 0), min(int((cx + R) / fs) + 2, fnx)
            j0, j1 = max(int((cy - R) / fs), 0), min(int((cy + R) / fs) + 2, fny)
            if i0 >= i1 or j0 >= j1:
                continue
            gx, gy = np.meshgrid(fx[i0:i1], fy[j0:j1], indexing='ij')
            m[i0:i1, j0:j1] |= _box_circle_blocked(gx, gy, cx, cy, r, c, s, a, b)
        out[k] = m
        out[k + half] = m
    f = int(round(res / fs))
    if f > 1:
        out = out.reshape(yaw_bins, fnx // f, f, fny // f, f).any(axis=(2, 4))
    return out


def _shift_or(dst, src, di, dj, bins):
    """dst[k, a, b] |= src[k, a - di, b - dj] for k in bins (in place, clipped at borders)."""
    nx, ny = src.shape[1], src.shape[2]
    a0, a1 = max(di, 0), nx + min(di, 0)
    b0, b1 = max(dj, 0), ny + min(dj, 0)
    dst[bins, a0:a1, b0:b1] |= src[bins, a0 - di:a1 - di, b0 - dj:b1 - dj]


def _allowed_bins(di, dj, yaw_bins, holonomic):
    """Yaw bins in which a move (di, dj) is allowed: along the heading, forwards or backwards."""
    if holonomic:
        return np.arange(yaw_bins)
    ang = math.degrees(math.atan2(dj, di))
    step = 360.0 / yaw_bins
    ok = []
    for k in range(yaw_bins):
        d = (ang - k * step) % 180.0
        if min(d, 180.0 - d) <= step + 1e-6:
            ok.append(k)
    return np.array(ok)


def se2_path_exists(cfg, layout, yaw_bins=YAW_BINS, resolution=0.1, holonomic=False):
    """Check SE(2) reachability of the real footprint (L x W + clearance) that must turn.

    State = (x, y, yaw bin). Collision is a rotated rectangle (half extents L/2 + c, W/2 + c)
    vs every circle and the walls, computed on a 5 cm grid and reduced conservatively to
    `resolution` (10 cm). Moves: 8-neighbour in the same yaw bin; in-place rotation to the
    adjacent bin if the cell is free in both bins (standard approximation). Unless
    holonomic=True a translation must lie within one bin step (22.5 deg) of the heading or its
    reverse, so the rover cannot crab sideways. Start = the actual
    start pose (forced free: the robot is physically there). Goal = robot centre
    x >= excavation x_min - L/2 (any part of the robot enters the excavation zone).
    Returns (ok, [(x, y, yaw_deg)] waypoints, ~0.5 m apart and at every yaw change).
    """
    length = cfg['randomiser']['robot_footprint'][0]
    blocked = _se2_blocked(cfg, layout, yaw_bins, resolution)
    free = ~blocked
    nb, nx, ny = free.shape
    sp = layout['start_pose']
    si = min(max(int(sp['x'] / resolution), 0), nx - 1)
    sj = min(max(int(sp['y'] / resolution), 0), ny - 1)
    step_deg = 360.0 / yaw_bins
    sk = int(round((sp['yaw_deg'] % 360.0) / step_deg)) % yaw_bins
    free[sk, si, sj] = True
    gi = int(math.ceil((cfg['zones']['excavation'][0] - length / 2) / resolution - 0.5))
    goal = np.zeros_like(free)
    goal[:, gi:, :] = True
    dist = np.full(free.shape, -1, np.int32)
    dist[sk, si, sj] = 0
    frontier = np.zeros_like(free)
    frontier[sk, si, sj] = True
    reached = frontier.copy()
    steps = 0
    hit = None
    moves = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    allowed = {m: _allowed_bins(m[0], m[1], nb, holonomic) for m in moves}
    while frontier.any():
        steps += 1
        new = np.zeros_like(free)
        for di, dj in moves:
            _shift_or(new, frontier, di, dj, allowed[(di, dj)])
        new |= np.roll(frontier, 1, axis=0)
        new |= np.roll(frontier, -1, axis=0)
        new &= free
        new &= ~reached
        dist[new] = steps
        reached |= new
        frontier = new
        g = new & goal
        if g.any():
            hit = tuple(int(v) for v in np.argwhere(g)[0])
            break
    if hit is None:
        return False, []
    chain = [hit]
    while dist[chain[-1]] > 0:
        k, i, j = chain[-1]
        d = dist[k, i, j] - 1
        nxt = None
        for dk, di, dj in [(0, a, b) for a, b in moves] + [(1, 0, 0), (-1, 0, 0)]:
            kk, ii, jj = (k + dk) % nb, i + di, j + dj
            if dk == 0 and k not in allowed[(-di, -dj)]:
                continue
            if 0 <= ii < nx and 0 <= jj < ny and dist[kk, ii, jj] == d:
                nxt = (kk, ii, jj)
                break
        chain.append(nxt)
    chain.reverse()
    pts = []
    last_k, last_n = None, 0
    for n, (k, i, j) in enumerate(chain):
        if last_k is None or k != last_k or n - last_n >= 5 or n == len(chain) - 1:
            yaw = k * step_deg
            pts.append((round((i + 0.5) * resolution, 3), round((j + 0.5) * resolution, 3),
                        round(yaw - 360.0 if yaw >= 180.0 else yaw, 1)))
            last_k, last_n = k, n
    return True, pts


def _straight_corridor(cfg, layout):
    """Return y of a clear straight eastward corridor (W + 2 clearance wide), else None.

    Tries start y in [W/2, start_h - W/2]. Returns None when every straight corridor is blocked.
    """
    rc = cfg['randomiser']
    width = rc['robot_footprint'][1]
    half = width / 2 + rc['clearance']
    z = cfg['zones']
    circles = _circles(cfg, layout)
    x_to = z['excavation'][0]
    for y0 in np.arange(width / 2, z['start'][3] - width / 2 + 1e-9, 0.025):
        if not any(cx - r < x_to and abs(cy - y0) < half + r for cx, cy, r in circles):
            return float(y0)
    return None


def _path_issues(cfg, layout):
    ok, _ = path_exists(cfg, layout)
    if not ok:
        return ['no path from the start zone to the excavation zone for a round robot (disc)']
    ok, _ = se2_path_exists(cfg, layout)
    if not ok:
        return ['no path for the 1.5 x 0.75 m footprint turning (SE(2)) to the excavation zone']
    return []


# ----------------------------------------------------------------------------- checker

def _box_hits_circle(cfg, pose, cx, cy, r):
    length, width = cfg['randomiser']['robot_footprint']
    th = math.radians(pose['yaw_deg'])
    return bool(_box_circle_blocked(np.array(pose['x']), np.array(pose['y']), cx, cy, r,
                                    math.cos(th), math.sin(th), length / 2, width / 2))


def check_layout(cfg, layout, strict=True, paths=True):
    """Return a list of human-readable violations (empty = valid).

    strict=True checks every rule in arena.yaml. strict=False reports only hard physical
    problems: overlapping obstacles or column, out of the arena, start footprint not in the start
    zone or on an obstacle, no path. paths=False skips the (slower) path checks.
    """
    v = []
    z = cfg['zones']
    rc = cfg['randomiser']
    w, h = cfg['arena']['size']
    items = _items(layout)
    exts = _extents(cfg, layout)
    col = _column(cfg)

    for (kind, i, x, y, r), e in zip(items, exts):
        ee = e if strict else r          # the soft rim may poke over the wall; the core may not
        if x - ee < -EPS or y - ee < -EPS or x + ee > w + EPS or y + ee > h + EPS:
            v.append(f'{_name(kind, i)} extends outside the arena')
        g = math.hypot(x - col[0], y - col[1]) - r - col[2]
        if g < -EPS:
            v.append(f'{_name(kind, i)} overlaps the column by {-g:.2f} m')
        for pid, px, py, pr in post_circles(cfg):
            g = math.hypot(x - px, y - py) - r - pr
            if g < -EPS:
                v.append(f'{_name(kind, i)} overlaps the post of marker {pid} by {-g:.2f} m')

    for a in range(len(items)):
        for b in range(a + 1, len(items)):
            ka, ia, xa, ya, ra = items[a]
            kb, ib, xb, yb, rb = items[b]
            g = math.hypot(xa - xb, ya - yb) - ra - rb
            if g < -EPS:
                v.append(f'{_name(ka, ia)} overlaps {_name(kb, ib)} by {-g:.2f} m')
            elif strict and g < rc['min_gap_between_obstacles'] - EPS:
                v.append(f'gap {g:.2f} m between {_name(ka, ia)} and {_name(kb, ib)} '
                         f'< {rc["min_gap_between_obstacles"]} m')

    if strict:
        ex0 = z['excavation'][0]
        for (kind, i, x, y, r), e in zip(items, exts):
            g = math.hypot(x - col[0], y - col[1]) - r - col[2]
            if 0 <= g < rc['min_gap_between_obstacles'] - EPS:
                v.append(f'gap {g:.2f} m between {_name(kind, i)} and the column '
                         f'< {rc["min_gap_between_obstacles"]} m')
            dw = min(x - r, w - x - r, y - r, h - y - r)
            if dw < rc['keep_off_walls'] - EPS:
                v.append(f'{_name(kind, i)} is {dw:.2f} m from a wall '
                         f'(< {rc["keep_off_walls"]} m)')
            zone = _zone_of(cfg, x, y)
            sm, cm = rc['start_zone_margin'], rc['construction_zone_margin']
            if _circle_rect_dist(x, y, z['start']) < e + sm - EPS:
                v.append(f'{_name(kind, i)} is closer than {sm} m to the start zone '
                         f'(extent incl. rim {e:.2f} m)')
            if zone != 'construction' \
                    and _circle_rect_dist(x, y, z['construction']) < e + cm - EPS:
                v.append(f'{_name(kind, i)} is closer than {cm} m to the construction zone '
                         f'(extent incl. rim {e:.2f} m)')
            if zone == 'obstacle' and x + e > ex0 + EPS:
                v.append(f'{_name(kind, i)} straddles the obstacle/excavation boundary '
                         f'(extent incl. rim reaches x = {x + e:.2f})')
            if zone == 'excavation' and x - e < ex0 - EPS:
                v.append(f'{_name(kind, i)} straddles the obstacle/excavation boundary '
                         f'(extent incl. rim reaches x = {x - e:.2f})')
            if zone == 'construction':
                cz = z['construction']
                if x - e < cz[0] - EPS or y - e < cz[1] - EPS or x + e > cz[2] + EPS \
                        or y + e > cz[3] + EPS:
                    v.append(f'{_name(kind, i)} straddles the construction zone boundary')

        def count(kind_, zone):
            return sum(1 for k, _, x, y, _r in items
                       if k == kind_ and _zone_of(cfg, x, y) == zone)

        for kind, spec, plural in (('boulder', cfg['boulders'], 'boulders'),
                                   ('crater', cfg['craters'], 'craters')):
            n = count(kind, 'obstacle')
            lo, hi = spec['min_in_obstacle_zone'], spec['max_in_obstacle_zone']
            if not lo <= n <= hi:
                v.append(f'{n} {plural} in the obstacle zone (allowed {lo}..{hi})')
            n = count(kind, 'excavation')
            lo, hi = spec['in_excavation_zone']
            if not lo <= n <= hi:
                v.append(f'{n} {plural} in the excavation zone (allowed {lo}..{hi})')
            n = count(kind, 'construction')
            lo, hi = spec.get('in_construction_zone', [0, 0])
            if not lo <= n <= hi:
                v.append(f'{n} {plural} in the construction zone (allowed {lo}..{hi})')
            if count(kind, 'start'):
                v.append(f'{count(kind, "start")} {plural} centred in the start zone')

        for i, b in enumerate(layout.get('boulders', [])):
            _range(v, f'boulder {i} diameter', b['diameter'], cfg['boulders']['diameter'])
            _range(v, f'boulder {i} height', b['height'], cfg['boulders']['height'])
        for i, c in enumerate(layout.get('craters', [])):
            _range(v, f'crater {i} diameter', c['diameter'], cfg['craters']['diameter'])
            if 'depth_ratio' in c:
                _range(v, f'crater {i} depth_ratio', c['depth_ratio'],
                       cfg['craters']['depth_ratio'])
            if 'depth' in c:
                lo, hi = (r * c['diameter'] for r in cfg['craters']['depth_ratio'])
                _range(v, f'crater {i} depth', c['depth'], [lo, hi])

        if rc.get('require_slalom', False):
            y0 = _straight_corridor(cfg, layout)
            if y0 is not None:
                v.append(f'a straight eastward corridor is clear at y = {y0:.2f} m '
                         f'(no slalom needed)')

    sp = layout.get('start_pose')
    if not sp:
        v.append('no start pose')
    else:
        m = rc['start_pose_margin'] if strict else 0.0
        x0, y0, x1, y1 = z['start']
        for cx, cy in _footprint_corners(cfg, sp):
            if not (x0 + m - EPS <= cx <= x1 - m + EPS and y0 + m - EPS <= cy <= y1 - m + EPS):
                v.append('start footprint not inside the start zone' +
                         (f' shrunk by {m} m' if strict else ''))
                break
        fixed = [('column', 0, col[0], col[1], col[2])]
        fixed += [('marker post', pid, x, y, r) for pid, x, y, r in post_circles(cfg)]
        for kind, i, x, y, r in items + fixed:
            if _box_hits_circle(cfg, sp, x, y, r):
                v.append(f'start footprint overlaps {_name(kind, i)}')

    if paths:
        v += _path_issues(cfg, layout)
    return v


def _range(v, label, value, rng_):
    if not rng_[0] - EPS <= value <= rng_[1] + EPS:
        v.append(f'{label} {value} outside {rng_[0]}..{rng_[1]}')


# ----------------------------------------------------------------------------- truth

def truth(cfg, layout, markers=False):
    """Ground truth for the test harness and scoring. Plain python types only.

    Path fields always treat the configured marker posts as obstacles. `markers=True` (the world
    was generated with --markers) also lists them under `marker_posts`.
    """
    z = cfg['zones']
    obstacles = []
    for b in layout.get('boulders', []):
        pos = [float(b['position'][0]), float(b['position'][1])]
        obstacles.append({'type': 'boulder', 'position': pos,
                          'radius': float(b['diameter']) / 2, 'height': float(b['height'])})
    for c in layout.get('craters', []):
        pos = [float(c['position'][0]), float(c['position'][1])]
        cp = crater_params(cfg, layout, len(obstacles) - len(layout.get('boulders', [])),
                           layout.get('seed'))
        obstacles.append({'type': 'crater', 'position': pos,
                          'radius': float(c['diameter']) / 2, 'depth': float(cp['depth_eff']),
                          'max_slope_deg': float(cp['max_slope_deg_eff']),
                          'rim': float(cp['rim']), 'outer_radius': float(cp['outer_radius'])})
    ok, path = path_exists(cfg, layout)
    res = {'exists': bool(ok), 'path': [[float(x), float(y)] for x, y in path],
           'inflate': float(_default_inflate(cfg))}
    if ok:
        res['bottleneck_width'] = round(float(_bottleneck_width(cfg, layout)), 3)
    ok2, path2 = se2_path_exists(cfg, layout)
    col = cfg['column']
    sp = layout['start_pose']
    n_con = sum(1 for k, _, x, y, _r in _items(layout) if _zone_of(cfg, x, y) == 'construction')
    return {
        'name': layout.get('name'),
        'seed': layout.get('seed'),
        'zones': {k: [float(a) for a in z[k]] for k in z},
        'target_box': {'centre': [float(a) for a in cfg['berm']['target_centre']],
                       'size': [float(a) for a in cfg['berm']['target_size']],
                       'size_schematic': [float(a) for a in cfg['berm']['target_size_schematic']]},
        'column': {'position': [float(a) for a in col['centre']],
                   'shape': col.get('shape', 'cylinder'),
                   'size': [float(a) for a in col.get('size', [2 * col['radius']] * 2)],
                   'yaw_deg': float(col.get('yaw_deg', 0.0)),
                   'bounding_radius': float(col['radius']), 'height': float(col['height'])},
        'obstacles': obstacles,
        'marker_posts': [{'id': int(pid), 'position': [float(x), float(y)], 'radius': float(r)}
                         for pid, x, y, r in post_circles(cfg)] if markers else [],
        'paths_include_marker_posts': True,
        'obstacles_in_construction_zone': int(n_con),
        'start_pose': {'x': float(sp['x']), 'y': float(sp['y']), 'yaw_deg': float(sp['yaw_deg'])},
        'path_exists': res,
        'se2_path_exists': bool(ok2),
        'se2_path': [[float(a), float(b), float(c)] for a, b, c in path2],
    }


# ----------------------------------------------------------------------------- map

_FONT = {
    '0': '111101101101111', '1': '010110010010111', '2': '111001111100111',
    '3': '111001111001111', '4': '101101111001001', '5': '111100111001111',
    '6': '111100111101111', '7': '111001001001001', '8': '111101111101111',
    '9': '111101111001111',
    'B': '110101110101110', 'C': '111100100100111', 'X': '101101010101101',
    'Y': '101101010010010',
}


class _Canvas:
    def __init__(self, cfg, scale, margin):
        w, h = cfg['arena']['size']
        self.scale, self.margin = scale, margin
        self.W, self.H = int(round(w * scale)), int(round(h * scale))
        self.img = np.full((self.H + 2 * margin, self.W + 2 * margin, 3), 255, np.uint8)
        self.arena = self.img[margin:margin + self.H, margin:margin + self.W]
        px = (np.arange(self.W) + 0.5) / scale
        py = (self.H - np.arange(self.H) - 0.5) / scale       # row 0 = north
        self.gx, self.gy = np.meshgrid(px, py)

    def px(self, x, y):
        return self.margin + x * self.scale, self.margin + self.H - y * self.scale

    def disc(self, x, y, r, color, r_in=0.0):
        d2 = (self.gx - x) ** 2 + (self.gy - y) ** 2
        m = d2 <= r * r
        if r_in > 0:
            m &= d2 >= r_in * r_in
        self.arena[m] = color

    def line(self, p0, p1, color, width=2, dash=None):
        """p0, p1 in world metres. dash = (on_px, off_px) or None."""
        a, b = np.array(self.px(*p0)), np.array(self.px(*p1))
        n = max(2, int(np.hypot(*(b - a)) * 2))
        t = np.linspace(0, 1, n)
        pts = a[None, :] + (b - a)[None, :] * t[:, None]
        if dash:
            along = t * np.hypot(*(b - a))
            keep = (along % (dash[0] + dash[1])) < dash[0]
            pts = pts[keep]
        hw = width // 2
        for x, y in pts:
            xi, yi = int(round(x)), int(round(y))
            self.img[max(yi - hw, 0):yi - hw + width, max(xi - hw, 0):xi - hw + width] = color

    def box(self, cx, cy, sx, sy, yaw_deg, color, inset=0.0):
        """Fill a rotated rectangle (world metres), shrunk by `inset` on every side."""
        c, s_ = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
        dx, dy = self.gx - cx, self.gy - cy
        u, v = c * dx + s_ * dy, -s_ * dx + c * dy
        m = (np.abs(u) <= sx / 2 - inset) & (np.abs(v) <= sy / 2 - inset)
        self.arena[m] = color

    def rect(self, rect, color, width=2, dash=None):
        x0, y0, x1, y1 = rect
        for p, q in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                     ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
            self.line(p, q, color, width, dash)

    def text(self, x_px, y_px, s, color=(40, 40, 40), k=2):
        for ch in s:
            glyph = _FONT[ch]
            for r in range(5):
                for c in range(3):
                    if glyph[r * 3 + c] == '1':
                        self.img[y_px + r * k:y_px + r * k + k,
                                 x_px + c * k:x_px + c * k + k] = color
            x_px += 4 * k


def _label_obstacles(cv, layout, kk):
    """Centre cross + id label on every obstacle (plan mode of render_map)."""
    for kind, i, x, y, _r in _items(layout):
        px, py = (int(round(v)) for v in cv.px(x, y))
        cv.img[py - 5:py + 6, px - 1:px + 1] = (0, 0, 0)
        cv.img[py - 1:py + 1, px - 5:px + 6] = (0, 0, 0)
        s = f'{kind[0].upper()}{i + 1}'
        w_, h_ = 4 * kk * len(s) - kk, 5 * kk
        x0, y1 = px - w_ // 2, py - 8
        cv.img[y1 - h_ - 2:y1 + 2, x0 - 3:x0 + w_ + 3] = (250, 248, 240)
        cv.text(x0, y1 - h_, s, k=kk)


def render_map(cfg, layout, png_path, path=None, scale=100, markers=False, plan=False):
    """Top-down PNG of the layout. North (y = +5) at the top, x to the right.

    `plan=True` is the printable set-up plan: a 50 cm grid with labelled metre lines, every
    obstacle's centre marked and labelled with its id (B1.. boulders, C1.. craters, as in
    setup_sheet.md), bigger digits. Without it the output is unchanged.
    """
    margin = 60 if plan else 34
    kk = 3 if plan else 2                                           # glyph scale
    cv = _Canvas(cfg, scale, margin)
    z = cfg['zones']
    cv.arena[:] = (222, 200, 150)                                   # sand
    # zone tints
    ex = z['excavation']
    m = (cv.gx >= ex[0])
    cv.arena[m] = (228, 208, 160)
    # grid every metre, faint
    w, h = cfg['arena']['size']
    if plan:                                                        # 50 cm grid, metre lines bold
        for k in range(1, int(round(w * 2))):
            cv.line((k / 2, 0), (k / 2, h), (206, 184, 136) if k % 2 else (150, 125, 80),
                    1 if k % 2 else 2)
        for k in range(1, int(round(h * 2))):
            cv.line((0, k / 2), (w, k / 2), (206, 184, 136) if k % 2 else (150, 125, 80),
                    1 if k % 2 else 2)
    else:
        for k in range(1, int(w)):
            cv.line((k, 0), (k, h), (206, 184, 136), 1)
        for k in range(1, int(h)):
            cv.line((0, k), (w, k), (206, 184, 136), 1)
    # zones
    dash = (10, 8)
    cv.rect(z['excavation'], (110, 110, 110), 2, dash)
    cv.rect(z['start'], (40, 90, 200), 3)
    cv.rect(z['construction'], (30, 140, 60), 3)
    tb = cfg['berm']
    tx, ty = tb['target_centre']
    for size, col_, d in ((tb['target_size'], (200, 40, 40), None),
                          (tb['target_size_schematic'], (200, 40, 40), (6, 6))):
        cv.rect((tx - size[0] / 2, ty - size[1] / 2, tx + size[0] / 2, ty + size[1] / 2),
                col_, 2, d)
    # obstacles
    for c in layout.get('craters', []):
        r = c['diameter'] / 2
        cv.disc(c['position'][0], c['position'][1], r, (95, 62, 38))
        cv.disc(c['position'][0], c['position'][1], r, (60, 38, 22), r_in=r - 0.03)
    for b in layout.get('boulders', []):
        r = b['diameter'] / 2
        cv.disc(b['position'][0], b['position'][1], r, (60, 60, 60))
        cv.disc(b['position'][0], b['position'][1], r - 0.02, (140, 140, 145))
    if markers:
        for _, px, py, pr in post_circles(cfg):
            cv.disc(px, py, max(pr, 0.03), (150, 30, 150))
    col = cfg['column']
    cx, cy, cr = _column(cfg)
    if col.get('shape', 'cylinder') == 'box':
        yaw = col.get('yaw_deg', 0.0)
        cv.box(cx, cy, *col['size'], yaw, (30, 30, 30))
        cv.box(cx, cy, *col['size'], yaw, (90, 90, 100), inset=0.03)
    else:
        cv.disc(cx, cy, cr, (30, 30, 30))
        cv.disc(cx, cy, cr - 0.03, (90, 90, 100))
    # path
    if path:
        # path points are (x, y) or (x, y, yaw_deg); with yaw the footprint is drawn there
        for p in path:
            if len(p) > 2:
                pts = _footprint_corners(cfg, {'x': p[0], 'y': p[1], 'yaw_deg': p[2]})
                for i in range(4):
                    cv.line(pts[i], pts[(i + 1) % 4], (240, 150, 200), 1)
        for p, q in zip(path[:-1], path[1:]):
            cv.line(p[:2], q[:2], (230, 40, 160), 3)
        for p in path:
            x, y = cv.px(p[0], p[1])
            cv.img[int(y) - 3:int(y) + 4, int(x) - 3:int(x) + 4] = (230, 40, 160)
    # robot footprint + heading arrow
    sp = layout.get('start_pose')
    if sp:
        pts = _footprint_corners(cfg, sp)
        for i in range(4):
            cv.line(pts[i], pts[(i + 1) % 4], (20, 60, 220), 3)
        yaw = math.radians(sp['yaw_deg'])
        length = cfg['randomiser']['robot_footprint'][0]
        tip = (sp['x'] + math.cos(yaw) * length * 0.5, sp['y'] + math.sin(yaw) * length * 0.5)
        cv.line((sp['x'], sp['y']), tip, (20, 60, 220), 3)
        for da in (150, -150):
            a = yaw + math.radians(da)
            cv.line(tip, (tip[0] + 0.15 * math.cos(a), tip[1] + 0.15 * math.sin(a)),
                    (20, 60, 220), 3)
    # border and metre ticks with labels
    cv.rect((0, 0, w, h), (0, 0, 0), 2)
    for k in range(int(w) + 1):
        x, y = cv.px(k, 0)
        cv.img[int(y) + 1:int(y) + 9, int(x) - 1:int(x) + 1] = 0
        s = str(k)
        cv.text(int(x) - 2 * kk * len(s) + kk, int(y) + 12 + kk, s, k=kk)
    for k in range(int(h) + 1):
        x, y = cv.px(0, k)
        cv.img[int(y) - 1:int(y) + 1, int(x) - 9:int(x) - 1] = 0
        s = str(k)
        cv.text(int(x) - 12 - 4 * kk * len(s) + 4, int(y) - (5 * kk) // 2, s, k=kk)
    if plan:
        x, y = cv.px(w, 0)
        cv.text(int(x) + 14, int(y) + 12, 'X', k=kk)                # axis names
        x, y = cv.px(0, h)
        cv.text(int(x) - 6 * kk, int(y) - 12 - 5 * kk, 'Y', k=kk)
        _label_obstacles(cv, layout, 4)
    write_png(png_path, cv.img)


# ----------------------------------------------------------------------------- set-up sheet

_COMPASS = ('east', 'north-east', 'north', 'north-west', 'west', 'south-west', 'south',
            'south-east')


def _cm(v):
    """Metres -> whole centimetres, rounding half up (never banker's rounding)."""
    return int(math.floor(v * 100 + 0.5))


def _cm1(v):
    return f'{v * 100:.1f}'


def _heading_words(yaw_deg):
    """Plain words for a heading: 'north-east' etc. in the sheet's compass (east = +x)."""
    return _COMPASS[int(((yaw_deg % 360.0) + 22.5) // 45) % 8]


def _zone(cfg, x):
    """'excavation' if x is in the digging half (x >= excavation start), else 'obstacle'."""
    return 'excavation' if x >= cfg['zones']['excavation'][0] else 'obstacle'


def setup_sheet(cfg, layout, out_dir, markers=False):
    """Write the human set-up sheet for building this layout on a real mock arena.

    Writes `<out_dir>/<name>/truth/setup_sheet.md` and `setup_plan.png` (it reveals obstacle
    positions, so it lives with the truth) and returns the markdown path. Deterministic; no other
    output file is touched.
    """
    from pathlib import Path
    root = Path(out_dir) / layout['name'] / 'truth'
    root.mkdir(parents=True, exist_ok=True)
    w, h = cfg['arena']['size']
    rows = []                                   # (x, y, kind order, i, id, kind, cells, zone)
    for i, b in enumerate(layout.get('boulders', [])):
        rows.append((b['position'][0], b['position'][1], 0, i, f'B{i + 1}', 'boulder',
                     [str(_cm(b['diameter'])), str(_cm(b['height'])), '-', '-', '-'],
                     _zone(cfg, b['position'][0])))
    for i, c in enumerate(layout.get('craters', [])):
        cp = crater_params(cfg, layout, i, layout.get('seed'))
        rows.append((c['position'][0], c['position'][1], 1, i, f'C{i + 1}', 'crater',
                     [str(_cm(c['diameter'])), '-', _cm1(cp['depth_eff']), _cm1(cp['rim']),
                      str(_cm(cp['outer_radius'] - c['diameter'] / 2))],
                     _zone(cfg, c['position'][0])))
    rows.sort(key=lambda r: (_cm(r[0]), _cm(r[1]), r[2], r[3]))
    sp = layout['start_pose']
    yaw = sp['yaw_deg'] % 360.0
    fl, fw = cfg['randomiser']['robot_footprint']
    col = cfg['column']
    cx, cy = col['centre']
    diag = math.hypot(w, h)
    n_b, n_c = len(layout.get('boulders', [])), len(layout.get('craters', []))
    corners = [f'{_cm(px)}, {_cm(py)}' for px, py in _footprint_corners(cfg, sp)]
    seed = layout.get('seed')

    L_ = [f'# Mock arena set-up sheet: {layout["name"]}', '',
          f'Seed: {seed if seed is not None else "none (fixed layout)"}. '
          f'{n_b} boulders, {n_c} craters. All numbers are centimetres from the origin.', '',
          '**Keep this sheet away from the robot software and its developers during tests: '
          'it lists where every obstacle is.**', '',
          '## 1. Origin and directions', '',
          f'- Wall height: {_cm(cfg["arena"]["wall"]["height"])} cm (ASSUMED, not in the '
          'rulebook; our best guess).',
          '- **Origin = the inner corner where the west and south walls meet, the start-zone '
          'corner.** Measure from the inside faces of the walls, never from the outside.',
          f'- **x** runs along the long {w:g} m wall (west wall corner towards the far end). '
          f'**y** runs along the short {h:g} m wall, away from the origin.',
          '- Seen from above with the origin at bottom-left: x goes right (east), y goes up '
          '(north). "North/east" on this sheet only mean this; ignore the real compass.',
          '- Mark each centre with a peg or tape cross, then place the obstacle on it. '
          'Measure x along the long wall and y at right angles from it (use a set square or '
          'the 3-4-5 trick), not diagonally.', '',
          '## 2. Obstacles (lay them out in this order, walking along the long wall)', '',
          '| Done | Id | Type | x (cm) | y (cm) | Diameter (cm) | Rock height (cm) '
          '| Crater depth (cm) | Rim height (cm) | Rim out to (cm from edge) | Zone |',
          '|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|']
    for x, y, _k, _i, oid, kind, rest, zone in rows:
        L_.append(f'| [ ] | {oid} | {kind} | {_cm(x)} | {_cm(y)} | ' + ' | '.join(rest)
                  + f' | {zone} |')
    L_ += ['',
           '- **Boulder (B):** pick a rock about this diameter and height; centre its footprint '
           'on the mark and sit it on the sand (it may sink 1-2 cm).',
           '- **Crater (C):** dig a bowl of this diameter and pile the dug sand round the edge as '
           'a low rim. Diameter is across the bowl, edge to edge.',
           '- **Crater depth** is measured from the flat, undisturbed sand level, not from the '
           'rim top. To measure: lay a straight ruler or batten across the crater, resting on '
           'the flat sand OUTSIDE the rim (or on two equal blocks). Measure from the batten '
           'down to the deepest point of the bowl. That number is the depth. If you rest on '
           'blocks, subtract the block height.',
           '- **Rim height** is the height of the rim top above the flat sand. **Rim out to** is '
           'roughly how far the raised sand reaches beyond the crater edge; blend it into the '
           'flat sand there.',
           '- Each Id (B1, C2, ...) is printed on `setup_plan.png` at the obstacle centre (+). '
           'On the plan: blue square = the 2 x 2 m start zone with the robot outline and arrow; '
           'green and red boxes = construction zone and target box (reference only, nothing '
           'to build)' + ('; purple dots = marker posts.'
                          if markers and marker_list(cfg) else '.'),
           '- Zone: `obstacle` = x under 500 cm; `excavation` = x from 500 cm (the digging '
           'half). Your own tolerances, not rules: positions +/-2 cm, rock size +/-3 cm, '
           'crater diameter +/-3 cm, depth +/-1 cm.', '',
           '## 3. Start pose (robot)', '',
           f'- Centre of the robot base: **x = {_cm(sp["x"])} cm, y = {_cm(sp["y"])} cm** '
           f'(robot footprint {fl * 100:.0f} x {fw * 100:.0f} cm).',
           f"- The robot's front points along the heading; its {fl * 100:.0f} cm side lies "
           f'along the heading. Footprint corners in cm, for outlining with tape: '
           f'front-left ({corners[0]}), front-right ({corners[1]}), '
           f'rear-right ({corners[2]}), rear-left ({corners[3]}).',
           f'- Heading: **{yaw:.0f} degrees counter-clockwise from the +x direction** (the long '
           f'wall, pointing away from the origin), seen from above. In words: facing roughly '
           f'**{_heading_words(yaw)}**.',
           '- To set it: stand over the robot looking along the long wall away from the origin, '
           'point the robot that way, then turn it left (counter-clockwise) by this angle; the '
           'front of the robot is where it points.', '',
           '## 4. Central column', '']
    if col.get('shape', 'cylinder') == 'box':
        L_.append(f'- Box {_cm(col["size"][0])} x {_cm(col["size"][1])} cm, centre '
                  f'**x = {_cm(cx)} cm, y = {_cm(cy)} cm**, '
                  f'{_cm(col["height"])} cm tall (or as tall as you have).')
    else:
        L_.append(f'- Cylinder diameter {_cm(col["size"][0])} cm, centre '
                  f'**x = {_cm(cx)} cm, y = {_cm(cy)} cm**, {_cm(col["height"])} cm tall.')
    L_.append('- If your arena has no such column, mark its footprint with tape or place a box of '
              'this size.')
    L_.append('')
    nxt = 5
    if markers and marker_list(cfg):
        L_ += [f'## {nxt}. ArUco markers', '',
               f'Dictionary {cfg["markers"].get("dictionary", "?")}, printed tag '
               f'{_cm(cfg["markers"]["size"])} cm (black square); add the white border.', '',
               '| Id | x (cm) | y (cm) | Centre height (cm) | Faces | Mount |',
               '|---:|---:|---:|---:|---|---|']
        for m in marker_list(cfg):
            mx, my = marker_placement(cfg, m)
            L_.append(f'| {m["id"]} | {_cm(mx)} | {_cm(my)} | {_cm(m["position"][2])} | '
                      f'{_heading_words(m["yaw_deg"])} | '
                      f'{"on the wall" if wall_mounted(cfg, m) else "on a thin post"} |')
        L_.append('')
        nxt += 1
    if not (markers and marker_list(cfg)):
        L_ += ['Markers: not included in this sheet (generate with --markers to get the '
               'marker table).', '']
    L_ += [f'## {nxt}. Check before running', '',
           f'- [ ] Walls form a clean {w:g} x {h:g} m rectangle: both diagonals measure '
           f'{_cm(diag)} cm.',
           '- [ ] Sand tilled and levelled; no old footprints or craters left over.',
           f'- [ ] Every one of the {n_b + n_c} obstacles in the table is placed and ticked; '
           'no extra rocks or objects.',
           '- [ ] Re-measure 3 obstacles from the origin with the tape; each within 2 cm.',
           '- [ ] Crater depths checked with the batten method above (from the flat sand level).',
           '- [ ] Robot placed at the start pose above (centre, heading); motors '
           'stay disabled until everyone is clear.',
           '- [ ] Remove all pegs and tape marks before the robot runs (they are not in the '
           'real arena and the robot could see them).',
           '- [ ] Photograph the finished field from above/one corner and file it with '
           f'`{layout["name"]}`.', '',
           '![plan](setup_plan.png)', '']
    md = root / 'setup_sheet.md'
    md.write_text('\n'.join(L_))
    render_map(cfg, layout, str(root / 'setup_plan.png'), scale=200, markers=markers, plan=True)
    return md
