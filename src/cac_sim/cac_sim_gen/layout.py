"""Layouts: seeded randomiser, rule checker, path-exists check, ground truth and top-down map.

A layout is a plain dict (see DESIGN.md). Everything here is deterministic for a given seed.
RNG streams: obstacles = default_rng([seed, 2]), start pose = default_rng([seed, 3]).
"""
import heapq
import math

import numpy as np

from .png import write_png

OBSTACLE_STREAM = 2
START_STREAM = 3
EPS = 1e-6
MAX_ATTEMPTS = 300
SLALOM_BAND = (2.0, 4.9)      # x range the robot must cross; a share of obstacles is put here
SLALOM_BIAS = 0.6             # probability an obstacle-zone obstacle is drawn from the band
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


def _items(layout):
    """All obstacles as (kind, index, x, y, radius)."""
    out = []
    for i, b in enumerate(layout.get('boulders', [])):
        out.append(('boulder', i, b['position'][0], b['position'][1], b['diameter'] / 2))
    for i, c in enumerate(layout.get('craters', [])):
        out.append(('crater', i, c['position'][0], c['position'][1], c['diameter'] / 2))
    return out


def _column(cfg):
    col = cfg['column']
    return col['centre'][0], col['centre'][1], col['radius']


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
    """Allowed (x_lo, x_hi, y_lo, y_hi) of the robot centre so the rotated footprint fits."""
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
    Position is sampled given the yaw; a yaw with no feasible position is resampled.
    """
    for _ in range(1000):
        yaw = float(rng.uniform(-180.0, 180.0))
        xl, xh, yl, yh = _pose_interval(cfg, yaw)
        if xh - xl < 0.004 or yh - yl < 0.004:
            continue
        # keep 1 mm inside so rounding to mm cannot push the footprint out
        x = float(rng.uniform(xl + 0.001, xh - 0.001))
        y = float(rng.uniform(yl + 0.001, yh - 0.001))
        return {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(yaw, 2)}
    raise RuntimeError('robot footprint does not fit in the start zone at any yaw')


# ----------------------------------------------------------------------------- randomiser

def _sample_obstacle(cfg, rng, kind, zone, placed, col):
    """Draw one obstacle centre/size in `zone` that clears everything already placed."""
    rc = cfg['randomiser']
    gap, off = rc['min_gap_between_obstacles'], rc['keep_off_walls']
    w, h = cfg['arena']['size']
    spec = cfg['boulders'] if kind == 'boulder' else cfg['craters']
    z = cfg['zones']
    slack = 0.002
    for _ in range(PLACE_TRIES):
        d = float(rng.uniform(*spec['diameter']))
        r = d / 2
        if zone == 'obstacle':
            if rng.random() < SLALOM_BIAS:
                x = float(rng.uniform(*SLALOM_BAND))
            else:
                x = float(rng.uniform(off + r, z['excavation'][0]))
        else:
            x = float(rng.uniform(z['excavation'][0], w - off - r))
        y = float(rng.uniform(off + r, h - off - r))
        if x < off + r or x > w - off - r:
            continue
        if zone == 'obstacle' and x >= z['excavation'][0]:
            continue
        if _circle_rect_dist(x, y, z['start']) < r + 0.05:
            continue
        if _circle_rect_dist(x, y, z['construction']) < r + 0.05:
            continue
        ok = True
        for (px, py, pr) in placed + [col]:
            if math.hypot(x - px, y - py) - r - pr < gap + slack:
                ok = False
                break
        if not ok:
            continue
        item = {'position': [round(x, 3), round(y, 3)], 'diameter': round(d, 3)}
        if kind == 'boulder':
            item['height'] = round(float(rng.uniform(*spec['height'])), 3)
        else:
            item['depth'] = round(float(rng.uniform(*spec['depth'])), 3)
            rim = spec.get('rim_height')
            if rim:
                item['rim'] = round(float(rng.uniform(*rim)), 3)
        return item
    return None


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
    # obstacle zone first (it decides the route), interleaved so both kinds spread
    jobs = []
    for zone in ('obstacle', 'excavation'):
        nb, nc = n[('boulder', zone)], n[('crater', zone)]
        seq = ['boulder'] * nb + ['crater'] * nc
        order = rng.permutation(len(seq))
        jobs += [(seq[i], zone) for i in order]
    col = _column(cfg)
    placed, boulders, craters = [], [], []
    for kind, zone in jobs:
        item = _sample_obstacle(cfg, rng, kind, zone, placed, col)
        if item is None:
            return None
        placed.append((item['position'][0], item['position'][1], item['diameter'] / 2))
        (boulders if kind == 'boulder' else craters).append(item)
    return boulders, craters


def random_layout(cfg, seed, stats=None):
    """Seeded random layout that passes check_layout(strict=True), path included.

    `stats` (optional dict) receives {'attempts': n, 'path_rejects': k}.
    """
    rng = np.random.default_rng([seed, OBSTACLE_STREAM])
    pose = random_start_pose(cfg, np.random.default_rng([seed, START_STREAM]))
    path_rejects = 0
    for attempt in range(1, MAX_ATTEMPTS + 1):
        res = _one_attempt(cfg, rng)
        if res is None:
            continue
        layout = {'name': f'seed_{seed:04d}', 'seed': int(seed), 'boulders': res[0],
                  'craters': res[1], 'start_pose': dict(pose)}
        issues = check_layout(cfg, layout, strict=True)
        if not issues:
            if stats is not None:
                stats.update(attempts=attempt, path_rejects=path_rejects)
            return layout
        if any('no path' in s for s in issues):
            path_rejects += 1
    raise RuntimeError(f'no valid layout for seed {seed} in {MAX_ATTEMPTS} attempts')


# ----------------------------------------------------------------------------- path check

def _blocked_grid(cfg, layout, inflate, res):
    w, h = cfg['arena']['size']
    nx, ny = int(round(w / res)), int(round(h / res))
    xs = (np.arange(nx) + 0.5) * res
    ys = (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys, indexing='ij')           # (nx, ny)
    blocked = (gx < inflate) | (gx > w - inflate) | (gy < inflate) | (gy > h - inflate)
    circles = [(x, y, r) for _, _, x, y, r in _items(layout)] + [_column(cfg)]
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
    """Can the robot get from the start zone to the excavation zone?

    Occupancy grid at `resolution` (5 cm). Boulders, craters, the column and the walls are
    inflated by `inflate` metres, default = robot half-width + clearance (inscribed-radius
    check: the robot is treated as a disc, valid because a rover can turn in place). A 1.5 m long
    robot turning in a tight spot sweeps a disc of radius ~0.84 m (half the diagonal) plus
    clearance; pass inflate=0.95 for that conservative variant. Returns (ok, polyline of (x, y)).
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


# ----------------------------------------------------------------------------- checker

def check_layout(cfg, layout, strict=True):
    """Return a list of human-readable violations (empty = valid).

    strict=True checks every rule in arena.yaml. strict=False reports only hard physical
    problems: overlapping obstacles, out of the arena, start footprint outside the start zone,
    no path. Column conflicts are strict-only: the column position is ASSUMED, not measured.
    """
    v = []
    z = cfg['zones']
    rc = cfg['randomiser']
    w, h = cfg['arena']['size']
    items = _items(layout)
    col = _column(cfg)

    # out of arena (hard)
    for kind, i, x, y, r in items:
        if x - r < -EPS or y - r < -EPS or x + r > w + EPS or y + r > h + EPS:
            v.append(f'{_name(kind, i)} extends outside the arena')

    # overlaps (hard) and gaps (strict)
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
        for kind, i, x, y, r in items:
            g = math.hypot(x - col[0], y - col[1]) - r - col[2]
            if g < rc['min_gap_between_obstacles'] - EPS:
                v.append(f'gap {g:.2f} m between {_name(kind, i)} and the column '
                         f'< {rc["min_gap_between_obstacles"]} m')
            dw = min(x - r, w - x - r, y - r, h - y - r)
            if dw < rc['keep_off_walls'] - EPS:
                v.append(f'{_name(kind, i)} is {dw:.2f} m from a wall '
                         f'(< {rc["keep_off_walls"]} m)')
            if _circle_rect_dist(x, y, z['start']) < r - EPS:
                v.append(f'{_name(kind, i)} intrudes into the start zone')
            if _circle_rect_dist(x, y, z['construction']) < r - EPS:
                v.append(f'{_name(kind, i)} intrudes into the construction zone')

        # counts per zone
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
            for k, zone in ((kind, 'start'), (kind, 'construction')):
                if count(k, zone):
                    v.append(f'{count(k, zone)} {plural} centred in the {zone} zone')

        # sizes vs ranges
        for i, b in enumerate(layout.get('boulders', [])):
            _range(v, f'boulder {i} diameter', b['diameter'], cfg['boulders']['diameter'])
            _range(v, f'boulder {i} height', b['height'], cfg['boulders']['height'])
        for i, c in enumerate(layout.get('craters', [])):
            _range(v, f'crater {i} diameter', c['diameter'], cfg['craters']['diameter'])
            _range(v, f'crater {i} depth', c['depth'], cfg['craters']['depth'])
            if 'rim' in c:
                _range(v, f'crater {i} rim', c['rim'], cfg['craters']['rim_height'])

    # start pose footprint (hard: inside the zone; strict: inside the shrunk zone)
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

    ok, _ = path_exists(cfg, layout)
    if not ok:
        v.append('no path from the start zone to the excavation zone for the robot footprint')
    return v


def _range(v, label, value, rng_):
    if not rng_[0] - EPS <= value <= rng_[1] + EPS:
        v.append(f'{label} {value} outside {rng_[0]}..{rng_[1]}')


# ----------------------------------------------------------------------------- truth

def truth(cfg, layout):
    """Ground truth for the test harness and scoring. Plain python types only."""
    z = cfg['zones']
    obstacles = []
    for b in layout.get('boulders', []):
        pos = [float(b['position'][0]), float(b['position'][1])]
        obstacles.append({'type': 'boulder', 'position': pos,
                          'radius': float(b['diameter']) / 2, 'height': float(b['height'])})
    for c in layout.get('craters', []):
        pos = [float(c['position'][0]), float(c['position'][1])]
        obstacles.append({'type': 'crater', 'position': pos,
                          'radius': float(c['diameter']) / 2, 'depth': float(c['depth'])})
    ok, path = path_exists(cfg, layout)
    res = {'exists': bool(ok), 'path': [[float(x), float(y)] for x, y in path],
           'inflate': float(_default_inflate(cfg))}
    if ok:
        res['bottleneck_width'] = round(float(_bottleneck_width(cfg, layout)), 3)
    col = cfg['column']
    sp = layout['start_pose']
    return {
        'name': layout.get('name'),
        'seed': layout.get('seed'),
        'zones': {k: [float(a) for a in z[k]] for k in z},
        'target_box': {'centre': [float(a) for a in cfg['berm']['target_centre']],
                       'size': [float(a) for a in cfg['berm']['target_size']],
                       'size_schematic': [float(a) for a in cfg['berm']['target_size_schematic']]},
        'column': {'position': [float(a) for a in col['centre']],
                   'radius': float(col['radius']), 'height': float(col['height'])},
        'obstacles': obstacles,
        'start_pose': {'x': float(sp['x']), 'y': float(sp['y']), 'yaw_deg': float(sp['yaw_deg'])},
        'path_exists': res,
    }


# ----------------------------------------------------------------------------- map

_FONT = {
    '0': '111101101101111', '1': '010110010010111', '2': '111001111100111',
    '3': '111001111001111', '4': '101101111001001', '5': '111100111001111',
    '6': '111100111101111', '7': '111001001001001', '8': '111101111101111',
    '9': '111101111001111',
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


def render_map(cfg, layout, png_path, path=None, scale=100):
    """Top-down PNG of the layout. North (y = +5) at the top, x to the right."""
    margin = 34
    cv = _Canvas(cfg, scale, margin)
    z = cfg['zones']
    cv.arena[:] = (222, 200, 150)                                   # sand
    # zone tints
    ex = z['excavation']
    m = (cv.gx >= ex[0])
    cv.arena[m] = (228, 208, 160)
    # grid every metre, faint
    w, h = cfg['arena']['size']
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
    cx, cy, cr = _column(cfg)
    cv.disc(cx, cy, cr, (30, 30, 30))
    cv.disc(cx, cy, cr - 0.03, (90, 90, 100))
    # path
    if path:
        for p, q in zip(path[:-1], path[1:]):
            cv.line(p, q, (230, 40, 160), 3)
        for p in path:
            x, y = cv.px(*p)
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
        cv.text(int(x) - 4 * len(s) + 2, int(y) + 14, s)
    for k in range(int(h) + 1):
        x, y = cv.px(0, k)
        cv.img[int(y) - 1:int(y) + 1, int(x) - 9:int(x) - 1] = 0
        s = str(k)
        cv.text(int(x) - 12 - 8 * len(s) + 4, int(y) - 5, s)
    write_png(png_path, cv.img)
