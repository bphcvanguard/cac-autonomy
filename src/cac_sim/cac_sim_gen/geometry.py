"""Arena geometry: sand height field, terrain and rock meshes, OBJ writer, generated textures.

numpy + stdlib only. Everything is deterministic for a given seed; each subsystem draws from its
own RNG stream (`numpy.random.default_rng([seed, STREAM_ID, ...])`), see DESIGN.md.
"""
import math
from pathlib import Path
import struct
import zlib

import numpy as np

from .mesh import Mesh

# RNG stream ids owned by this module. layout.py uses [seed, 2] and [seed, 3] and world.py
# [seed, 7]; numpy SeedSequence treats [s, 2] and [s, 2, 0] alike, so every id here is >= 20.
STREAM_TILL = 20     # tilled-sand relief
STREAM_RIM = 21      # crater rim height ratios: [seed, 21, crater index]
STREAM_DEPTH = 22    # crater depth ratios: [seed, 22, crater index]
STREAM_UV = 23       # terrain UV warp (anti-tiling)
STREAM_SAND_TEX = 24
STREAM_SAND_NRM = 25
STREAM_ROCK_NRM = 26
STREAM_ROCK = 30     # rock shape: [seed, 30, rock index]
STREAM_ROCK_TEX = 50  # rock texture variant k: [seed, 50 + k]

UV_TILE = 1.0        # metres of terrain per texture repeat


def _seed(seed):
    return int(seed) if seed is not None else 0


# ---------------------------------------------------------------------------------------------
# Height field
# ---------------------------------------------------------------------------------------------

def _till_params(seed):
    """Random parameters of the tilled relief (amplitudes sum to 1, scaled by roughness later)."""
    rng = np.random.default_rng([_seed(seed), STREAM_TILL])
    p = {}
    # Low-frequency undulation: a few long waves, wavelength 1-3 m.
    n = 4
    ang = rng.uniform(0, np.pi, n)
    lam = rng.uniform(1.0, 3.0, n)
    p['low'] = (np.cos(ang) * 2 * np.pi / lam, np.sin(ang) * 2 * np.pi / lam,
                rng.uniform(0, 2 * np.pi, n), np.full(n, 0.35 / n))
    # Furrows: parallel ridges at a random angle, spacing 0.22-0.32 m, slight wobble.
    th = rng.uniform(0, np.pi)
    p['furrow'] = (math.cos(th), math.sin(th), rng.uniform(0.22, 0.32), rng.uniform(0, 2 * np.pi),
                   rng.uniform(1.5, 3.0), rng.uniform(0, 2 * np.pi), 0.45)
    # Fine noise: short waves, wavelength 0.10-0.20 m (anything shorter aliases on the 5 cm mesh;
    # the albedo texture carries the grain).
    n = 10
    ang = rng.uniform(0, np.pi, n)
    lam = rng.uniform(0.10, 0.20, n)
    p['fine'] = (np.cos(ang) * 2 * np.pi / lam, np.sin(ang) * 2 * np.pi / lam,
                 rng.uniform(0, 2 * np.pi, n), np.full(n, 0.20 / n))
    return p


def _waves(x, y, params):
    kx, ky, ph, amp = params
    z = np.zeros_like(x)
    for i in range(len(kx)):
        z += amp[i] * np.sin(kx[i] * x + ky[i] * y + ph[i])
    return z


def _tilled(x, y, seed):
    """Tilled-sand relief in [-1, 1] (unit amplitude)."""
    p = _till_params(seed)
    c, s, lam, ph, wob_lam, wob_ph, amp = p['furrow']
    u = c * x + s * y          # across the furrows
    v = -s * x + c * y         # along the furrows
    u = u + 0.04 * np.sin(2 * np.pi * v / wob_lam + wob_ph)
    # Rounded ridges with sharper troughs (tine marks): 1 - 2|sin|^1.5 shaped, mean ~0.
    t = np.abs(np.sin(np.pi * u / lam + ph)) ** 1.5
    furrow = amp * (1.0 - 2.0 * t)
    return _waves(x, y, p['low']) + furrow + _waves(x, y, p['fine'])


DEFAULT_MAX_SLOPE_DEG = 38.0   # dry sand cannot hold a steeper face
DEFAULT_DEPTH_RATIO = (0.10, 0.25)
DEFAULT_RIM_RATIO = (0.04, 0.08)
RIM_CREST = 1.05               # rim crest radius / R  (R = D/2)
RIM_OUTER = 0.35               # outer-flank width / R: terrain is untouched beyond 1.40 R


def _shape(diameter, depth, rim, max_slope_deg):
    """Return (R, crest_r, flank_w, depth_eff, rim_eff, steepest_deg) of one crater.

    Profile (smoothstep S(t) = 3t^2 - 2t^3, r = distance from centre):
      r <= crest_r:  z = -depth + (depth + rim) * S(r / crest_r)   bowl: parabolic near the
                     centre, steepest mid-wall, zero slope at the rim crest (no lip crease)
      crest_r < r < crest_r + flank_w:  z = rim * (1 - S((r - crest_r) / flank_w))   outer flank
      beyond:        z = 0
    Steepest slope = max(1.5 (depth + rim) / crest_r, 1.5 rim / flank_w). If that exceeds
    `max_slope_deg`, depth and rim are scaled down together until it does not.
    """
    R = 0.5 * float(diameter)
    rc, wo = RIM_CREST * R, RIM_OUTER * R
    d, h = max(0.0, float(depth)), max(0.0, float(rim))
    steep = max(1.5 * (d + h) / rc, 1.5 * h / wo)
    cap = math.tan(math.radians(float(max_slope_deg)))
    if steep > cap:
        d, h, steep = d * cap / steep, h * cap / steep, cap
    return R, rc, wo, d, h, math.degrees(math.atan(steep))


def crater_outer_radius(diameter):
    """Radius beyond which the terrain is exactly the undisturbed tilled surface: 1.40 * D/2.

    The rim crest is at 1.05 * D/2 and its outer flank reaches z = 0 at 1.40 * D/2; the tilled
    relief blends back in over D/2 .. 1.40 * D/2. Independent of rim height and slope.
    """
    return (RIM_CREST + RIM_OUTER) * 0.5 * float(diameter)


def crater_profile(diameter, depth, rim, max_slope_deg=DEFAULT_MAX_SLOPE_DEG):
    """Radial height function z(r) of one crater relative to undisturbed sand (no tilled relief).

    Shallow bowl + soft raised rim, slope-continuous everywhere, see _shape(). Returns f(r) ->
    ndarray, vectorised.
    """
    _R, rc, wo, d, h, _s = _shape(diameter, depth, rim, max_slope_deg)

    def smooth(t):
        t = np.clip(t, 0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)

    def z_of_r(r):
        r = np.asarray(r, dtype=float)
        inner = -d + (d + h) * smooth(r / rc)
        outer = h * (1.0 - smooth((r - rc) / wo))
        return np.where(r <= rc, inner, outer)
    return z_of_r


def crater_params(cfg, layout, i, seed):
    """Resolve crater `i` of `layout` to the exact numbers the terrain is built from.

    Returns dict(position, diameter, depth (requested), depth_eff, rim, max_slope_deg_eff,
    outer_radius).
    Requested depth: layout 'depth' (m) if given, else layout 'depth_ratio' * D, else a ratio drawn
    uniformly in craters.depth_ratio from stream [seed, 22, i]. Rim: layout 'rim' (m) if given,
    else craters.rim_ratio * D drawn from stream [seed, 21, i]. `rim` in the result is the rim
    height actually built (after the slope cap). Use this in layout checks, world building and
    truth so every module sees the same crater.
    """
    c = (layout.get('craters') or [])[i]
    ccfg = cfg['craters']
    diameter = float(c['diameter'])
    if c.get('depth') is not None:
        depth = float(c['depth'])
    else:
        ratio = c.get('depth_ratio')
        if ratio is None:
            lo, hi = ccfg.get('depth_ratio', DEFAULT_DEPTH_RATIO)
            ratio = np.random.default_rng([_seed(seed), STREAM_DEPTH, i]).uniform(lo, hi)
        depth = float(ratio) * diameter
    if c.get('rim') is not None:
        rim = float(c['rim'])
    else:
        lo, hi = ccfg.get('rim_ratio', DEFAULT_RIM_RATIO)
        rim = float(np.random.default_rng([_seed(seed), STREAM_RIM, i]).uniform(lo, hi)) * diameter
    cap = float(ccfg.get('max_slope_deg', DEFAULT_MAX_SLOPE_DEG))
    _R, _rc, _wo, d, h, steep = _shape(diameter, depth, rim, cap)
    return {
        'position': [float(c['position'][0]), float(c['position'][1])],
        'diameter': diameter, 'depth': depth,
        'depth_eff': min(d, float(cfg['arena']['sand_depth'])),
        'rim': h, 'max_slope_deg_eff': steep,
        'outer_radius': crater_outer_radius(diameter),
    }


def _crater_params(cfg, layout, seed):
    """[(cx, cy, diameter, depth_eff, rim_eff)] for every crater in the layout."""
    out = []
    for i in range(len(layout.get('craters') or [])):
        p = crater_params(cfg, layout, i, seed)
        out.append((p['position'][0], p['position'][1], p['diameter'], p['depth_eff'], p['rim']))
    return out


def surface_height(cfg, layout, x, y, seed):
    """Sand surface height z(x, y) [m]; z = 0 is undisturbed sand. Vectorised over x, y.

    Outside every crater_outer_radius() the result is exactly the tilled relief. Inside D/2 the
    relief is absent (the hole was dug); between D/2 and the outer radius it blends back in.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    x, y = np.broadcast_arrays(x, y)
    rough = float(cfg['sand']['tilled_roughness'])
    sand_depth = float(cfg['arena']['sand_depth'])

    z = np.zeros(x.shape)
    fade = np.ones(x.shape)
    for cx, cy, diameter, depth, rim in _crater_params(cfg, layout, seed):
        d = np.hypot(x - cx, y - cy)
        # depth/rim are already slope-capped; 90 deg means "no further cap".
        z += crater_profile(diameter, depth, rim, 90.0)(d)
        R = 0.5 * diameter
        t = np.clip((d - R) / (crater_outer_radius(diameter) - R), 0.0, 1.0)
        fade *= t * t * (3.0 - 2.0 * t)
    z += rough * fade * _tilled(x, y, seed)
    return np.maximum(z, -sand_depth)


# ---------------------------------------------------------------------------------------------
# Terrain mesh
# ---------------------------------------------------------------------------------------------

def _vertex_normals(v, f):
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])   # area-weighted
    n = np.zeros_like(v)
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1.0
    return n / ln


def _refine_mask(nx, ny, dx, dy, craters, margin):
    ic = (np.arange(nx) + 0.5) * dx
    jc = (np.arange(ny) + 0.5) * dy
    gx, gy = np.meshgrid(ic, jc, indexing='ij')
    mask = np.zeros((nx, ny), dtype=bool)
    half_diag = 0.5 * math.hypot(dx, dy)
    for cx, cy, diameter, _depth, _rim in craters:
        reach = crater_outer_radius(diameter) + margin + half_diag
        mask |= np.hypot(gx - cx, gy - cy) < reach
    return mask


def _count_tris(mask, k):
    nx, ny = mask.shape
    fine = int(mask.sum())
    pad = np.zeros((nx + 2, ny + 2), dtype=bool)
    pad[1:-1, 1:-1] = mask
    nb = (pad[:-2, 1:-1].astype(int) + pad[2:, 1:-1] + pad[1:-1, :-2] + pad[1:-1, 2:])
    coarse = ~mask
    plain = int((coarse & (nb == 0)).sum())
    trans = coarse & (nb > 0)
    # Fan around a centre vertex: 4 corners + (k-1) extra points per fine-facing edge.
    return fine * 2 * k * k + plain * 2 + int((4 + (k - 1) * nb[trans]).sum())


def build_terrain(cfg, layout, seed, res=0.05, fine_res=0.015, max_triangles=80000):
    """Triangulated sand surface over exactly [0, W] x [0, H].

    A regular grid of `res` cells; cells around each crater are split k x k (k = ceil(res /
    fine_res)) so crater walls are smooth. Coarse cells that touch fine cells are fanned around a
    centre vertex through every vertex on their edges, so there are no T-junctions (no cracks).
    If the result would exceed `max_triangles`, k is reduced until it fits.
    """
    W, H = (float(s) for s in cfg['arena']['size'])
    nx = max(1, int(math.ceil(W / res - 1e-9)))
    ny = max(1, int(math.ceil(H / res - 1e-9)))
    dx, dy = W / nx, H / ny
    craters = _crater_params(cfg, layout, seed)
    mask = _refine_mask(nx, ny, dx, dy, craters, margin=0.02)
    k = max(1, int(math.ceil(res / fine_res - 1e-9)))
    while k > 1 and max_triangles and _count_tris(mask, k) > max_triangles:
        k -= 1
    if k == 1:
        mask[:] = False

    L = 2 * k                     # lattice points per coarse cell (doubled so centres are integer)
    stride = ny * L + 1
    keys = {}
    tris = []

    def vid(a, b):
        key = a * stride + b
        i = keys.get(key)
        if i is None:
            i = keys[key] = len(keys)
        return i

    pad = np.zeros((nx + 2, ny + 2), dtype=bool)
    pad[1:-1, 1:-1] = mask
    for i in range(nx):
        for j in range(ny):
            a0, b0 = i * L, j * L
            if mask[i, j]:
                for p in range(k):
                    for q in range(k):
                        a, b = a0 + 2 * p, b0 + 2 * q
                        v00, v10 = vid(a, b), vid(a + 2, b)
                        v01, v11 = vid(a, b + 2), vid(a + 2, b + 2)
                        if (p + q) % 2:
                            tris += [(v00, v10, v11), (v00, v11, v01)]
                        else:
                            tris += [(v00, v10, v01), (v10, v11, v01)]
                continue
            south, north = pad[i + 1, j], pad[i + 1, j + 2]
            west, east = pad[i, j + 1], pad[i + 2, j + 1]
            if not (south or north or west or east):
                v00, v10 = vid(a0, b0), vid(a0 + L, b0)
                v01, v11 = vid(a0, b0 + L), vid(a0 + L, b0 + L)
                if (i + j) % 2:
                    tris += [(v00, v10, v11), (v00, v11, v01)]
                else:
                    tris += [(v00, v10, v01), (v10, v11, v01)]
                continue
            # Transition cell: CCW boundary polygon, fanned from the cell centre.
            ring = []
            step = 2
            for t in range(0, L, step if south else L):        # south edge, west -> east
                ring.append((a0 + t, b0))
            for t in range(0, L, step if east else L):         # east edge, south -> north
                ring.append((a0 + L, b0 + t))
            for t in range(0, L, step if north else L):        # north edge, east -> west
                ring.append((a0 + L - t, b0 + L))
            for t in range(0, L, step if west else L):         # west edge, north -> south
                ring.append((a0, b0 + L - t))
            c = vid(a0 + k, b0 + k)
            ids = [vid(a, b) for a, b in ring]
            for m in range(len(ids)):
                tris.append((c, ids[m], ids[(m + 1) % len(ids)]))

    kv = np.fromiter(keys.keys(), dtype=np.int64, count=len(keys))
    order = np.fromiter(keys.values(), dtype=np.int64, count=len(keys))
    lat = np.empty((len(keys), 2), dtype=np.int64)
    lat[order, 0] = kv // stride
    lat[order, 1] = kv % stride
    x = lat[:, 0] * (dx / L)
    y = lat[:, 1] * (dy / L)
    # Snap the far border exactly onto W / H (no float drift at the walls).
    x[lat[:, 0] == nx * L] = W
    y[lat[:, 1] == ny * L] = H
    z = surface_height(cfg, layout, x, y, seed)
    v = np.column_stack([x, y, z])
    f = np.asarray(tris, dtype=np.int64)
    return Mesh(vertices=v, faces=f, uvs=terrain_uv(x, y, seed), normals=_vertex_normals(v, f))


UV_WARP = 0.025      # m per warp wave (3 waves per axis): breaks the 1 m texture-repeat grid


def terrain_uv(x, y, seed):
    """Terrain texture coordinates: (x, y) / UV_TILE plus a gentle low-frequency domain warp.

    The warp (3 waves per axis, 1.6-3.4 m wavelength, <= 7.5 cm displacement, <= ~25 % local
    stretch) shifts each 1 m repeat of the sand texture by a different amount, so the tiling
    grid does not line up into visible rows from robot height. Smooth and fold-free.
    """
    rng = np.random.default_rng([_seed(seed), STREAM_UV])
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    out = []
    for base in (x, y):
        w = base.copy()
        for _ in range(3):
            ang = rng.uniform(0, 2 * np.pi)
            k = 2 * np.pi / rng.uniform(1.6, 3.4)
            w = w + UV_WARP * np.sin(k * (math.cos(ang) * x + math.sin(ang) * y)
                                     + rng.uniform(0, 2 * np.pi))
        out.append(w / UV_TILE)
    return np.column_stack(out)


# ---------------------------------------------------------------------------------------------
# Rocks
# ---------------------------------------------------------------------------------------------

def _convex_hull(p, eps=1e-9):
    """Triangles (CCW seen from outside) of the convex hull of points p (N, 3), N >= 4.

    Incremental hull; points inside the hull are left unreferenced. Assumes the points are not
    all coplanar (the rock generator jitters them).
    """
    i0 = int(np.argmin(p[:, 0]))
    i1 = int(np.argmax(np.linalg.norm(p - p[i0], axis=1)))
    i2 = int(np.argmax(np.linalg.norm(np.cross(p - p[i0], p[i1] - p[i0]), axis=1)))
    i3 = int(np.argmax(np.abs((p - p[i0]) @ np.cross(p[i1] - p[i0], p[i2] - p[i0]))))
    inside = p[[i0, i1, i2, i3]].mean(axis=0)
    faces = []
    for a, b, c in ((i0, i1, i2), (i0, i2, i3), (i0, i3, i1), (i1, i3, i2)):
        if np.dot(np.cross(p[b] - p[a], p[c] - p[a]), inside - p[a]) > 0:
            b, c = c, b
        faces.append((a, b, c))
    for i in range(len(p)):
        if i in (i0, i1, i2, i3):
            continue
        f = np.asarray(faces)
        a = p[f[:, 0]]
        nrm = np.cross(p[f[:, 1]] - a, p[f[:, 2]] - a)
        vis = np.einsum('ij,ij->i', p[i] - a, nrm) > eps * np.linalg.norm(nrm, axis=1)
        if not vis.any():
            continue
        edges = {(u, w) for t in f[vis] for u, w in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0]))}
        faces = [tuple(t) for t in f[~vis]]
        faces += [(u, w, i) for u, w in edges if (w, u) not in edges]
    return np.asarray(faces, dtype=np.int64)


def _subdivide(v, f):
    """One 1-to-4 midpoint split (shared edges get one shared midpoint, so it stays closed)."""
    v = list(v)
    mid = {}

    def m(a, b):
        key = (min(a, b), max(a, b))
        if key not in mid:
            v.append(0.5 * (v[a] + v[b]))
            mid[key] = len(v) - 1
        return mid[key]
    nf = []
    for a, b, c in f:
        ab, bc, ca = m(a, b), m(b, c), m(c, a)
        nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
    return np.asarray(v), np.asarray(nf, dtype=np.int64)


def _split_normals(v, f, crease_deg):
    """Auto-smooth: per-corner normals that average only faces within `crease_deg` of each other.

    Returns (vertices, faces, normals, source_index) where a vertex is duplicated once per distinct
    corner normal, so sharp rock edges shade as edges while gently curved areas stay smooth.
    Positions are unchanged: the surface is still closed (weld by position to check).
    """
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])     # area-weighted
    fu = fn / np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-20)
    cos_t = math.cos(math.radians(crease_deg))
    inc = [[] for _ in range(len(v))]
    for j, t in enumerate(f):
        for k in t:
            inc[k].append(j)
    keys, out_n, src = {}, [], []
    nf = np.empty_like(f)
    for j, t in enumerate(f):
        for c, k in enumerate(t):
            g = np.asarray(inc[k])
            g = g[fu[g] @ fu[j] > cos_t]
            n = fn[g].sum(axis=0)
            n /= max(np.linalg.norm(n), 1e-20)
            key = (int(k), *np.round(n, 3))
            idx = keys.get(key)
            if idx is None:
                idx = keys[key] = len(src)
                src.append(int(k))
                out_n.append(n)
            nf[j, c] = idx
    src = np.asarray(src, dtype=np.int64)
    return v[src], nf, np.asarray(out_n), src


ROCK_UV_SCALE = 0.3    # metres of rock surface per rock-texture repeat
ROCK_CREASE_DEG = 36.0
ROCK_ROUND = 0.25      # Laplacian smoothing weight (0 = sharp hull, 1 = blob)


def rock_mesh(diameter, height, seed, sink=0.04, index=0):
    """Irregular, angular closed boulder: origin at base centre, top at z = height.

    Footprint: every vertex lies within D/2 of the origin and the farthest is exactly D/2.
    Shape stream is [seed, 30, index] (index = rock number within the world).
    The underside extends `sink` metres below
    z = 0 so a rock placed a few cm into the sand shows no gap. Shape: convex hull of 18-33
    jittered points on a random ellipsoid (elongation, flat base, blunt top, taper, sometimes a
    fracture face), split once 1-to-4, one light Laplacian pass (weathered edges), then
    low-frequency radial noise and 1-3 shallow dents so the facets are not CAD-flat. Edges sharper
    than ROCK_CREASE_DEG keep split normals (auto-smooth): it shades as angular stone, not a
    blob. Triangle count is bounded by the hull (<= 33 points -> <= 62 faces) times the 1-to-4
    split, i.e. <= 248 (measured in tests); UVs are a planar projection (seamless texture,
    ROCK_UV_SCALE m/tile).
    """
    rng = np.random.default_rng([_seed(seed), STREAM_ROCK, int(index)])
    n = int(rng.integers(18, 34))
    d = rng.normal(size=(n, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    axes = np.array([1.0, rng.uniform(0.55, 1.0), rng.uniform(0.55, 0.95)])
    p = d * rng.uniform(0.80, 1.0, (n, 1)) * axes
    # Flat base (bedding plane / sunk underside) and, for some rocks, a flat-ish top.
    zb = -rng.uniform(0.25, 0.6) * axes[2]
    p[:, 2] = np.maximum(p[:, 2], zb + rng.uniform(0.0, 0.03, n) * axes[2])
    # Blunt top: several points share a top level, so the summit is a facet, never a spike.
    zt = np.quantile(p[:, 2], rng.uniform(0.6, 0.9))
    p[:, 2] = np.minimum(p[:, 2], zt - rng.uniform(0.0, 0.06, n) * axes[2])
    # A tilted cut for some (a fracture face).
    if rng.uniform() < 0.5:
        w = rng.normal(size=3)
        w[2] = abs(w[2]) * 0.6
        w /= np.linalg.norm(w)
        proj = p @ w
        cut = np.quantile(proj, rng.uniform(0.7, 0.85))
        p -= np.maximum(proj - cut, 0.0)[:, None] * w[None, :]
    # Taper: boulders are wider at the base than at the top (no cylinder-like sides).
    zn = (p[:, 2] - p[:, 2].min()) / (np.ptp(p[:, 2]) + 1e-12)
    p[:, :2] *= (1.0 - rng.uniform(0.1, 0.3) * zn)[:, None]
    p += rng.normal(scale=0.004, size=p.shape)       # break exact coplanarity
    f = _convex_hull(p)
    used = np.unique(f)
    remap = np.full(len(p), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    v, f = _subdivide(p[used], remap[f])
    # One Laplacian pass: rounds the hull's corners and edges a little (weathered, not cut glass).
    nb = np.zeros_like(v)
    cnt = np.zeros(len(v))
    for a, b in ((0, 1), (1, 2), (2, 0)):
        np.add.at(nb, f[:, a], v[f[:, b]])
        np.add.at(nb, f[:, b], v[f[:, a]])
        np.add.at(cnt, f[:, a], 1)
        np.add.at(cnt, f[:, b], 1)
    v = v + ROCK_ROUND * (nb / cnt[:, None] - v)
    # Low-frequency radial noise + shallow dents (weathering; makes it slightly non-convex).
    c = v.mean(axis=0)
    rel = v - c
    r = np.linalg.norm(rel, axis=1)
    u = rel / r[:, None]
    scale = np.ones(len(v))
    for _ in range(4):
        w = rng.normal(size=3)
        w /= np.linalg.norm(w)
        scale += rng.uniform(0.015, 0.03) * np.sin(rng.uniform(2.0, 5.0) * (u @ w)
                                                   + rng.uniform(0, 2 * np.pi))
    for _ in range(int(rng.integers(1, 4))):
        w = rng.normal(size=3)
        w /= np.linalg.norm(w)
        scale -= rng.uniform(0.02, 0.05) * np.exp(-(1.0 - u @ w) / rng.uniform(0.02, 0.06))
    v = c + rel * scale[:, None]
    # Vertical: the lowest `gf` of the shape is underground (compressed into [-sink, 0]).
    zmin, zmax = v[:, 2].min(), v[:, 2].max()
    zg = zmin + rng.uniform(0.15, 0.3) * (zmax - zmin)
    z = np.where(v[:, 2] >= zg, (v[:, 2] - zg) / (zmax - zg) * height,
                 (v[:, 2] - zg) / (zg - zmin) * sink)
    xy = v[:, :2] - 0.5 * (v[:, :2].max(axis=0) + v[:, :2].min(axis=0))
    xy *= 0.5 * diameter / np.hypot(xy[:, 0], xy[:, 1]).max()
    v = np.column_stack([xy, z])
    v, f, nrm, _src = _split_normals(v, f, ROCK_CREASE_DEG)
    # Planar projection along a tilted axis: continuous across split vertices (no seams).
    uv = np.column_stack([(v[:, 0] + 0.6 * v[:, 1]) / ROCK_UV_SCALE,
                          (v[:, 2] + 0.4 * v[:, 1] - 0.3 * v[:, 0]) / ROCK_UV_SCALE])
    return Mesh(vertices=v, faces=f, uvs=uv, normals=nrm)


# ---------------------------------------------------------------------------------------------
# OBJ
# ---------------------------------------------------------------------------------------------

def write_obj(path, mesh, mtl_name=None):
    """Write a Wavefront OBJ (v/vt/vn).

    With `mtl_name`, also writes a sibling <stem>.mtl whose map_Kd is ../materials/<mtl_name>.png.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    v = np.asarray(mesh.vertices, dtype=float)
    f = np.asarray(mesh.faces, dtype=np.int64) + 1
    uv = mesh.uvs if mesh.uvs is not None else np.zeros((len(v), 2))
    n = mesh.normals if mesh.normals is not None else _vertex_normals(v, f - 1)
    lines = ['# cac_sim generated mesh: %d vertices, %d faces' % (len(v), len(f))]
    if mtl_name:
        lines += ['mtllib %s.mtl' % path.stem]
    lines += ['v %.4f %.4f %.4f' % tuple(p) for p in v]
    lines += ['vt %.4f %.4f' % tuple(p) for p in uv]
    lines += ['vn %.4f %.4f %.4f' % tuple(p) for p in n]
    if mtl_name:
        lines += ['usemtl %s' % mtl_name]
    lines += ['f %d/%d/%d %d/%d/%d %d/%d/%d' % (a, a, a, b, b, b, c, c, c) for a, b, c in f]
    path.write_text('\n'.join(lines) + '\n')
    if mtl_name:
        path.with_suffix('.mtl').write_text(
            'newmtl %s\nKa 1.0 1.0 1.0\nKd 1.0 1.0 1.0\nKs 0.0 0.0 0.0\nNs 1.0\nd 1.0\nillum 1\n'
            'map_Kd ../materials/%s.png\n' % (mtl_name, mtl_name))
    return path


# ---------------------------------------------------------------------------------------------
# Textures (seamless: noise is filtered in the Fourier domain, which is periodic by construction)
# ---------------------------------------------------------------------------------------------

def write_png(path, img):
    """Write an (H, W, 3) uint8 or float [0, 1] array as an 8-bit RGB PNG (stdlib only)."""
    a = np.asarray(img)
    if a.dtype != np.uint8:
        a = (np.clip(a, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
    if a.ndim == 2:
        a = np.repeat(a[:, :, None], 3, axis=2)
    h, w = a.shape[:2]
    raw = np.concatenate([np.zeros((h, 1), dtype=np.uint8), a.reshape(h, w * 3)], axis=1)

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))
    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw.tobytes(), 6))
           + chunk(b'IEND', b''))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)
    return path


def _tile_noise(rng, size, wavelength_px, slope=0.0):
    """Seamless band-limited noise, zero mean, unit std. Energy around `wavelength_px`."""
    white = rng.normal(size=(size, size))
    fx = np.fft.fftfreq(size)
    fr = np.hypot(fx[:, None], fx[None, :])
    f0 = 1.0 / wavelength_px
    filt = np.exp(-0.5 * (np.log((fr + 1e-9) / f0) / 0.6) ** 2) * (fr + 1e-9) ** (-slope)
    filt[0, 0] = 0.0
    out = np.real(np.fft.ifft2(np.fft.fft2(white) * filt))
    return out / (out.std() + 1e-12)


def _smooth_mask(x, sharp=1.5):
    return 0.5 + 0.5 * np.tanh(sharp * x)


def _normalise_mean(img, albedo):
    """Scale each channel so its mean (after clipping) is `albedo`."""
    img = np.clip(img, 0.0, 1.0)
    for _ in range(3):
        img = np.clip(img * (np.asarray(albedo) / img.reshape(-1, 3).mean(axis=0)), 0.0, 1.0)
    return img


def sand_texture(path, seed, size=1024, albedo=(0.75, 0.68, 0.55)):
    """Seamless dry fine-sand albedo (1 tile = UV_TILE metres), channel means == `albedo`.

    Fine grain (1-3 mm) with per-grain colour scatter, sparse coarse grains, 2-5 cm clumping and
    only weak, multi-scale large mottling: a strong single-scale blotch is what makes a 1 m tile
    visibly repeat from robot height, so the low frequencies are kept faint and broadband.
    """
    rng = np.random.default_rng([_seed(seed), STREAM_SAND_TEX])
    base = np.asarray(albedo, dtype=float)
    mottle = (_tile_noise(rng, size, size / 2.0) + 0.7 * _tile_noise(rng, size, size / 5.0)
              + 0.5 * _tile_noise(rng, size, size / 11.0)) / 1.35
    clump = _tile_noise(rng, size, size / 30.0)               # ~3 cm
    grain = _tile_noise(rng, size, 1.8)                        # ~1-2 mm
    grain2 = _tile_noise(rng, size, 3.5)                       # ~3 mm
    lum = 1.0 + 0.025 * mottle + 0.03 * clump + 0.07 * grain + 0.04 * grain2
    # Sparse coarse grains / tiny clods (2-4 px blobs), darker or lighter.
    blob = _tile_noise(rng, size, 3.0)
    sel = rng.random((size, size))
    lum = lum - 0.10 * ((blob > 2.3) & (sel < 0.5)) + 0.08 * ((blob > 2.3) & (sel >= 0.5))
    speck = rng.random((size, size))
    lum = lum - 0.22 * (speck > 0.997) + 0.12 * (speck < 0.003)
    img = base[None, None, :] * lum[:, :, None]
    # Per-grain mineral colour scatter (quartz/feldspar/dark grains) + a faint warm/cool drift.
    tint = 0.025 * _tile_noise(rng, size, 1.8) + 0.008 * _tile_noise(rng, size, size / 4.0)
    img[:, :, 0] *= 1.0 + tint
    img[:, :, 2] *= 1.0 - 1.5 * tint
    return write_png(path, _normalise_mean(img, base))


def _normal_png(path, h, rms_slope):
    """Tangent-space normal map (OpenGL convention: +x = +u, +y = +v) from a periodic height."""
    size = h.shape[0]
    k = 2j * np.pi * np.fft.fftfreq(size)
    H = np.fft.fft2(h)
    d_col = np.real(np.fft.ifft2(H * k[None, :]))      # dh / d(column) = dh / du
    d_row = np.real(np.fft.ifft2(H * k[:, None]))      # dh / d(row) = -dh / dv (row 0 is v = 1)
    s = rms_slope / (np.sqrt(np.mean(d_col ** 2 + d_row ** 2)) + 1e-12)
    n = np.stack([-s * d_col, s * d_row, np.ones_like(h)], axis=-1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    return write_png(path, 0.5 * n + 0.5)


def sand_normal_map(path, seed, size=1024, rms_slope=0.22):
    """Seamless tangent-space normal map of fine sand relief (grains + small clumps).

    Same tiling as sand_texture (1 tile = UV_TILE m). For a PBR <normal_map> on the terrain.
    """
    rng = np.random.default_rng([_seed(seed), STREAM_SAND_NRM])
    h = (_tile_noise(rng, size, 2.0) + 0.8 * _tile_noise(rng, size, 5.0)
         + 6.0 * _tile_noise(rng, size, size / 40.0) + 12.0 * _tile_noise(rng, size, size / 12.0))
    return _normal_png(path, h, rms_slope)


# (main colour, secondary/stain colour, bedding strata?) - tan / buff / light-grey stone.
ROCK_PALETTES = (
    ((0.74, 0.65, 0.52), (0.64, 0.58, 0.50), True),     # warm tan sandstone, greyer patches
    ((0.79, 0.72, 0.61), (0.74, 0.63, 0.52), False),    # light buff, pinkish-tan stains
    ((0.70, 0.68, 0.63), (0.73, 0.66, 0.55), False),    # light-grey limestone, tan stains
    ((0.76, 0.67, 0.55), (0.67, 0.61, 0.54), True),     # mid tan, banded
)


def _rock_albedo(rng, size, palette):
    main, stain, strata = palette
    jit = 1.0 + rng.uniform(-0.04, 0.04, 3)
    main = np.asarray(main) * jit
    stain = np.asarray(stain) * jit
    mix = 0.75 * _smooth_mask(_tile_noise(rng, size, size / 3.0)
                              + 0.4 * _tile_noise(rng, size, size / 10.0), 1.0)
    img = main[None, None, :] * (1 - mix[:, :, None]) + stain[None, None, :] * mix[:, :, None]
    lum = (1.0 + 0.05 * _tile_noise(rng, size, size / 6.0)
           + 0.05 * _tile_noise(rng, size, size / 24.0) + 0.09 * _tile_noise(rng, size, 2.0)
           + 0.05 * _tile_noise(rng, size, 5.0))
    if strata:
        # Bedding bands: an integer number of (warped) periods per tile, so still seamless.
        rows = np.arange(size)[:, None] / size
        warp = 0.06 * _tile_noise(rng, size, size / 2.0) + 0.01 * _tile_noise(rng, size, size / 12)
        per = int(rng.integers(5, 10))
        lum += 0.045 * np.sin(2 * np.pi * per * (rows + warp)) + 0.02 * np.sin(
            2 * np.pi * 3 * per * (rows + warp) + 1.0)
    # Hairline cracks / crevices (ridged noise -> thin dark lines) and small pits.
    ridge = 1.0 - np.abs(_tile_noise(rng, size, size / 6.0)) / 3.0
    lum -= 0.22 * np.clip(ridge, 0, 1) ** 40
    pits = _tile_noise(rng, size, 4.0)
    lum -= 0.18 * np.clip(pits - 2.0, 0, 1.0)
    speck = rng.random((size, size))
    lum = lum - 0.35 * (speck > 0.996) + 0.18 * (speck < 0.008)
    return np.clip(img * lum[:, :, None], 0.0, 1.0)


def rock_texture_variants(out_dir, seed, n=3, size=512):
    """Write rock_0.png .. rock_{n-1}.png (different stone palettes and grain) into out_dir.

    Returns the file names (not paths), e.g. ['rock_0.png', 'rock_1.png', 'rock_2.png'].
    """
    out_dir = Path(out_dir)
    names = []
    for k in range(int(n)):
        rng = np.random.default_rng([_seed(seed), STREAM_ROCK_TEX + k])
        name = 'rock_%d.png' % k
        write_png(out_dir / name, _rock_albedo(rng, size, ROCK_PALETTES[k % len(ROCK_PALETTES)]))
        names.append(name)
    return names


def rock_normal_map(path, seed, size=512, rms_slope=0.35):
    """Seamless tangent-space normal map for rocks (pitted, weathered stone surface)."""
    rng = np.random.default_rng([_seed(seed), STREAM_ROCK_NRM])
    h = (_tile_noise(rng, size, 2.5) + 1.5 * _tile_noise(rng, size, 6.0)
         + 6.0 * _tile_noise(rng, size, size / 20.0) + 10.0 * _tile_noise(rng, size, size / 6.0))
    return _normal_png(path, h, rms_slope)
