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

# RNG stream ids owned by this module.
STREAM_TILL = 1      # tilled-sand relief
STREAM_RIM = 2       # crater rim height ratios (per crater index)
STREAM_ROCK = 3      # rock shape
STREAM_SAND_TEX = 4
STREAM_ROCK_TEX = 5
STREAM_DEPTH = 6     # crater depth ratios (per crater index)

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


def effective_crater_depth(diameter, depth, rim=0.0, max_slope_deg=DEFAULT_MAX_SLOPE_DEG,
                           sand_depth=None):
    """Floor depth below undisturbed sand after the slope cap (and the sand-depth cap, if given).

    Equals `depth` unless 1.5 * (depth + rim) / (1.05 * D/2) > tan(max_slope_deg), in which case
    depth and rim are scaled down together (see _shape).
    """
    d = _shape(diameter, depth, rim, max_slope_deg)[3]
    return min(d, float(sand_depth)) if sand_depth is not None else d


def crater_outer_radius(diameter, rim=None, max_slope_deg=None):
    """Radius beyond which the terrain is exactly the undisturbed tilled surface: 1.40 * D/2.

    The rim crest is at 1.05 * D/2 and its outer flank reaches z = 0 at 1.40 * D/2; the tilled
    relief blends back in over D/2 .. 1.40 * D/2. Independent of rim height and slope (the
    arguments are accepted for API compatibility).
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
    wall_deg (= max_slope_deg_eff, kept for older callers), outer_radius).
    Requested depth: layout 'depth' (m) if given, else layout 'depth_ratio' * D, else a ratio drawn
    uniformly in craters.depth_ratio from stream [seed, 6, i]. Rim: layout 'rim' (m) if given,
    else craters.rim_ratio * D drawn from stream [seed, 2, i]. `rim` in the result is the rim
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
        'rim': h, 'max_slope_deg_eff': steep, 'wall_deg': steep,
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
    return Mesh(vertices=v, faces=f, uvs=np.column_stack([x, y]) / UV_TILE,
                normals=_vertex_normals(v, f))


# ---------------------------------------------------------------------------------------------
# Rocks
# ---------------------------------------------------------------------------------------------

def _icosphere(subdiv):
    t = (1.0 + 5 ** 0.5) / 2.0
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t),
         (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4),
         (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8),
         (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    v = [np.array(p, dtype=float) / np.linalg.norm(p) for p in v]
    for _ in range(subdiv):
        mid = {}

        def m(a, b):
            key = (min(a, b), max(a, b))
            if key not in mid:
                p = v[a] + v[b]
                v.append(p / np.linalg.norm(p))
                mid[key] = len(v) - 1
            return mid[key]
        nf = []
        for a, b, c in f:
            ab, bc, ca = m(a, b), m(b, c), m(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = nf
    return np.array(v), np.array(f, dtype=np.int64)


def rock_mesh(diameter, height, seed, sink=0.04):
    """Irregular closed boulder: origin at base centre, top at z = height, footprint ~diameter.

    The underside extends `sink` metres below z = 0 so a rock placed a few cm into the sand shows
    no gap. 320 triangles.
    """
    rng = np.random.default_rng([_seed(seed), STREAM_ROCK])
    d, f = _icosphere(2)
    # Low-frequency radial noise (sum of random plane waves on the sphere) + a few flat facets.
    r = np.ones(len(d))
    for _ in range(6):
        w = rng.normal(size=3)
        w /= np.linalg.norm(w)
        r += rng.uniform(0.04, 0.09) * np.sin(rng.uniform(1.5, 3.5) * d @ w + rng.uniform(0, 6.3))
    for _ in range(3):
        nrm = rng.normal(size=3)
        nrm[2] = abs(nrm[2]) * 0.5
        nrm /= np.linalg.norm(nrm)
        cut = rng.uniform(0.80, 0.92)
        proj = d @ nrm * r
        over = proj > cut
        r[over] *= cut / proj[over]
    p = d * r[:, None]
    # Widest girth about a third of the way up; the bottom is flattened.
    pz = np.maximum(p[:, 2], -0.75)
    eq = 0.35 * height
    z = np.where(pz >= 0, eq + pz * (height - eq), eq + pz / 0.75 * (eq + sink))
    aspect = rng.uniform(0.82, 1.0)
    rot = rng.uniform(0, 2 * np.pi)
    x, y = p[:, 0], p[:, 1] * aspect
    x, y = x * math.cos(rot) - y * math.sin(rot), x * math.sin(rot) + y * math.cos(rot)
    xy = np.column_stack([x, y])
    xy -= 0.5 * (xy.max(axis=0) + xy.min(axis=0))
    xy *= diameter / (xy.max(axis=0) - xy.min(axis=0)).max()
    z = (z + sink) * (height + sink) / (z.max() + sink) - sink
    v = np.column_stack([xy, z])
    # Box-ish projection: works with the seamless rock texture, no spherical seam.
    uv = np.column_stack([(v[:, 0] + 0.6 * v[:, 1]) / 0.3, (v[:, 2] + 0.4 * v[:, 1]) / 0.3])
    return Mesh(vertices=v, faces=f, uvs=uv, normals=_vertex_normals(v, f))


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


def sand_texture(path, seed, size=1024, albedo=(0.76, 0.69, 0.52)):
    """Seamless dry-sand albedo (1 tile = UV_TILE metres): grains + soft mottling."""
    rng = np.random.default_rng([_seed(seed), STREAM_SAND_TEX])
    base = np.asarray(albedo, dtype=float)
    mottle = _tile_noise(rng, size, size / 3.0)            # ~30 cm patches
    mid = _tile_noise(rng, size, size / 25.0)              # ~4 cm
    grain = _tile_noise(rng, size, 2.0)                    # 1-2 mm grains
    lum = 1.0 + 0.05 * mottle + 0.04 * mid + 0.10 * grain
    # Occasional dark/light grains (mineral specks).
    speck = rng.random((size, size))
    lum = lum - 0.25 * (speck > 0.996) + 0.15 * (speck < 0.004)
    hue = 0.03 * _tile_noise(rng, size, size / 6.0)        # warm/cool drift
    img = base[None, None, :] * lum[:, :, None]
    img[:, :, 0] *= 1.0 + hue
    img[:, :, 2] *= 1.0 - hue
    return write_png(path, img)


def rock_texture(path, seed, size=512):
    """Seamless grey-brown speckled rock albedo."""
    rng = np.random.default_rng([_seed(seed), STREAM_ROCK_TEX])
    base = np.array([0.46, 0.42, 0.37])
    blotch = _tile_noise(rng, size, size / 4.0)
    mid = _tile_noise(rng, size, size / 20.0)
    fine = _tile_noise(rng, size, 2.5)
    lum = 1.0 + 0.10 * blotch + 0.08 * mid + 0.12 * fine
    speck = rng.random((size, size))
    lum = lum - 0.35 * (speck > 0.985) + 0.30 * (speck < 0.01)
    tint = 0.06 * _tile_noise(rng, size, size / 5.0)
    img = base[None, None, :] * lum[:, :, None]
    img[:, :, 0] *= 1.0 + tint
    img[:, :, 2] *= 1.0 - tint
    return write_png(path, img)
