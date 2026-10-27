"""Geometry module tests (no ROS needed).

Run: PYTHONPATH= python3 -m pytest test/test_geometry.py -q
"""
import math
from pathlib import Path
import struct
import sys
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cac_sim_gen import geometry as g  # noqa: E402, I100
from cac_sim_gen.config import load_config, load_layout  # noqa: E402
import numpy as np  # noqa: E402, I100, I201
import pytest  # noqa: E402, I201


@pytest.fixture(scope='module')
def cfg():
    return load_config()


@pytest.fixture(scope='module')
def schematic():
    return load_layout('schematic')


@pytest.fixture(scope='module')
def worst(cfg):
    """Max crater count, max size, spread over the arena (incl. one touching a wall)."""
    pos = [(1.0, 3.0), (2.5, 4.4), (3.0, 1.0), (4.0, 3.2), (3.4, 2.2), (6.0, 1.0), (8.6, 4.6)]
    return {'name': 'worst', 'seed': 3, 'boulders': [], 'start_pose': {},
            'craters': [{'position': list(p), 'diameter': 0.5, 'depth_ratio': 0.25} for p in pos]}


def _edge_counts(faces):
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    e = np.sort(e, axis=1)
    uniq, cnt = np.unique(e, axis=0, return_counts=True)
    return uniq, cnt


def _check_surface(m, cfg):
    W, H = cfg['arena']['size']
    v, f = m.vertices, m.faces
    assert np.isfinite(v).all() and np.isfinite(m.normals).all() and np.isfinite(m.uvs).all()
    assert v[:, 0].min() == 0.0 and v[:, 0].max() == W
    assert v[:, 1].min() == 0.0 and v[:, 1].max() == H
    # No degenerate faces, no duplicate faces, no duplicate vertices.
    assert (f[:, 0] != f[:, 1]).all() and (f[:, 1] != f[:, 2]).all() and (f[:, 0] != f[:, 2]).all()
    assert len(np.unique(np.sort(f, axis=1), axis=0)) == len(f)
    assert len(np.unique(np.round(v[:, :2], 7), axis=0)) == len(v)
    # CCW from above, and the projected triangles tile the arena exactly (no overlap, no gap).
    a = v[f[:, 1], :2] - v[f[:, 0], :2]
    b = v[f[:, 2], :2] - v[f[:, 0], :2]
    area = 0.5 * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])
    assert (area > 1e-9).all()
    assert abs(area.sum() - W * H) < 1e-6
    assert (m.normals[:, 2] > 0).all()
    # Crack-free: interior edges shared by exactly 2 faces; edges used once lie on the border.
    edges, cnt = _edge_counts(f)
    assert cnt.max() == 2
    border = edges[cnt == 1]
    p, q = v[border[:, 0], :2], v[border[:, 1], :2]
    on = ((np.isclose(p[:, 0], 0) & np.isclose(q[:, 0], 0))
          | (np.isclose(p[:, 0], W) & np.isclose(q[:, 0], W))
          | (np.isclose(p[:, 1], 0) & np.isclose(q[:, 1], 0))
          | (np.isclose(p[:, 1], H) & np.isclose(q[:, 1], H)))
    assert on.all()


def test_terrain_schematic_budget_and_watertight(cfg, schematic):
    vis = g.build_terrain(cfg, schematic, 0)
    col = g.build_terrain(cfg, schematic, 0, res=0.10, fine_res=0.04)
    assert len(vis.faces) <= 80000
    assert len(col.faces) <= 20000
    _check_surface(vis, cfg)
    _check_surface(col, cfg)


def test_terrain_worst_case_budget(cfg, worst):
    vis = g.build_terrain(cfg, worst, 5)
    col = g.build_terrain(cfg, worst, 5, res=0.10, fine_res=0.04)
    assert len(vis.faces) <= 80000
    assert len(col.faces) <= 20000
    _check_surface(vis, cfg)
    _check_surface(col, cfg)


def test_terrain_refined_at_craters(cfg, schematic):
    m = g.build_terrain(cfg, schematic, 0)
    cx, cy = schematic['craters'][0]['position']
    v = m.vertices
    near = np.hypot(v[:, 0] - cx, v[:, 1] - cy) < 0.25
    # 0.25 m radius disc at <= 15 mm spacing: ~pi * 0.25^2 / 0.015^2 = 870 vertices.
    assert near.sum() > 600
    # Mesh vertices carry the analytic height.
    assert np.allclose(v[:, 2], g.surface_height(cfg, schematic, v[:, 0], v[:, 1], 0))


def _params(cfg, layout, seed):
    return [g.crater_params(cfg, layout, i, seed) for i in range(len(layout['craters']))]


def test_crater_depth_and_flat_sand(cfg, schematic):
    rough = cfg['sand']['tilled_roughness']
    lo, hi = cfg['craters']['depth_ratio']
    for c, p in zip(schematic['craters'], _params(cfg, schematic, 0)):
        z = g.surface_height(cfg, schematic, c['position'][0], c['position'][1], 0)
        assert abs(z + p['depth_eff']) <= 0.002
        assert lo * c['diameter'] - 1e-9 <= p['depth_eff'] <= hi * c['diameter'] + 1e-9
        assert p['outer_radius'] == pytest.approx(1.4 * c['diameter'] / 2)
        assert p['max_slope_deg_eff'] <= cfg['craters']['max_slope_deg'] + 1e-9
    xs, ys = np.meshgrid(np.linspace(0, 9, 400), np.linspace(0, 5, 250))
    z = g.surface_height(cfg, schematic, xs, ys, 0)
    assert np.isfinite(z).all()
    assert z.min() >= -cfg['arena']['sand_depth'] - 1e-12
    far = np.ones_like(xs, dtype=bool)
    for c in schematic['craters']:
        far &= np.hypot(xs - c['position'][0], ys - c['position'][1]) > c['diameter']
    assert np.abs(z[far]).max() <= 2 * rough
    assert z[far].std() > 0.2 * rough          # the relief is actually there


@pytest.mark.parametrize('d', [0.30, 0.40, 0.50])
@pytest.mark.parametrize('ratio,rim_ratio', [(0.10, 0.04), (0.10, 0.08), (0.25, 0.04),
                                             (0.25, 0.08), (0.45, 0.0)])
def test_crater_profile(d, ratio, rim_ratio):
    cap = 38.0
    depth, rim = ratio * d, rim_ratio * d
    eff = g.effective_crater_depth(d, depth, rim, cap)
    assert eff <= depth + 1e-12
    if ratio <= 0.10:
        assert eff == pytest.approx(depth)        # shallow bowls are never clamped
    prof = g.crater_profile(d, depth, rim, cap)
    outer = g.crater_outer_radius(d, rim, cap)
    h = 1e-4
    r = np.arange(0.0, outer + 0.1, h)
    z = prof(r)
    assert z[0] == pytest.approx(-eff, abs=1e-9)
    crest = r[np.argmax(z)]
    assert d / 2 < crest < 1.1 * d / 2              # rim crest just outside R
    assert np.all(np.abs(z[r >= outer]) < 1e-12)
    assert 1.3 * d / 2 <= outer <= 1.5 * d / 2
    slope = np.diff(z) / h
    assert np.degrees(np.arctan(np.abs(slope).max())) <= cap + 1.0
    # Bowl, not a cone: deepest at the centre with zero slope there, monotonic up to the crest.
    assert abs(slope[0]) < 0.01
    assert np.all(np.diff(z[r <= crest]) >= -1e-12)
    # No crease anywhere: slope changes continuously (second difference bounded by curvature).
    curv = np.abs(np.diff(slope)) / h
    assert curv.max() < 12.0 * (eff + rim + 1e-3) / (d / 2) ** 2 * 4
    assert np.abs(np.diff(slope)).max() < 0.01
    # The slope cap alone limits depth/diameter to tan(cap) * 1.05 / 3 (0.27 at 38 deg).
    assert eff <= math.tan(math.radians(cap)) * 1.05 / 3 * d + 1e-9


def test_crater_surface_slope_and_extent(cfg):
    """Max slope of the real surface <= max_slope_deg + 1; nothing disturbed beyond outer."""
    cap = cfg['craters']['max_slope_deg']
    craters = [{'position': [2.0, 2.0], 'diameter': 0.5, 'depth_ratio': 0.25, 'rim': 0.04},
               {'position': [4.0, 3.0], 'diameter': 0.3, 'depth': 0.2},
               {'position': [6.0, 1.5], 'diameter': 0.4}]
    lay = {'craters': craters}
    h = 0.002
    for rough in (0.0, cfg['sand']['tilled_roughness']):
        c2 = dict(cfg, sand=dict(cfg['sand'], tilled_roughness=rough))
        for c, p in zip(craters, _params(c2, lay, 7)):
            cx, cy = c['position']
            outer = p['outer_radius']
            xs, ys = np.meshgrid(np.arange(cx - outer - 0.1, cx + outer + 0.1, h),
                                 np.arange(cy - outer - 0.1, cy + outer + 0.1, h))
            z = g.surface_height(c2, lay, xs, ys, 7)
            gy, gx = np.gradient(z, h)
            slope = np.degrees(np.arctan(np.hypot(gx, gy)))
            rr = np.hypot(xs - cx, ys - cy)
            # With roughness on, check the dug part (inside D/2); with it off, everything.
            region = rr < outer if rough == 0.0 else rr < c['diameter'] / 2
            assert slope[region].max() <= cap + 1.0
            assert z[rr < 0.01].min() == pytest.approx(-p['depth_eff'], abs=0.002)
            flat = g.surface_height(c2, {'craters': []}, xs, ys, 7)
            disturbed = np.abs(z - flat) > 0.001
            assert disturbed.any()
            assert rr[disturbed].max() <= outer + 1e-9


def test_crater_overrides_and_draws(cfg):
    """Layout depth / depth_ratio / rim win; otherwise drawn per crater, stable when adding."""
    lo, hi = cfg['craters']['depth_ratio']
    rlo, rhi = cfg['craters']['rim_ratio']
    craters = [{'position': [0.6 + 0.8 * i, 2.0], 'diameter': 0.3 + 0.02 * i} for i in range(10)]
    craters[0]['depth'] = 0.05
    craters[1]['depth_ratio'] = 0.12
    craters[2]['rim'] = 0.0
    lay = {'craters': craters}
    ps = _params(cfg, lay, 11)
    assert ps[0]['depth'] == 0.05 and ps[0]['depth_eff'] == pytest.approx(0.05)
    assert ps[1]['depth'] == pytest.approx(0.12 * 0.32)
    assert ps[2]['rim'] == 0.0
    ratios = [p['depth'] / p['diameter'] for p in ps[2:]]
    assert all(lo <= q <= hi for q in ratios) and max(ratios) - min(ratios) > 0.03
    assert all(rlo * p['diameter'] * 0.7 <= p['rim'] <= rhi * p['diameter'] + 1e-12
               for p in ps[3:])
    assert _params(cfg, lay, 11) == ps
    more = {'craters': craters + [dict(craters[5], position=[9.0, 4.0])]}
    assert _params(cfg, more, 11)[:10] == ps


def test_fine_band_covers_disturbed_region(cfg, schematic):
    m = g.build_terrain(cfg, schematic, 0)
    v = m.vertices
    for c, p in zip(schematic['craters'], _params(cfg, schematic, 0)):
        outer = p['outer_radius']
        ring = np.abs(np.hypot(v[:, 0] - c['position'][0], v[:, 1] - c['position'][1])
                      - (outer - 0.01)) < 0.01
        # 2 cm wide ring just inside the outer radius: fine spacing gives >> coarse count.
        expected_fine = 2 * np.pi * outer * 0.02 / 0.015 ** 2
        assert ring.sum() > 0.5 * expected_fine


def test_sand_depth_clamp(cfg):
    cr = dict(cfg['craters'], max_slope_deg=89.0)
    lay = {'craters': [{'position': [2, 2], 'diameter': 0.5, 'depth': 0.6}]}
    c2 = dict(cfg, craters=cr)
    z = g.surface_height(c2, lay, 2.0, 2.0, 1)
    assert z == pytest.approx(-cfg['arena']['sand_depth'])
    assert g.crater_params(c2, lay, 0, 1)['depth_eff'] == cfg['arena']['sand_depth']


def test_determinism(cfg, schematic):
    a = g.build_terrain(cfg, schematic, 42)
    b = g.build_terrain(cfg, schematic, 42)
    c = g.build_terrain(cfg, schematic, 43)
    assert np.array_equal(a.vertices, b.vertices) and np.array_equal(a.faces, b.faces)
    assert not np.array_equal(a.vertices[:, 2], c.vertices[:, 2])
    r1, r2 = g.rock_mesh(0.35, 0.2, 9), g.rock_mesh(0.35, 0.2, 9)
    assert np.array_equal(r1.vertices, r2.vertices)
    assert not np.array_equal(r1.vertices, g.rock_mesh(0.35, 0.2, 10).vertices)
    # seed None behaves like seed 0
    assert np.array_equal(g.build_terrain(cfg, schematic, None).vertices,
                          g.build_terrain(cfg, schematic, 0).vertices)


@pytest.mark.parametrize('d,h,seed', [(0.30, 0.12, 0), (0.33, 0.25, 1), (0.40, 0.35, 2),
                                      (0.35, 0.18, 3), (0.31, 0.30, 4)])
def test_rock(d, h, seed):
    m = g.rock_mesh(d, h, seed)
    v, f = m.vertices, m.faces
    assert len(f) <= 400 and np.isfinite(v).all()
    ext = v.max(axis=0) - v.min(axis=0)
    assert abs(max(ext[0], ext[1]) - d) <= 0.15 * d
    assert abs(v[:, 2].max() - h) <= 0.15 * h
    assert -0.06 <= v[:, 2].min() <= -0.02        # extends a few cm below the base
    assert np.allclose(0.5 * (v[:, :2].max(axis=0) + v[:, :2].min(axis=0)), 0, atol=1e-9)
    # Closed manifold: every edge shared by exactly 2 faces, consistent winding, outward.
    _, cnt = _edge_counts(f)
    assert (cnt == 2).all()
    directed = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    assert len(np.unique(directed, axis=0)) == len(directed)
    vol = np.einsum('ij,ij->i', v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum() / 6
    assert vol > 0
    assert m.uvs.shape == (len(v), 2) and m.normals.shape == v.shape


def test_write_obj(tmp_path, cfg, schematic):
    m = g.rock_mesh(0.33, 0.2, 0)
    p = g.write_obj(tmp_path / 'meshes' / 'rock_0.obj', m, mtl_name='rock')
    text = p.read_text().splitlines()
    assert 'mtllib rock_0.mtl' in text and 'usemtl rock' in text
    vs = [ln for ln in text if ln.startswith('v ')]
    fs = [ln for ln in text if ln.startswith('f ')]
    assert len(vs) == len(m.vertices) and len(fs) == len(m.faces)
    assert sum(ln.startswith('vt ') for ln in text) == len(m.vertices)
    assert sum(ln.startswith('vn ') for ln in text) == len(m.vertices)
    idx = np.array([[int(t.split('/')[0]) for t in ln.split()[1:]] for ln in fs])
    assert np.array_equal(idx - 1, m.faces)
    mtl = (tmp_path / 'meshes' / 'rock_0.mtl').read_text()
    assert 'newmtl rock' in mtl and 'map_Kd ../materials/rock.png' in mtl
    q = g.write_obj(tmp_path / 'meshes' / 'terrain_collision.obj',
                    g.build_terrain(cfg, schematic, 0, res=0.1, fine_res=0.04))
    assert not q.with_suffix('.mtl').exists()


def _decode_png(path):
    data = Path(path).read_bytes()
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    pos, idat, ihdr = 8, b'', None
    while pos < len(data):
        n, tag = struct.unpack('>I4s', data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + n]
        crc, = struct.unpack('>I', data[pos + 8 + n:pos + 12 + n])
        assert crc == zlib.crc32(tag + body) & 0xffffffff
        if tag == b'IHDR':
            ihdr = struct.unpack('>IIBBBBB', body)
        elif tag == b'IDAT':
            idat += body
        pos += 12 + n
    w, h, depth, ctype = ihdr[:4]
    assert depth == 8 and ctype == 2
    raw = np.frombuffer(zlib.decompress(idat), dtype=np.uint8).reshape(h, 1 + 3 * w)
    assert (raw[:, 0] == 0).all()
    return raw[:, 1:].reshape(h, w, 3)


def test_textures(tmp_path):
    s = _decode_png(g.sand_texture(tmp_path / 'sand.png', 0, size=256))
    r = _decode_png(g.rock_texture(tmp_path / 'rock.png', 0, size=128))
    assert s.shape == (256, 256, 3) and r.shape == (128, 128, 3)
    mean = s.reshape(-1, 3).mean(axis=0) / 255
    assert np.allclose(mean, [0.76, 0.69, 0.52], atol=0.04)
    assert s.std() > 3 and r.std() > 3
    rm = r.reshape(-1, 3).mean(axis=0) / 255
    assert 0.3 < rm.mean() < 0.55 and rm[0] >= rm[2]     # grey-brown
    # Seamless: the wrap-around seam is no rougher than an interior row/column step.
    for img in (s.astype(float), r.astype(float)):
        seam = np.abs(img[0] - img[-1]).mean()
        inner = np.abs(np.diff(img, axis=0)).mean()
        assert seam < 1.3 * inner
        seam = np.abs(img[:, 0] - img[:, -1]).mean()
        inner = np.abs(np.diff(img, axis=1)).mean()
        assert seam < 1.3 * inner
    a = g.sand_texture(tmp_path / 'a.png', 5, size=64).read_bytes()
    b = g.sand_texture(tmp_path / 'b.png', 5, size=64).read_bytes()
    assert a == b
