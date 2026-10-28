"""World writer: well-formed SDF, truth fenced off, legal markers, deterministic output."""
import filecmp
import hashlib
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from cac_sim_gen import config, geometry, layout as layout_mod, markers as markers_mod, world
import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
SCRIPT = PKG / 'scripts' / 'generate_arena'


def _uris(root):
    return [u.text.strip() for u in root.iter() if u.tag in ('uri', 'albedo_map') and u.text]


def _check(root_dir, layout, extra_models=()):
    tree = ET.parse(root_dir / 'arena.sdf')                  # well-formed
    models = tree.getroot().find('world').findall('model')
    names = [m.get('name') for m in models]
    assert len(names) == len(set(names)), 'duplicate model names'
    expected = {'terrain', 'subfloor', 'wall_south', 'wall_north', 'wall_west', 'wall_east',
                'column',
                'hall_south', 'hall_north', 'hall_west', 'hall_east'}
    expected |= {f'rock_{i}' for i in range(len(layout['boulders']))}
    expected |= set(extra_models)
    assert expected == set(names)
    uris = _uris(tree.getroot())
    assert uris
    for u in uris:
        assert '://' not in u, f'non-relative URI {u}'
        assert (root_dir / u).is_file(), f'missing {u}'
    assert yaml.safe_load((root_dir / 'truth' / 'truth.yaml').read_text())
    assert (root_dir / 'truth' / 'map.png').is_file()
    return tree


def _has_aruco():
    try:
        import cv2
        return bool(cv2.aruco)
    except Exception:  # noqa: BLE001
        return False


def test_schematic(tmp_path):
    cfg = config.load_config()
    lay = config.load_layout('schematic')
    d = world.write_world(cfg, lay, tmp_path)
    assert d == tmp_path / 'schematic'
    _check(d, lay)


needs_aruco = pytest.mark.skipif(not _has_aruco(), reason='cv2.aruco not installed')


def test_every_obj_has_a_material(tmp_path):
    """Each OBJ gets mtllib + a sibling mtl (gz warns 'Missing material' otherwise)."""
    cfg = config.load_config()
    d = world.write_world(cfg, config.load_layout('schematic'), tmp_path)
    objs = list((d / 'meshes').glob('*.obj'))
    assert len(objs) > 2
    for o in objs:
        text = o.read_text()
        assert f'mtllib {o.stem}.mtl' in text and 'usemtl ' in text, o
        assert (o.with_suffix('.mtl')).is_file(), o


@needs_aruco
def test_random_with_extras(tmp_path):
    cfg = config.load_config()
    lay = layout_mod.random_layout(cfg, 3)
    lay['name'] = 'seed_0003'
    d = world.write_world(cfg, lay, tmp_path, markers=True, sitcams=True, overlay=True,
                          low_gfx=True)
    names = ['sitcam_sw', 'sitcam_ne', 'overlay']
    names += [f'marker_{m["id"]}' for m in cfg['markers']['list']]
    _check(d, lay, names)


def test_truth_is_fenced_off(tmp_path):
    """Nothing outside truth/ (and arena.sdf) may carry obstacle positions: no yaml, no map."""
    cfg = config.load_config()
    lay = layout_mod.random_layout(cfg, 4)
    lay['name'] = 'seed_0004'
    d = world.write_world(cfg, lay, tmp_path, markers=_has_aruco(), overlay=True)
    outside = [p for p in d.rglob('*') if p.is_file() and 'truth' not in p.relative_to(d).parts]
    assert outside
    for p in outside:
        assert p.suffix not in ('.yaml', '.yml', '.json'), p
        assert 'map' not in p.name, p
        if p.suffix not in ('.sdf', '.png', '.obj', '.mtl'):
            raise AssertionError(f'unexpected file {p}')
    # installed share/, when available: worlds/ holds no truth, truth/ holds it
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('cac_sim'))
    except Exception:  # noqa: BLE001
        return
    worlds = share / 'worlds'
    if worlds.is_dir():
        for p in worlds.rglob('*'):
            assert p.name not in ('truth.yaml', 'map.png') and 'truth' not in p.parts, p
            assert p.suffix not in ('.yaml', '.yml'), p


def test_cli_truth_warning(tmp_path):
    r = subprocess.run([sys.executable, str(SCRIPT), '--layout', 'empty', '--out', str(tmp_path)],
                       capture_output=True, text=True, check=True)
    assert 'robot stack must never read it' in r.stdout
    assert 'info [empty]:' in r.stderr           # strict deviations are visible, not silent
    assert (tmp_path / 'empty' / 'truth' / 'truth.yaml').is_file()


def _overlaps(a, b):
    return a[0] < b[2] and a[2] > b[0] and a[1] < b[3] and a[3] > b[1]


def test_markers_clear_of_start_zone():
    """Every collision shape of every marker lies outside the start zone interior."""
    cfg = config.load_config()
    start = cfg['zones']['start']
    for mid, boxes in world.marker_footprints(cfg).items():
        for b in boxes:
            assert not _overlaps(b, start), (mid, b)
    # and in the SDF: no marker collision element at all on the board
    cfg_sdf = ET.fromstring(world._pretty('<r>' + ''.join(world._markers(
        cfg, {m['id']: 'x.png' for m in cfg['markers']['list']})) + '</r>'))
    for model in cfg_sdf.findall('model'):
        board = [lk for lk in model.findall('link') if lk.get('name') == 'board'][0]
        assert board.find('collision') is None


def test_marker_rules():
    cfg = config.load_config()
    assert world.check_markers(cfg) == []
    bad = {**cfg, 'markers': {**cfg['markers'], 'list': [
        {'id': i, 'position': [3, 3, 0.55], 'yaw_deg': 0} for i in range(5)]}}
    assert len(world.check_markers(bad)) == 6      # too many + 5 boards too tall


def _cli_run(out, *extra):
    subprocess.run([sys.executable, str(SCRIPT), '--layout', 'random', '--seed', '7', '--out',
                    str(out), '--markers', '--sitcams', '--overlay', *extra],
                   capture_output=True, text=True, check=True)


def test_deterministic_across_processes(tmp_path):
    """Two separate processes produce byte-identical files (sdf, meshes, textures, truth)."""
    a, b = tmp_path / 'a', tmp_path / 'b'
    _cli_run(a)
    _cli_run(b)
    files_a = sorted(p.relative_to(a) for p in a.rglob('*') if p.is_file())
    files_b = sorted(p.relative_to(b) for p in b.rglob('*') if p.is_file())
    assert files_a == files_b and files_a
    for f in files_a:
        assert filecmp.cmp(a / f, b / f, shallow=False), f'{f} differs between runs'


def test_lighting_and_hall_floor(tmp_path):
    cfg = config.load_config()
    for low in (False, True):
        root = world.write_world(cfg, layout_mod.random_layout(cfg, 3), tmp_path / str(low),
                                 low_gfx=low)
        w = ET.parse(root / 'arena.sdf').getroot().find('world')
        casters = [light.get('name') for light in w.findall('light')
                   if light.findtext('cast_shadows') == 'true']
        assert casters == ([] if low else ['key'])
        assert w.findtext('scene/shadows') == ('false' if low else 'true')
        assert len(w.findall('light')) >= 5
        L, W = cfg['arena']['size']
        for m in w.findall('model'):
            if not m.get('name').startswith('hall_'):
                continue
            assert m.find('.//collision') is None
            x, y, _ = (float(v) for v in m.findtext('pose').split()[:3])
            sx, sy, _ = (float(v) for v in m.findtext('.//box/size').split())
            inside_x = x + sx / 2 > -0.3 + 1e-9 and x - sx / 2 < L + 0.3 - 1e-9
            inside_y = y + sy / 2 > -0.3 + 1e-9 and y - sy / 2 < W + 0.3 - 1e-9
            assert not (inside_x and inside_y), f'{m.get("name")} within 0.3 m of the arena'


@pytest.mark.parametrize('mode', ['tiles', 'mesh'])
def test_terrain_sdf_carries_configured_friction(tmp_path, mode):
    """Every sand collision emits terrain.friction for both engines.

    ode mu = mu2 for dartsim, bullet friction = friction2 for bullet-featherstone.
    dartsim: effective on the tiles' boxes, inert on mesh collisions. bullet-featherstone:
    effective on both (README "How the sand drives").
    """
    cfg = config.load_config()
    cfg['terrain']['friction'] = 0.37
    cfg['sand']['collision'] = mode
    root = world.write_world(cfg, config.load_layout('schematic'), tmp_path)
    w = ET.parse(root / 'arena.sdf').getroot().find('world')
    terrain = [m for m in w.findall('model') if m.get('name') == 'terrain']
    assert len(terrain) == 1
    cols = terrain[0].findall('link/collision')
    assert cols
    for c in cols:
        ode = c.find('surface/friction/ode')
        assert ode is not None, c.get('name')
        assert float(ode.findtext('mu')) == pytest.approx(0.37)
        assert float(ode.findtext('mu2')) == pytest.approx(0.37)
        bullet = c.find('surface/friction/bullet')     # bullet-featherstone reads only these
        assert bullet is not None, c.get('name')
        assert float(bullet.findtext('friction')) == pytest.approx(0.37)
        assert float(bullet.findtext('friction2')) == pytest.approx(0.37)
    boxes = [c for c in cols if c.find('geometry/box') is not None]
    assert len(boxes) == (len(cols) - 1 if mode == 'tiles' else 0)
    assert config.load_config()['terrain']['friction'] == pytest.approx(0.5)
    assert config.load_config()['sand']['collision'] == 'tiles'   # the shipped default


def _physics(root):
    w = ET.parse(root / 'arena.sdf').getroot().find('world')
    phys = w.findall('physics')
    plugins = [p for p in w.findall('plugin') if p.get('name') == 'gz::sim::systems::Physics']
    assert len(phys) == 1 and len(plugins) == 1
    return phys[0].get('type'), plugins[0].findtext('engine/filename').strip()


@pytest.mark.parametrize('engine,plugin', [
    ('bullet-featherstone', 'gz-physics-bullet-featherstone-plugin'),
    ('dart', 'gz-physics-dartsim-plugin')])
def test_physics_block_reflects_config(tmp_path, engine, plugin):
    """physics.engine sets BOTH <physics type> and the Physics system's <engine><filename>.

    gz-sim 8 picks the engine from <engine><filename> (or --physics-engine), not from the type:
    type="bullet-featherstone" alone still loads dartsim (README "Physics engine").
    """
    cfg = config.load_config()
    cfg['physics'] = {'engine': engine}
    root = world.write_world(cfg, config.load_layout('schematic'), tmp_path)
    assert _physics(root) == (engine, plugin)


def test_physics_default_and_cli_override(tmp_path):
    assert config.load_config()['physics']['engine'] == 'dart'   # shipped default
    for flag, want in ((None, 'dart'), ('dart', 'dart'),
                       ('bullet-featherstone', 'bullet-featherstone')):
        out = tmp_path / str(flag)
        args = ['--layout', 'empty', '--out', str(out)]
        args += ['--physics-engine', flag] if flag else []
        subprocess.run([sys.executable, str(SCRIPT), *args], check=True, capture_output=True)
        assert _physics(out / 'empty')[0] == want
    bad = subprocess.run([sys.executable, str(SCRIPT), '--layout', 'empty', '--out',
                          str(tmp_path / 'bad'), '--physics-engine', 'ode'], capture_output=True)
    assert bad.returncode != 0


def test_physics_engine_rejects_unknown():
    cfg = config.load_config()
    cfg['physics'] = {'engine': 'ode'}
    with pytest.raises(ValueError):
        world.physics_engine(cfg)
    del cfg['physics']
    assert world.physics_engine(cfg) == 'dart'


def test_engine_changes_only_the_physics_block(tmp_path):
    """Engines dart vs bullet-featherstone: same bytes except <physics> type and engine plugin."""
    roots = {}
    for eng in ('dart', 'bullet-featherstone'):
        cfg = config.load_config()
        cfg['physics'] = {'engine': eng}
        roots[eng] = world.write_world(cfg, config.load_layout('schematic'), tmp_path / eng)
    a, b = ((roots[e] / 'arena.sdf').read_text() for e in ('dart', 'bullet-featherstone'))
    a = a.replace('type="dart"', 'type="bullet-featherstone"').replace(
        'gz-physics-dartsim-plugin', 'gz-physics-bullet-featherstone-plugin')
    assert a == b
    others = [f.relative_to(roots['dart']) for f in roots['dart'].rglob('*')
              if f.is_file() and f.name != 'arena.sdf']
    assert others
    for f in others:
        assert filecmp.cmp(roots['dart'] / f, roots['bullet-featherstone'] / f, shallow=False), f


def test_marker_geometry_lives_in_markers_module():
    cfg = config.load_config()
    assert world.marker_footprints(cfg) == markers_mod.marker_footprints(cfg)
    assert markers_mod.post_circles(cfg)


def _run(*args):
    return subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True)


def test_cli_markers_unavailable_is_an_error(tmp_path):
    """Asking for --markers without usable aruco: stderr message, non-zero exit, no world."""
    bad = tmp_path / 'bad.yaml'
    cfg = yaml.safe_load(config.DEFAULT_CONFIG.read_text())
    cfg['markers']['dictionary'] = 'DICT_NOPE_99'
    bad.write_text(yaml.safe_dump(cfg))
    r = _run('--layout', 'empty', '--out', str(tmp_path / 'o'), '--markers', '--config', str(bad))
    if _has_aruco():
        assert r.returncode != 0 and r.stdout == ''
        assert 'unknown ArUco dictionary' in r.stderr and 'DICT_NOPE_99' in r.stderr
        assert 'not available' not in r.stderr
    else:
        assert r.returncode != 0 and 'cv2.aruco is not available' in r.stderr
    assert not (tmp_path / 'o' / 'empty').exists()


def test_marker_error_kinds(monkeypatch):
    """An ImportError and a bad dictionary name give different messages."""
    cfg = config.load_config()
    monkeypatch.setitem(sys.modules, 'cv2', None)            # import cv2 -> ImportError
    with pytest.raises(world.MarkerError, match='not available'):
        world.check_marker_support(cfg)
    monkeypatch.undo()
    if _has_aruco():
        bad = {**cfg, 'markers': {**cfg['markers'], 'dictionary': 'DICT_NOPE'}}
        with pytest.raises(world.MarkerError, match='unknown ArUco dictionary'):
            world.check_marker_support(bad)


@needs_aruco
def test_write_world_markers_raises_not_silent(tmp_path):
    cfg = config.load_config()
    bad = {**cfg, 'markers': {**cfg['markers'], 'dictionary': 'DICT_NOPE'}}
    with pytest.raises(world.MarkerError):
        world.write_world(bad, config.empty_layout(bad), tmp_path, markers=True)


def test_posts_reserved_without_markers_flag(tmp_path):
    """A world generated WITHOUT --markers still has path-checked, post-aware truth."""
    cfg = config.load_config()
    lay = layout_mod.random_layout(cfg, 13)
    d = world.write_world(cfg, lay, tmp_path)
    t = yaml.safe_load((d / 'truth' / 'truth.yaml').read_text())
    assert t['paths_include_marker_posts'] and t['marker_posts'] == []
    assert t['se2_path_exists']


def _files(root):
    return {p.relative_to(root).as_posix(): p.read_bytes() for p in sorted(root.rglob('*'))
            if p.is_file()}


def test_vary_appearance_changes_only_looks(tmp_path):
    cfg = config.load_config()
    lay = layout_mod.random_layout(cfg, 7)
    lay['name'] = 'a'
    base = _files(world.write_world(cfg, lay, tmp_path / 'off', markers=False))
    off2 = _files(world.write_world(cfg, lay, tmp_path / 'off2'))
    assert base == off2                                   # flag off: deterministic
    on = _files(world.write_world(cfg, lay, tmp_path / 'on', vary_appearance=True))
    same_seed = _files(world.write_world(cfg, lay, tmp_path / 'on2', vary_appearance=True))
    assert on == same_seed                                # same seed: identical look
    changed = {k for k in base if base[k] != on[k]}
    assert set(base) == set(on)
    # meshes, normal maps, mtl and truth are byte-identical; only the look files differ
    assert {'arena.sdf', 'materials/sand.png'} <= changed
    assert changed <= {'arena.sdf', 'materials/sand.png', 'materials/rock_0.png',
                       'materials/rock_1.png', 'materials/rock_2.png'}
    other = _files(world.write_world(cfg, {**lay, 'seed': 8}, tmp_path / 'o8',
                                     vary_appearance=True))
    assert other['arena.sdf'] != on['arena.sdf']
    assert other['materials/sand.png'] != on['materials/sand.png']
    # independent look seed: same layout and geometry, different appearance
    k1 = _files(world.write_world(cfg, lay, tmp_path / 'k1', vary_appearance=True,
                                  appearance_seed=1))
    assert k1['arena.sdf'] != on['arena.sdf']
    assert {k: v for k, v in k1.items() if k.startswith(('meshes/', 'truth/'))} == {
        k: v for k, v in base.items() if k.startswith(('meshes/', 'truth/'))}


def test_appearance_within_bounds():
    from cac_sim_gen import appearance
    cfg = config.load_config()
    ap = cfg['appearance']
    for s in range(60):
        a = appearance.sample(cfg, s)
        assert ap['light_level'][0] <= a['light_level'] <= ap['light_level'][1]
        assert ap['color_temp_k'][0] <= a['color_temp_k'] <= ap['color_temp_k'][1]
        assert abs(a['key_dir'][0] ** 2 + a['key_dir'][1] ** 2 + a['key_dir'][2] ** 2 - 1) < 1e-9
        assert a['key_dir'][2] < -0.8
        assert all(0.05 <= c <= 0.95 for c in a['sand_albedo'])
        assert all(0.4 < t < 1.7 for t in a['tint'])      # tinted, never neon
        w = appearance.wall_rgb(world.WALL_RGB, a)
        assert all(0.55 <= c <= 0.97 for c in w)


def test_cli_vary_appearance_flag(tmp_path):
    for flags, name in ((['--vary-appearance'], 'v'), (['--appearance-seed', '3'], 'k')):
        subprocess.run([sys.executable, str(SCRIPT), '--layout', 'random', '--seed', '7',
                        '--out', str(tmp_path / name), *flags],
                       check=True, capture_output=True)
    plain = tmp_path / 'p'
    subprocess.run([sys.executable, str(SCRIPT), '--layout', 'random', '--seed', '7',
                    '--out', str(plain)], check=True, capture_output=True)
    sdfs = [(tmp_path / n / 'seed_0007' / 'arena.sdf').read_text() for n in 'vkp']
    assert len(set(sdfs)) == 3


# Golden output without --vary-appearance. If you change the arena ON PURPOSE (geometry, layout,
# lighting defaults), regenerate and update these hashes in the same commit, and say why.
GOLDEN_SDF_SHA256 = {
    # 2026-10-27: denser fields with obstacles biased onto the rover's routes (intentional)
    # 2026-10-27: terrain.friction 0.9 -> 0.5 (dry-sand estimate); only the terrain <mu>/<mu2>
    # bytes differ (checked by substituting 0.9 back: old hash reproduced). Inert in gz-sim 8.
    # 2026-10-28: sand.collision added; these are the 'mesh' mode bytes (no longer the default).
    # 2026-10-28 later (both tables): physics.engine bullet-featherstone (rover mimic joints) adds
    # <engine><filename> to the Physics plugin + type="bullet-featherstone", and every sand
    # collision gains <bullet><friction>/<friction2>. Checked: reverting just those two strings
    # reproduces the previous hashes exactly (all four).
    # 2026-10-28 final: default engine back to dart (type="dart" + dartsim plugin) and sand gains
    # <torsional><coefficient>0 (terrain.torsional); all four hashes regenerated for those two changes.
    ('random', '3'): '1f51f4c4176c6777b2995943e665a61c521063f95a44b2848d7e5259992d7e02',
    ('schematic', None): '6c60dee88e3906ca0a383d818d3cd307adc791e9d564e7f91ff55791afe4ca5f',
}
# 2026-10-28 later: crater patches subdivided 3 x 3 (review: small bowls were ~2 cm short).
# --sand-collision tiles = the DEFAULT since 2026-10-28: only the terrain <collision> elements
# differ from mesh mode; the visual mesh, textures and truth are byte-identical
# (test_tiles_change_only_...).
GOLDEN_SDF_SHA256_TILES = {
    ('random', '3'): 'd9da8709308da487aa8b73b81b3c491ae400f24766a7ece25e52c20a404f5907',
    ('schematic', None): 'fb32ca64c63be08e8b3c91da844918fb20def89bd187d3aef9c5cb67733bfe3e',
}


@pytest.mark.parametrize('mode', ['mesh', 'tiles'])
@pytest.mark.parametrize('layout,seed', list(GOLDEN_SDF_SHA256))
def test_default_output_matches_golden(tmp_path, layout, seed, mode):
    args = ['--layout', layout, '--out', str(tmp_path), '--sand-collision', mode]
    args += ['--seed', seed] if seed else []
    subprocess.run([sys.executable, str(SCRIPT), *args], check=True, capture_output=True)
    name = f'seed_{int(seed):04d}' if seed else layout
    digest = hashlib.sha256((tmp_path / name / 'arena.sdf').read_bytes()).hexdigest()
    golden = GOLDEN_SDF_SHA256_TILES if mode == 'tiles' else GOLDEN_SDF_SHA256
    assert digest == golden[(layout, seed)]


def test_default_sand_collision_is_tiles(tmp_path):
    """Without the flag the arena uses tiles (friction effective on flat sand)."""
    args = ['--layout', 'schematic', '--out', str(tmp_path)]
    subprocess.run([sys.executable, str(SCRIPT), *args], check=True, capture_output=True)
    digest = hashlib.sha256((tmp_path / 'schematic' / 'arena.sdf').read_bytes()).hexdigest()
    assert digest == GOLDEN_SDF_SHA256_TILES[('schematic', None)]


# ----------------------------------------------------------------------------- sand collision

def _sand_collision(root):
    """(boxes [(x0, y0, x1, y1, z_top, z_bottom)], patch triangles (M, 3, 3) or None)."""
    w = ET.parse(root / 'arena.sdf').getroot().find('world')
    link = [m for m in w.findall('model') if m.get('name') == 'terrain'][0].find('link')
    boxes, patch = [], None
    for c in link.findall('collision'):
        if c.find('geometry/box') is not None:
            x, y, z = (float(v) for v in c.findtext('pose').split()[:3])
            sx, sy, sz = (float(v) for v in c.findtext('geometry/box/size').split())
            boxes.append((x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2, z + sz / 2, z - sz / 2))
        else:
            assert c.findtext('geometry/mesh/uri').strip() == 'meshes/terrain_collision.obj'
            lines = (root / 'meshes' / 'terrain_collision.obj').read_text().splitlines()
            v = np.array([[float(t) for t in ln.split()[1:]] for ln in lines
                          if ln.startswith('v ')])
            f = np.array([[int(t.split('/')[0]) - 1 for t in ln.split()[1:]] for ln in lines
                          if ln.startswith('f ')])
            patch = v[f]
    return boxes, patch


def _in_triangles(px, py, tri):
    """Bool (P,): point inside (or on) the xy projection of any triangle."""
    a, b, c = tri[:, 0, :2], tri[:, 1, :2], tri[:, 2, :2]
    p = np.stack([px, py], axis=1)[:, None, :]

    def cross(o, u, q):
        return (u[..., 0] - o[..., 0]) * (q[..., 1] - o[..., 1]) - \
            (u[..., 1] - o[..., 1]) * (q[..., 0] - o[..., 0])
    d1, d2, d3 = cross(a, b, p), cross(b, c, p), cross(c, a, p)
    neg = (d1 < -1e-9) | (d2 < -1e-9) | (d3 < -1e-9)
    pos = (d1 > 1e-9) | (d2 > 1e-9) | (d3 > 1e-9)
    return (~(neg & pos)).any(axis=1)


TILE_LAYOUTS = [('schematic', None), ('empty', None), ('crater_field', None),
                ('busy_dig', None), ('random', 3), ('random', 16)]


def _tile_world(cfg, tmp_path, name, seed):
    cfg['sand']['collision'] = 'tiles'
    if name == 'random':
        lay = layout_mod.random_layout(cfg, seed)
    elif name == 'empty':
        lay = config.empty_layout(cfg)
    else:
        lay = config.load_layout(name)
    return lay, world.write_world(cfg, lay, tmp_path)


@pytest.mark.parametrize('name,seed', TILE_LAYOUTS)
def test_tiles_cover_flat_sand_exactly_once(tmp_path, name, seed):
    """Every 10 cm grid point is under exactly one box top at z = 0, or in a crater patch."""
    cfg = config.load_config()
    lay, root = _tile_world(cfg, tmp_path, name, seed)
    boxes, patch = _sand_collision(root)
    L, W = cfg['arena']['size']
    d = cfg['arena']['sand_depth']
    for x0, y0, x1, y1, top, bottom in boxes:
        assert top == pytest.approx(0.0, abs=1e-9) and bottom == pytest.approx(-d, abs=1e-9)
        assert -1e-7 <= x0 < x1 <= L + 1e-7 and -1e-7 <= y0 < y1 <= W + 1e-7
    gx, gy = np.meshgrid(np.arange(0.0, L + 1e-9, 0.1), np.arange(0.0, W + 1e-9, 0.1),
                         indexing='ij')
    gx, gy = gx.ravel(), gy.ravel()
    in_box = np.zeros(gx.shape, dtype=int)
    for x0, y0, x1, y1, _t, _b in boxes:       # half-open, so a seam point counts once
        in_box += ((gx >= x0 - 1e-6) & (gx < x1 - 1e-6) | (gx >= L - 1e-6) & (x1 >= L - 1e-6)) & \
            ((gy >= y0 - 1e-6) & (gy < y1 - 1e-6) | (gy >= W - 1e-6) & (y1 >= W - 1e-6))
    in_patch = np.zeros(gx.shape, dtype=bool) if patch is None else _in_triangles(gx, gy, patch)
    # a point on a box/patch seam is legitimately both; otherwise exactly one
    assert ((in_box == 1) | in_patch).all(), 'uncovered sand'
    assert (in_box <= 1).all(), 'overlapping boxes'
    interior = np.ones(gx.shape, dtype=bool)
    if patch is not None:
        lo, hi = patch[..., :2].min(axis=1), patch[..., :2].max(axis=1)
        strict = np.zeros(gx.shape, dtype=bool)
        for (ax, ay), (bx, by) in zip(lo, hi):
            strict |= (gx > ax + 1e-6) & (gx < bx - 1e-6) & (gy > ay + 1e-6) & (gy < by - 1e-6)
        assert not (strict & (in_box > 0)).any(), 'box overlaps a crater patch'
        interior = ~strict
    # every crater, out to its outer radius, is patch (mesh), never box
    for i in range(len(lay['craters'])):
        p = geometry.crater_params(cfg, lay, i, lay.get('seed') or 0)
        cx, cy = p['position']
        near = np.hypot(gx - cx, gy - cy) <= p['outer_radius']
        assert in_patch[near].all() and not (in_box[near & interior] > 0).any()
    # areas add up to the arena: no gaps, no overlaps
    area = sum((x1 - x0) * (y1 - y0) for x0, y0, x1, y1, _t, _b in boxes)
    if patch is not None:
        e1, e2 = patch[:, 1, :2] - patch[:, 0, :2], patch[:, 2, :2] - patch[:, 0, :2]
        area += 0.5 * np.abs(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]).sum()
        # the patch border meets the box tops: every vertex outside all outer radii is z = 0
        v = patch.reshape(-1, 3)
        out = np.ones(len(v), dtype=bool)
        for i in range(len(lay['craters'])):
            p = geometry.crater_params(cfg, lay, i, lay.get('seed') or 0)
            out &= np.hypot(v[:, 0] - p['position'][0], v[:, 1] - p['position'][1]) > \
                p['outer_radius']
        assert out.any() and np.abs(v[out, 2]).max() < 1e-4
    else:
        assert not lay['craters']
    assert area == pytest.approx(L * W, abs=1e-3)


def test_tile_count_bounded(tmp_path):
    """Greedy merge keeps the box count small on every shipped and random layout."""
    cfg = config.load_config()
    names = ('schematic', 'gate', 'column_pinch', 'crater_field', 'busy_dig', 'start_trap')
    lays = [config.load_layout(n) for n in names]
    lays += [layout_mod.random_layout(cfg, s) for s in range(1, 21)]
    counts = []
    for lay in lays:
        rects, patch = geometry.sand_collision_tiles(cfg, lay, lay.get('seed') or 0)
        counts.append(len(rects))
        assert (patch is None) == (not lay['craters'])
    assert max(counts) <= 40, counts
    assert len(geometry.sand_collision_tiles(cfg, config.empty_layout(cfg), 0)[0]) == 1


def test_tiles_change_only_the_collision(tmp_path):
    """Tiles vs mesh: visual mesh, rocks, textures and truth are byte-identical."""
    cfg = config.load_config()
    lay = layout_mod.random_layout(cfg, 3)
    cfg['sand']['collision'] = 'tiles'
    a = world.write_world(cfg, lay, tmp_path / 'tiles')
    cfg_m = config.load_config()
    cfg_m['sand']['collision'] = 'mesh'
    b = world.write_world(cfg_m, lay, tmp_path / 'mesh')
    fa = sorted(p.relative_to(a) for p in a.rglob('*') if p.is_file())
    assert fa == sorted(p.relative_to(b) for p in b.rglob('*') if p.is_file())
    differ = {str(f) for f in fa if not filecmp.cmp(a / f, b / f, shallow=False)}
    assert differ == {'arena.sdf', 'meshes/terrain_collision.obj'}
    bad = config.load_config()
    bad['sand']['collision'] = 'heightmap'
    with pytest.raises(ValueError):
        world.write_world(bad, lay, tmp_path / 'bad')


def test_tiles_without_craters_have_no_mesh_collision(tmp_path):
    cfg = config.load_config()
    cfg['sand']['collision'] = 'tiles'
    root = world.write_world(cfg, config.empty_layout(cfg), tmp_path)
    boxes, patch = _sand_collision(root)
    assert len(boxes) == 1 and patch is None
    assert not (root / 'meshes' / 'terrain_collision.obj').exists()


# ----------------------------------------------------------------------------- set-up sheet

def _tree_bytes(root):
    return {str(p.relative_to(root)): p.read_bytes() for p in sorted(root.rglob('*'))
            if p.is_file()}


def test_setup_sheet_only_with_flag_and_other_bytes_unchanged(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    base = [sys.executable, str(SCRIPT), '--layout', 'random', '--seed', '7']
    subprocess.run(base + ['--out', str(a)], capture_output=True, check=True)
    subprocess.run(base + ['--out', str(b), '--setup-sheet'], capture_output=True, check=True)
    ta, tb = _tree_bytes(a), _tree_bytes(b)
    assert not any('setup_' in k for k in ta)
    extra = set(tb) - set(ta)
    assert extra == {'seed_0007/truth/setup_sheet.md', 'seed_0007/truth/setup_plan.png'}
    assert all(tb[k] == v for k, v in ta.items())            # nothing else changed


def test_setup_sheet_matches_truth(tmp_path):
    cfg = config.resolve(config.load_config())
    lay = layout_mod.random_layout(cfg, 7)
    md = layout_mod.setup_sheet(cfg, lay, tmp_path)
    assert md.parent == tmp_path / lay['name'] / 'truth'
    text = md.read_text()
    t = layout_mod.truth(cfg, lay)
    rows = {}
    for line in text.splitlines():
        cells = [c.strip() for c in line.strip('|').split('|')]
        if len(cells) == 11 and cells[1][:1] in 'BC' and cells[1][1:].isdigit():
            rows[cells[1]] = cells
    assert len(rows) == len(t['obstacles'])
    xs = [int(r[3]) for r in rows.values()]
    assert xs == sorted(xs)                                  # sorted by x
    seen = {'boulder': 0, 'crater': 0}
    for o in t['obstacles']:
        k = o['type']
        seen[k] += 1
        idx = seen[k]
        r = rows[f'{k[0].upper()}{idx}']
        assert abs(int(r[3]) - o['position'][0] * 100) <= 0.5 + 1e-6
        assert abs(int(r[4]) - o['position'][1] * 100) <= 0.5 + 1e-6
        assert abs(int(r[5]) - o['radius'] * 200) <= 0.5 + 1e-6
        if k == 'crater':
            assert abs(float(r[7]) - o['depth'] * 100) <= 0.05 + 1e-6
            assert abs(float(r[8]) - o['rim'] * 100) <= 0.05 + 1e-6
        else:
            assert abs(int(r[6]) - o['height'] * 100) <= 0.5 + 1e-6
    sp = t['start_pose']
    assert f'x = {round(sp["x"] * 100)} cm, y = {round(sp["y"] * 100)} cm' in text
    assert 'west and south walls meet' in text


def test_setup_sheet_deterministic_and_markers(tmp_path):
    cfg = config.resolve(config.load_config())
    lay = layout_mod.random_layout(cfg, 7)
    p1 = layout_mod.setup_sheet(cfg, lay, tmp_path / '1', markers=True)
    p2 = layout_mod.setup_sheet(cfg, lay, tmp_path / '2', markers=True)
    assert p1.read_bytes() == p2.read_bytes()
    png = 'setup_plan.png'
    assert (p1.parent / png).read_bytes() == (p2.parent / png).read_bytes()
    assert 'ArUco markers' in p1.read_text()
    p3 = layout_mod.setup_sheet(cfg, lay, tmp_path / '3')
    assert 'ArUco markers' not in p3.read_text()


@pytest.mark.parametrize('name', ['schematic', 'crater_field', 'column_pinch'])
def test_crater_patch_keeps_bowl_depth(name):
    """The collision patch reaches each crater's floor (within 3 mm) and its border is flat."""
    from cac_sim_gen import geometry
    cfg = config.load_config()
    lay = config.load_layout(name)
    _, patch = geometry.sand_collision_tiles(cfg, lay, lay.get('seed'))
    v = patch.vertices
    for i in range(len(lay['craters'])):
        p = geometry.crater_params(cfg, lay, i, lay.get('seed'))
        near = ((v[:, 0] - p['position'][0]) ** 2 + (v[:, 1] - p['position'][1]) ** 2
                < (p['diameter'] / 2) ** 2)
        assert v[near, 2].min() <= -p['depth_eff'] + 0.003
