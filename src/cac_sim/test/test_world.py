"""World writer: well-formed SDF, truth fenced off, legal markers, deterministic output."""
import filecmp
import hashlib
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from cac_sim_gen import config, layout as layout_mod, markers as markers_mod, world
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
    ('random', '3'): '65d2085cd58212dde201bfe9854869afa4bb512db38834f0058e910543603a5b',
    ('schematic', None): 'cb17a6e899b7efb90b3fcdce3c33734f715ba19aa64d78c00474c3f7bc8a270f',
}


@pytest.mark.parametrize('layout,seed', list(GOLDEN_SDF_SHA256))
def test_default_output_matches_golden(tmp_path, layout, seed):
    args = ['--layout', layout, '--out', str(tmp_path)] + (['--seed', seed] if seed else [])
    subprocess.run([sys.executable, str(SCRIPT), *args], check=True, capture_output=True)
    name = f'seed_{int(seed):04d}' if seed else layout
    digest = hashlib.sha256((tmp_path / name / 'arena.sdf').read_bytes()).hexdigest()
    assert digest == GOLDEN_SDF_SHA256[(layout, seed)]


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
        if len(cells) == 9 and cells[1][:1] in 'BC' and cells[1][1:].isdigit():
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
