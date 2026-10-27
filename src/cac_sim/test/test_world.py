"""World writer: well-formed SDF, truth fenced off, legal markers, deterministic output."""
import filecmp
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from cac_sim_gen import config, layout as layout_mod, world
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


def test_random_with_extras(tmp_path):
    cfg = config.load_config()
    lay = layout_mod.random_layout(cfg, 3)
    lay['name'] = 'seed_0003'
    d = world.write_world(cfg, lay, tmp_path, markers=True, sitcams=True, overlay=True,
                          low_gfx=True)
    names = ['sitcam_sw', 'sitcam_ne', 'overlay']
    if _has_aruco():
        names += [f'marker_{m["id"]}' for m in cfg['markers']['list']]
    _check(d, lay, names)


def test_truth_is_fenced_off(tmp_path):
    """Nothing outside truth/ (and arena.sdf) may carry obstacle positions: no yaml, no map."""
    cfg = config.load_config()
    lay = layout_mod.random_layout(cfg, 4)
    lay['name'] = 'seed_0004'
    d = world.write_world(cfg, lay, tmp_path, markers=True, overlay=True)
    outside = [p for p in d.rglob('*') if p.is_file() and 'truth' not in p.relative_to(d).parts]
    assert outside
    for p in outside:
        assert p.suffix not in ('.yaml', '.yml', '.json'), p
        assert 'map' not in p.name, p
        if p.suffix not in ('.sdf', '.png', '.obj'):
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
