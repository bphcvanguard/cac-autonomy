"""World writer: SDF is well-formed, every referenced file exists, truth.yaml loads."""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cac_sim_gen import config, layout as layout_mod, world  # noqa: E402


def _uris(root):
    return [u.text.strip() for u in root.iter() if u.tag in ('uri', 'albedo_map') and u.text]


def _check(root_dir, layout, extra_models=()):
    sdf = root_dir / 'arena.sdf'
    tree = ET.parse(sdf)                     # well-formed
    models = tree.getroot().find('world').findall('model')
    names = [m.get('name') for m in models]
    assert len(names) == len(set(names)), 'duplicate model names'
    expected = {'terrain', 'subfloor', 'wall_south', 'wall_north', 'wall_west', 'wall_east', 'column'}
    expected |= {f'rock_{i}' for i in range(len(layout['boulders']))}
    expected |= set(extra_models)
    assert expected == set(names)
    uris = _uris(tree.getroot())
    assert uris
    for u in uris:
        assert '://' not in u, f'non-relative URI {u}'
        assert (root_dir / u).is_file(), f'missing {u}'
    truth = yaml.safe_load((root_dir / 'truth.yaml').read_text())
    assert truth
    assert (root_dir / 'map.png').is_file()
    return tree


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
    d = world.write_world(cfg, lay, tmp_path, markers=True, sitcams=True, overlay=True, low_gfx=True)
    names = ['sitcam_sw', 'sitcam_ne', 'overlay']
    try:
        import cv2
        cv2.aruco  # noqa: B018
        names += [f'marker_{m["id"]}' for m in cfg['markers']['list']]
    except Exception:  # noqa: BLE001
        pass
    _check(d, lay, names)


def test_deterministic(tmp_path):
    cfg = config.load_config()
    lay = layout_mod.random_layout(cfg, 5)
    lay['name'] = 'a'
    a = world.write_world(cfg, lay, tmp_path)
    lay['name'] = 'b'
    b = world.write_world(cfg, lay, tmp_path)
    assert (a / 'arena.sdf').read_text() == (b / 'arena.sdf').read_text()
