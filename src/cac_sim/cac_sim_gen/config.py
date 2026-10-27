"""Load the arena config and layout files."""
from pathlib import Path

import yaml

PKG_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = PKG_ROOT / 'config' / 'arena.yaml'
LAYOUT_DIR = PKG_ROOT / 'layouts'


def _find_share(rel):
    """Return a file from the source tree, or from the installed share/ after colcon build."""
    p = PKG_ROOT / rel
    if p.exists():
        return p
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('cac_sim')) / rel
    except Exception:
        return p


def load_config(path=None):
    path = Path(path) if path else _find_share('config/arena.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return resolve(cfg)


def resolve(cfg, marker_preset=None):
    """Fill derived keys: markers.list from the chosen preset, column.radius (bounding circle)."""
    m = cfg.get('markers', {})
    if 'presets' in m:
        m['preset'] = marker_preset or m.get('preset')
        m['list'] = m['presets'][m['preset']]
    c = cfg['column']
    if c.get('shape', 'cylinder') == 'box':
        c['radius'] = 0.5 * (c['size'][0] ** 2 + c['size'][1] ** 2) ** 0.5
    elif 'size' in c:
        c['radius'] = c['size'][0] / 2
    return cfg


def load_layout(path_or_name):
    """Load a layout YAML by path, or by name from layouts/ (e.g. 'schematic')."""
    p = Path(path_or_name)
    if not p.suffix:
        p = _find_share(f'layouts/{path_or_name}.yaml')
    with open(p) as f:
        layout = yaml.safe_load(f)
    layout.setdefault('boulders', [])
    layout.setdefault('craters', [])
    return layout


def empty_layout(cfg):
    sz = cfg['zones']['start']
    return {
        'name': 'empty', 'seed': None, 'boulders': [], 'craters': [],
        'start_pose': {'x': (sz[0] + sz[2]) / 2, 'y': (sz[1] + sz[3]) / 2, 'yaw_deg': 0.0},
    }
