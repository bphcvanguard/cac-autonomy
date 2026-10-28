"""Seeded appearance variation (opt-in): lighting, sand, rock and wall colours only.

Never touches geometry, collision, layout or truth. Draws from its own RNG stream
`[appearance_seed, STREAM_APPEARANCE]` (DESIGN.md), so the layout and every other stream are
unchanged whether or not it is used. Bounds are in `arena.yaml` under `appearance:`.
"""
import math

import numpy as np

STREAM_APPEARANCE = 70      # [appearance_seed, 70]; ids 2, 3, 7, 20-26, 30, 50+k are taken

# Neutral anchors for the unvaried scene (world.py): wall tint target and key-light direction.
WALL_GREY = (0.74, 0.74, 0.73)
BASE_KEY_DIR = (-0.25, 0.2, -0.95)
NEUTRAL_K = 4800.0          # colour temperature that leaves the lamps' colour unchanged

_KEYS = ('light_level', 'ambient_factor', 'color_temp_k', 'key_elevation_deg',
         'key_azimuth_jitter_deg', 'sand_brightness', 'sand_grey_mix', 'sand_tint_jitter',
         'sand_contrast', 'rock_brightness', 'wall_greyness', 'wall_lightness')


def _kelvin_rgb(k):
    """Blackbody colour (Tanner Helland fit), 0..1 per channel, valid for 1000..40000 K."""
    t = k / 100.0
    r = 1.0 if t <= 66 else 1.292936 * (t - 60) ** -0.1332047592
    g = (0.390082 * math.log(t) - 0.631841) if t <= 66 else 1.129891 * (t - 60) ** -0.0755148492
    b = 1.0 if t >= 66 else (0.0 if t <= 19 else 0.543207 * math.log(t - 10) - 1.196254)
    return np.clip([r, g, b], 0.0, 1.0)


def light_tint(k):
    """RGB multiplier for colour temperature k, luminance-preserving, 1.0 at NEUTRAL_K."""
    t = _kelvin_rgb(k) / _kelvin_rgb(NEUTRAL_K)
    return t / (0.2126 * t[0] + 0.7152 * t[1] + 0.0722 * t[2])


def sample(cfg, appearance_seed):
    """Return the appearance dict for one seed (pure function of cfg['appearance'] and seed)."""
    ap = cfg.get('appearance')
    if ap is None:
        raise ValueError('--vary-appearance needs an `appearance:` section in arena.yaml '
                         '(your --config file predates it; copy it from the packaged arena.yaml)')
    missing = [k for k in _KEYS if k not in ap]
    if missing:
        raise KeyError(f'arena.yaml appearance: is missing {missing}')
    rng = np.random.default_rng([int(appearance_seed), STREAM_APPEARANCE])
    u = {k: float(rng.uniform(*ap[k])) for k in _KEYS}      # fixed draw order = _KEYS
    palette_offset = int(rng.integers(0, 4))
    az0 = math.atan2(BASE_KEY_DIR[1], BASE_KEY_DIR[0])
    az = az0 + math.radians(u['key_azimuth_jitter_deg'])
    el = math.radians(u['key_elevation_deg'])
    key_dir = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el))

    sand = np.asarray(cfg['sand']['albedo'], dtype=float)
    grey = float(sand @ [0.2126, 0.7152, 0.0722])
    sand = sand * (1 - u['sand_grey_mix']) + grey * u['sand_grey_mix']
    sand = sand * u['sand_brightness'] * (1 + rng.uniform(-1, 1, 3) * u['sand_tint_jitter'])
    return {'appearance_seed': int(appearance_seed), 'light_level': u['light_level'],
            'ambient': u['ambient_factor'] * u['light_level'], 'key_dir': key_dir,
            'tint': tuple(float(x) for x in light_tint(u['color_temp_k'])),
            'color_temp_k': u['color_temp_k'],
            'sand_albedo': tuple(float(x) for x in np.clip(sand, 0.05, 0.95)),
            'sand_contrast': u['sand_contrast'], 'rock_palette_offset': palette_offset,
            'rock_brightness': u['rock_brightness'],
            'wall_greyness': u['wall_greyness'], 'wall_lightness': u['wall_lightness']}


def wall_rgb(base, app):
    """Beige (base) -> light grey by `wall_greyness`, then scaled by `wall_lightness`."""
    g = app['wall_greyness']
    rgb = np.asarray(base) * (1 - g) + np.asarray(WALL_GREY) * g
    return tuple(float(x) for x in np.clip(rgb * app['wall_lightness'], 0.0, 0.97))
