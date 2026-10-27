"""Write one self-contained Gazebo Harmonic world directory.

arena.sdf + meshes/ + materials/, plus truth/ (truth.yaml + map.png, harness only).

All URIs in arena.sdf are RELATIVE to the SDF file, so the directory can be moved, installed or
generated anywhere. No Fuel/https/model:// URIs.
"""
import math
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np
import yaml

from . import geometry, layout as layout_mod

# Visual / collision mesh resolutions in metres (budgets: visual <= 80k tris, collision <= 20k).
VISUAL_RES = 0.04
COLLISION_RES = 0.08
FINE_RES = 0.02
SINK = 0.01          # rock_mesh already extends 4 cm below its origin; this makes the total ~5 cm
OVERLAY_Z = 0.02     # above the +/- tilled-sand relief (a z of 0.002 would be buried by it)

WALL_RGB = (0.84, 0.78, 0.64)       # light-beige painted plywood (Artemis Arena walls)
CONCRETE_RGB = (0.62, 0.62, 0.60)   # painted concrete column
SUBFLOOR_RGB = (0.62, 0.50, 0.36)   # hidden: matches sand tone in case it ever shows
HALL_FLOOR_RGB = (0.50, 0.51, 0.52)  # polished-concrete hall floor outside the walls
ROCK_VARIANTS = 3
HALL_MARGIN = 0.3        # hall floor starts at least this far outside the inner wall faces
HALL_EXTENT = 30.0       # how far the hall floor reaches beyond the arena
LIGHT_HEIGHT = 4.5       # overhead lamps above the sand


def _f(*vals):
    return ' '.join(f'{v:.6g}' for v in vals)


def _pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return f'<pose>{_f(x, y, z, roll, pitch, yaw)}</pose>'


def _material(rgb, rough=0.9, metal=0.0, emissive=None, alpha=1.0):
    r, g, b = rgb
    em = f'<emissive>{_f(*emissive, 1)}</emissive>' if emissive else ''
    col = _f(r, g, b, alpha)
    return (f'<material><ambient>{col}</ambient><diffuse>{col}</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular>{em}'
            f'<pbr><metal><roughness>{rough}</roughness><metalness>{metal}</metalness>'
            '</metal></pbr></material>')


def _tex_material(albedo_uri, rough=0.95, normal_uri=None):
    nm = f'<normal_map>{escape(normal_uri)}</normal_map>' if normal_uri else ''
    return ('<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
            '<specular>0.05 0.05 0.05 1</specular>'
            f'<pbr><metal><albedo_map>{escape(albedo_uri)}</albedo_map>{nm}'
            f'<roughness>{rough}</roughness><metalness>0</metalness></metal></pbr></material>')


def _box_model(name, size, pose, material, collision=True, shadows=True):
    sx, sy, sz = size
    geom = f'<geometry><box><size>{_f(sx, sy, sz)}</size></box></geometry>'
    col = f'<collision name="collision">{geom}</collision>' if collision else ''
    sh = '' if shadows else '<cast_shadows>false</cast_shadows>'
    return (f'<model name="{name}"><static>true</static>{pose}<link name="link">{col}'
            f'<visual name="visual">{sh}{geom}{material}</visual></link></model>')


def _mesh_geom(uri, scale=None):
    sc = f'<scale>{_f(*scale)}</scale>' if scale else ''
    return f'<geometry><mesh><uri>{escape(uri)}</uri>{sc}</mesh></geometry>'


def _surface(cfg, layout, x, y, seed):
    return float(np.asarray(geometry.surface_height(cfg, layout, x, y, seed)).ravel()[0])


def _walls(cfg):
    L, W = cfg['arena']['size']
    t = cfg['arena']['wall']['thickness']
    h = cfg['arena']['wall']['height']
    d = cfg['arena']['sand_depth']
    H = h + d
    zc = (h - d) / 2
    mat = _material(WALL_RGB, rough=0.85)
    out = []
    # inner faces exactly at x=0, x=L, y=0, y=W; the long walls span the corners
    out.append(_box_model('wall_south', (L + 2 * t, t, H), _pose(L / 2, -t / 2, zc), mat))
    out.append(_box_model('wall_north', (L + 2 * t, t, H), _pose(L / 2, W + t / 2, zc), mat))
    out.append(_box_model('wall_west', (t, W, H), _pose(-t / 2, W / 2, zc), mat))
    out.append(_box_model('wall_east', (t, W, H), _pose(L + t / 2, W / 2, zc), mat))
    return out


def _floor(cfg):
    """Hidden solid floor under the sand, so nothing falls through the open terrain mesh."""
    L, W = cfg['arena']['size']
    d = cfg['arena']['sand_depth']
    return _box_model('subfloor', (L, W, 0.2), _pose(L / 2, W / 2, -d - 0.1),
                      _material(SUBFLOOR_RGB, rough=1.0), shadows=False)


def _hall_floor(cfg):
    """Visual-only grey hall floor outside the walls, so the horizon is not empty sky.

    Four slabs whose tops sit just below the sand (-sand_depth - 2 cm), so from inside the arena
    the walls always hide them. They start at least HALL_MARGIN outside the inner wall faces
    (never inside the arena), have no collision and cast no shadows.
    """
    L, W = cfg['arena']['size']
    t = cfg['arena']['wall']['thickness']
    d = cfg['arena']['sand_depth']
    m = max(HALL_MARGIN, t + 0.02)
    e = HALL_EXTENT
    th = 0.1
    z = -d - 0.02 - th / 2
    mat = _material(HALL_FLOOR_RGB, rough=0.6)
    slabs = (('hall_south', L + 2 * m + 2 * e, e, L / 2, -m - e / 2),
             ('hall_north', L + 2 * m + 2 * e, e, L / 2, W + m + e / 2),
             ('hall_west', e, W + 2 * m, -m - e / 2, W / 2),
             ('hall_east', e, W + 2 * m, L + m + e / 2, W / 2))
    return [_box_model(n, (sx, sy, th), _pose(x, y, z), mat, collision=False, shadows=False)
            for n, sx, sy, x, y in slabs]


def _column(cfg):
    c = cfg['column']
    d = cfg['arena']['sand_depth']
    x, y = c['centre']
    length = c['height'] + d
    if c.get('shape', 'cylinder') == 'box':
        sx, sy = c['size']
        geom = f'<geometry><box><size>{_f(sx, sy, length)}</size></box></geometry>'
    else:
        geom = (f'<geometry><cylinder><radius>{c["radius"]}</radius>'
                f'<length>{length}</length></cylinder></geometry>')
    yaw = np.radians(c.get('yaw_deg', 0.0))
    return (f'<model name="column"><static>true</static>'
            f'{_pose(x, y, (c["height"] - d) / 2, yaw=yaw)}'
            f'<link name="link"><collision name="collision">{geom}</collision>'
            f'<visual name="visual">{geom}{_material(CONCRETE_RGB, rough=0.85)}'
            '</visual></link></model>')


def _pipes(cfg):
    """Return the optional PVC ducts lying on the sand inside every wall (Artemis Arena)."""
    dia = cfg['arena']['wall'].get('inner_pipe_diameter', 0.0)
    if not dia:
        return []
    L, W = cfg['arena']['size']
    r = dia / 2
    mat = _material((0.92, 0.92, 0.9), rough=0.5)
    out = []
    for name, length, x, y, yaw in (('pipe_south', L, L / 2, r, 0.0),
                                    ('pipe_north', L, L / 2, W - r, 0.0),
                                    ('pipe_west', W - 4 * r, r, W / 2, np.pi / 2),
                                    ('pipe_east', W - 4 * r, L - r, W / 2, np.pi / 2)):
        geom = (f'<geometry><cylinder><radius>{_f(r)}</radius><length>{_f(length)}</length>'
                '</cylinder></geometry>')
        out.append(f'<model name="{name}"><static>true</static>'
                   f'{_pose(x, y, r, roll=0.0, pitch=np.pi / 2, yaw=yaw)}'
                   f'<link name="link"><collision name="collision">{geom}</collision>'
                   f'<visual name="visual">{geom}{mat}</visual></link></model>')
    return out


def _terrain():
    return ('<model name="terrain"><static>true</static><pose>0 0 0 0 0 0</pose><link name="link">'
            '<collision name="collision">'
            f'{_mesh_geom("meshes/terrain_collision.obj")}'
            '<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>'
            '</collision>'
            f'<visual name="visual">{_mesh_geom("meshes/terrain.obj")}'
            f'{_tex_material("materials/sand.png", 0.97, "materials/sand_normal.png")}'
            '</visual></link></model>')


def _boulder(cfg, layout, i, b, seed, yaw):
    x, y = b['position']
    z = _surface(cfg, layout, x, y, seed) - SINK
    geom = _mesh_geom(f'meshes/rock_{i}.obj')
    tex = _tex_material(f'materials/rock_{i % ROCK_VARIANTS}.png', 0.9,
                        'materials/rock_normal.png')
    return (f'<model name="rock_{i}"><static>true</static>{_pose(x, y, z, 0, 0, yaw)}'
            '<link name="link">'
            f'<collision name="collision">{geom}</collision>'
            f'<visual name="visual">{geom}{tex}</visual></link></model>')


def _marker_images(cfg, out_dir):
    """Write marker_<id>.png (tag + white border). Returns {id: file}, or {} without cv2.aruco."""
    try:
        import cv2
        aruco = cv2.aruco
        dic = aruco.getPredefinedDictionary(getattr(aruco, cfg['markers']['dictionary']))
    except Exception as e:  # noqa: BLE001 - any import/attr failure means "no aruco here"
        print(f'warning: cv2.aruco not usable ({e}); skipping --markers')
        return {}
    px, border = 200, 50
    files = {}
    for m in cfg['markers']['list']:
        if hasattr(aruco, 'generateImageMarker'):
            img = aruco.generateImageMarker(dic, m['id'], px)
        else:
            img = aruco.drawMarker(dic, m['id'], px)
        img = cv2.copyMakeBorder(img, border, border, border, border,
                                 cv2.BORDER_CONSTANT, value=255)
        name = f'marker_{m["id"]}.png'
        cv2.imwrite(str(out_dir / name), img)
        files[m['id']] = name
    return files


MARKER_MARGIN = 0.01      # clearance between a marker's collision footprint and the start zone
POST_R = 0.01
BOARD_TH = 0.01


def _board_side(cfg):
    return cfg['markers']['size'] * (200 + 2 * 50) / 200     # tag + white border


def _wall_mounted(cfg, m):
    x, y, _ = m['position']
    w, h = cfg['arena']['size']
    return x < 0.02 or y < 0.02 or x > w - 0.02 or y > h - 0.02


def marker_placement(cfg, m):
    """Return (x, y) of a marker, moved clear of the start zone.

    The board and post footprints (conservative axis-aligned boxes) are moved just far enough to
    stay outside the start zone interior. Wall-mounted boards are flush with the wall and
    unchanged (they carry no post).
    """
    x, y, _ = m['position']
    yaw = math.radians(m['yaw_deg'])
    if _wall_mounted(cfg, m):
        return x, y
    side = _board_side(cfg)
    hx = (abs(math.sin(yaw)) * side + abs(math.cos(yaw)) * BOARD_TH) / 2
    hy = (abs(math.cos(yaw)) * side + abs(math.sin(yaw)) * BOARD_TH) / 2
    hx, hy = max(hx, POST_R), max(hy, POST_R)
    zx0, zy0, zx1, zy1 = cfg['zones']['start']
    inside_x = x - hx < zx1 and x + hx > zx0
    inside_y = y - hy < zy1 and y + hy > zy0
    if inside_x and inside_y:
        dx = zx1 + MARKER_MARGIN - (x - hx)
        dy = zy1 + MARKER_MARGIN - (y - hy)
        if dx <= dy:
            x += dx
        else:
            y += dy
    return x, y


def marker_footprints(cfg):
    """{id: [(x0, y0, x1, y1), ...]} of every COLLISION shape of each marker (the post only)."""
    out = {}
    for m in cfg['markers']['list']:
        if _wall_mounted(cfg, m):
            out[m['id']] = []
            continue
        x, y = marker_placement(cfg, m)
        out[m['id']] = [(x - POST_R, y - POST_R, x + POST_R, y + POST_R)]
    return out


def check_markers(cfg):
    """Rule check: at most 4 markers, every board top below 0.60 m."""
    v = []
    lst = cfg['markers']['list']
    if len(lst) > 4:
        v.append(f'{len(lst)} markers; the rules allow at most 4')
    half = _board_side(cfg) / 2
    for m in lst:
        top = m['position'][2] + half
        if top >= 0.60:
            v.append(f'marker {m["id"]}: board top at {top:.2f} m, must be below 0.60 m')
    return v


def _markers(cfg, files):
    """Boards are visual only; free-standing ones get a thin collision post outside the start."""
    side = _board_side(cfg)
    out = []
    for m in cfg['markers']['list']:
        if m['id'] not in files:
            continue
        z = m['position'][2]
        yaw = math.radians(m['yaw_deg'])
        x, y = marker_placement(cfg, m)
        # nudge along the facing direction so a wall-mounted board does not z-fight with the wall
        px, py = x + 0.006 * math.cos(yaw), y + 0.006 * math.sin(yaw)
        post = ''
        if not _wall_mounted(cfg, m):
            ph = z - side / 2
            pg = (f'<geometry><cylinder><radius>{POST_R}</radius><length>{ph:.4f}</length>'
                  '</cylinder></geometry>')
            post = (f'<link name="post">{_pose(x, y, ph / 2)}'
                    f'<collision name="collision">{pg}</collision>'
                    f'<visual name="visual">{pg}{_material((0.3, 0.3, 0.3))}</visual></link>')
        geom = f'<geometry><box><size>{_f(BOARD_TH, side, side)}</size></box></geometry>'
        tex = _tex_material('materials/' + files[m['id']], 1.0)
        out.append(
            f'<model name="marker_{m["id"]}"><static>true</static>'
            f'<link name="board">{_pose(px, py, z, 0, 0, yaw)}'
            f'<visual name="visual">{geom}{tex}</visual></link>{post}</model>')
    return out


def _sitcams(cfg):
    out = []
    for c in cfg['situational_cameras']:
        px, py, pz = c['position']
        lx, ly, lz = c['look_at']
        dx, dy, dz = lx - px, ly - py, lz - pz
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, math.hypot(dx, dy))
        n = c['name']
        out.append(
            f'<model name="{n}"><static>true</static>'
            f'<link name="link">{_pose(px, py, pz, 0, pitch, yaw)}'
            '<sensor name="camera" type="camera"><always_on>true</always_on>'
            '<update_rate>5</update_rate>'
            f'<topic>/sitcam/{n}/image</topic><camera><horizontal_fov>1.2</horizontal_fov>'
            '<image><width>1280</width><height>720</height><format>R8G8B8</format></image>'
            '<clip><near>0.1</near><far>40</far></clip></camera></sensor></link></model>')
    return out


def _strip_rect(name, rect, rgb, width=0.05):
    x0, y0, x1, y1 = rect
    mat = _material(rgb, emissive=rgb)
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    sides = [(mx, y0 + width / 2, x1 - x0, width), (mx, y1 - width / 2, x1 - x0, width),
             (x0 + width / 2, my, width, y1 - y0), (x1 - width / 2, my, width, y1 - y0)]
    vis = ''.join(
        f'<visual name="{name}_{k}">{_pose(cx, cy, OVERLAY_Z)}<cast_shadows>false</cast_shadows>'
        f'<geometry><box><size>{_f(sx, sy, 0.004)}</size></box></geometry>{mat}</visual>'
        for k, (cx, cy, sx, sy) in enumerate(sides))
    return vis


def _overlay(cfg, layout):
    z = cfg['zones']
    vis = _strip_rect('start', z['start'], (0.1, 0.9, 0.1))
    vis += _strip_rect('obstacle', z['obstacle'], (0.95, 0.85, 0.1))
    vis += _strip_rect('excavation', z['excavation'], (0.2, 0.4, 1.0))
    vis += _strip_rect('construction', z['construction'], (1.0, 0.5, 0.0))
    cx, cy = cfg['berm']['target_centre']
    tw, th = cfg['berm']['target_size']
    vis += _strip_rect('target', [cx - tw / 2, cy - th / 2, cx + tw / 2, cy + th / 2],
                       (1.0, 0.1, 0.1), 0.04)
    sp = layout.get('start_pose')
    if sp:
        L, Wd = cfg['randomiser']['robot_footprint']
        a = math.radians(sp.get('yaw_deg', 0.0))
        vis += (f'<visual name="robot_start">{_pose(sp["x"], sp["y"], OVERLAY_Z, 0, 0, a)}'
                '<cast_shadows>false</cast_shadows>'
                f'<geometry><box><size>{_f(L, Wd, 0.004)}</size></box></geometry>'
                f'{_material((0.1, 0.9, 0.1), emissive=(0.1, 0.5, 0.1), alpha=0.35)}</visual>')
    return ('<model name="overlay"><static>true</static><link name="link">'
            + vis + '</link></model>')


def _lights(cfg, low_gfx):
    """Indoor rig: a dim directional key plus a grid of overhead lamps (no shadows).

    Only the key casts shadows (one shadow pass is all a CPU laptop can afford) and --low-gfx
    turns even that off, so the low-gfx world has zero shadow casters.
    """
    L, W = cfg['arena']['size']
    sh = 'false' if low_gfx else 'true'
    out = (f'<light type="directional" name="key"><cast_shadows>{sh}</cast_shadows>'
           '<pose>0 0 10 0 0 0</pose><diffuse>0.55 0.55 0.52 1</diffuse>'
           '<specular>0.05 0.05 0.05 1</specular>'
           '<direction>-0.25 0.2 -0.95</direction></light>')
    for i, fx in enumerate((0.3, 0.7)):
        for j, fy in enumerate((0.27, 0.73)):
            out += (f'<light type="point" name="lamp_{i}{j}"><cast_shadows>false</cast_shadows>'
                    f'{_pose(fx * L, fy * W, LIGHT_HEIGHT)}'
                    '<diffuse>0.42 0.42 0.4 1</diffuse><specular>0.02 0.02 0.02 1</specular>'
                    '<attenuation><range>14</range><constant>1.0</constant><linear>0.0</linear>'
                    '<quadratic>0.004</quadratic></attenuation></light>')
    return out


def _header(low_gfx):
    return (
        '<physics name="1ms" type="dart"><max_step_size>0.001</max_step_size>'
        '<real_time_factor>1.0</real_time_factor></physics>'
        '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
        '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>'
        '<plugin filename="gz-sim-scene-broadcaster-system"'
        ' name="gz::sim::systems::SceneBroadcaster"/>'
        '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
        '<render_engine>ogre2</render_engine></plugin>'
        '<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>'
        '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>'
        '<scene><ambient>0.42 0.42 0.42 1</ambient><background>0.62 0.63 0.64 1</background>'
        f'<shadows>{"false" if low_gfx else "true"}</shadows><grid>false</grid></scene>')


def _pretty(sdf):
    import xml.dom.minidom
    return xml.dom.minidom.parseString(sdf).toprettyxml(indent='  ')


def write_world(cfg, layout, out_dir, markers=False, sitcams=False, overlay=False, low_gfx=False):
    """Write <out_dir>/<layout name>/ and return that directory (the world is its arena.sdf)."""
    name = layout.get('name') or 'arena'
    root = Path(out_dir) / name
    (root / 'meshes').mkdir(parents=True, exist_ok=True)
    (root / 'materials').mkdir(parents=True, exist_ok=True)
    seed = layout.get('seed')
    seed = 0 if seed is None else int(seed)

    terrain = geometry.build_terrain(cfg, layout, seed, VISUAL_RES, FINE_RES)
    geometry.write_obj(root / 'meshes' / 'terrain.obj', terrain)
    coarse = geometry.build_terrain(cfg, layout, seed, COLLISION_RES, COLLISION_RES)
    geometry.write_obj(root / 'meshes' / 'terrain_collision.obj', coarse)
    geometry.sand_texture(root / 'materials' / 'sand.png', seed, 1024,
                          albedo=tuple(cfg['sand']['albedo']))
    geometry.sand_normal_map(root / 'materials' / 'sand_normal.png', seed, 1024)
    geometry.rock_texture_variants(root / 'materials', seed, ROCK_VARIANTS)
    geometry.rock_normal_map(root / 'materials' / 'rock_normal.png', seed, 512)

    yaw_rng = np.random.default_rng([seed, 7])    # own stream: boulder yaw
    models = [_terrain(), _floor(cfg), *_hall_floor(cfg), *_walls(cfg), *_pipes(cfg), _column(cfg)]
    for i, b in enumerate(layout['boulders']):
        geometry.write_obj(root / 'meshes' / f'rock_{i}.obj',
                           geometry.rock_mesh(b['diameter'], b['height'], seed * 1000 + i))
        models.append(_boulder(cfg, layout, i, b, seed, float(yaw_rng.uniform(0, 2 * math.pi))))
    if markers:
        models += _markers(cfg, _marker_images(cfg, root / 'materials'))
    if sitcams:
        models += _sitcams(cfg)
    if overlay:
        models.append(_overlay(cfg, layout))

    sdf = (f'<?xml version="1.0"?><sdf version="1.9"><world name="cac_arena">{_header(low_gfx)}'
           f'{_lights(cfg, low_gfx)}{"".join(models)}</world></sdf>')
    (root / 'arena.sdf').write_text(_pretty(sdf))

    # Ground truth lives in its own sub-directory so it is never installed with the world.
    (root / 'truth').mkdir(exist_ok=True)
    with open(root / 'truth' / 'truth.yaml', 'w') as f:
        f.write('# Ground truth for the test harness / scoring ONLY.\n'
                '# The robot stack must never read it: obstacle locations may not be uploaded.\n')
        yaml.safe_dump(layout_mod.truth(cfg, layout), f, sort_keys=False)
    layout_mod.render_map(cfg, layout, str(root / 'truth' / 'map.png'))
    return root
