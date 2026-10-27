"""ArUco marker board geometry shared by the world writer and the layout checks.

Boards are visual only. A free-standing board stands on a thin collision post, and that post is
a real obstacle: the layout path checks and the ground truth treat every configured post as a
small disc whether or not the world is generated with --markers (the team will run with markers).
"""
import math

MARKER_MARGIN = 0.01      # clearance between a marker's collision footprint and the start zone
POST_R = 0.01
BOARD_TH = 0.01
TAG_PX = 200              # rendered tag size in pixels
BORDER_PX = 50            # white border around the tag in pixels


def marker_list(cfg):
    """Return the configured marker list (empty if the config has none)."""
    return cfg.get('markers', {}).get('list', [])


def board_side(cfg):
    """Return the board edge in metres: the printed tag plus its white border."""
    return cfg['markers']['size'] * (TAG_PX + 2 * BORDER_PX) / TAG_PX


def wall_mounted(cfg, m):
    """Return True if the marker sits flush on an arena wall (such a board has no post)."""
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
    if wall_mounted(cfg, m):
        return x, y
    side = board_side(cfg)
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
    for m in marker_list(cfg):
        if wall_mounted(cfg, m):
            out[m['id']] = []
            continue
        x, y = marker_placement(cfg, m)
        out[m['id']] = [(x - POST_R, y - POST_R, x + POST_R, y + POST_R)]
    return out


def post_circles(cfg):
    """Return [(id, x, y, r)] for every collision post of the configured marker preset."""
    out = []
    for mid, boxes in marker_footprints(cfg).items():
        for x0, y0, x1, y1 in boxes:
            out.append((mid, (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2))
    return out


def check_markers(cfg):
    """Rule check: at most 4 markers, every board top below 0.60 m."""
    v = []
    lst = marker_list(cfg)
    if len(lst) > 4:
        v.append(f'{len(lst)} markers; the rules allow at most 4')
    half = board_side(cfg) / 2
    for m in lst:
        top = m['position'][2] + half
        if top >= 0.60:
            v.append(f'marker {m["id"]}: board top at {top:.2f} m, must be below 0.60 m')
    return v
