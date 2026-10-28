"""Put a robot (URDF) into a generated arena world at the layout's start pose.

The URDF is converted with `gz sdf -p` (Gazebo must be installed). Its package:// meshes are copied
into <world>/robot/<package>/ and referenced by relative paths, so the world directory stays
self-contained and movable, like the rest of the arena.
"""
import math
from pathlib import Path
import re
import shutil
import subprocess

DROP = 0.05     # m: spawn height above z = 0, so the wheels settle onto the tilled sand


class RobotError(RuntimeError):
    pass


def robot_model(urdf, root, start_pose, name='rover'):
    """Return an SDF <model> string for `urdf`, placed at start_pose ({x, y, yaw_deg})."""
    urdf = Path(urdf).resolve()
    try:
        out = subprocess.run(['gz', 'sdf', '-p', str(urdf)], capture_output=True, text=True,
                             check=True).stdout
    except (OSError, subprocess.CalledProcessError) as e:
        raise RobotError(f'cannot convert {urdf} with `gz sdf -p` ({e})') from e
    m = re.search(r'<model\b.*</model>', out, re.S)
    if not m:
        raise RobotError(f'no <model> in the SDF converted from {urdf}')
    model = m.group(0)
    pkg_dir = urdf.parent.parent                 # <package>/urdf/<file>.urdf
    for pkg, rel in set(re.findall(r'(?:package|model)://([A-Za-z0-9_]+)/([^<"\s]+)', model)):
        src = pkg_dir / rel if pkg_dir.name == pkg else None
        if src is None or not src.is_file():
            raise RobotError(f'mesh {pkg}/{rel} not found next to {urdf}')
        dst = Path(root) / 'robot' / pkg / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst)
    model = re.sub(r'(?:package|model)://([A-Za-z0-9_]+)/', r'robot/\1/', model)
    yaw = math.radians(start_pose.get('yaw_deg', 0.0))
    pose = f'<pose>{start_pose["x"]:.4f} {start_pose["y"]:.4f} {DROP} 0 0 {yaw:.6f}</pose>'
    return re.sub(r"<model name=['\"][^'\"]*['\"]>", f"<model name='{name}'>{pose}", model,
                  count=1)
