"""Assemble a URDF at zero pose (no ROS needed): wheel-ground contact check + 3-view render."""
import math
import sys
import xml.etree.ElementTree as ET

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402,I100
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

sys.path.insert(0, '/home/kpoola/cad/tools')
from build_urdf import read_stl  # noqa: E402,I100

urdf, pkg, out = sys.argv[1:4]
r = ET.parse(urdf).getroot()


def rpy(r_, p, y):
    cx, sx = math.cos(r_), math.sin(r_)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)
    return np.array([[cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
                     [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
                     [-sy, cy*sx, cy*cx]])


T = {'base_footprint': np.eye(4)}
joints = r.findall('joint')
while len(T) < len(r.findall('link')):
    for j in joints:
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        if p in T and c not in T:
            o = j.find('origin')
            M = np.eye(4)
            M[:3, :3] = rpy(*map(float, o.get('rpy').split()))
            M[:3, 3] = list(map(float, o.get('xyz').split()))
            T[c] = T[p] @ M
cols = {'wheel': '#222', 'rocker': '#e67e22', 'bogie': '#2980b9', 'diff_bar': '#c0392b',
        'base_link': '#7f8c8d'}
fig = plt.figure(figsize=(18, 6))
axs = [fig.add_subplot(1, 3, k + 1, projection='3d') for k in range(3)]
print('wheel bottom z in base_footprint (m):')
for L in r.findall('link'):
    n = L.get('name')
    v = L.find('visual')
    if v is None:
        continue
    fn = v.find('geometry/mesh').get('filename').replace('package://cac_description/',
                                                         pkg + '/')
    t = read_stl(fn).reshape(-1, 3)
    t = (t @ T[n][:3, :3].T + T[n][:3, 3]).reshape(-1, 3, 3)
    if 'wheel' in n:
        c = T[n][:3, 3]
        print(f'  {n:20s} centre {np.round(c, 4)}  bottom {t[:, :, 2].min():+.4f}')
    col = next((cc for k, cc in cols.items() if k in n), '#27ae60')
    tt = t[:: max(1, len(t) // 3000)]
    for ax in axs:
        ax.add_collection3d(Poly3DCollection(tt, facecolor=col, edgecolor='none'))
views = [(0, -90, 'side: robot +x right'), (0, 0, 'front view (from +x)'), (20, -60, 'iso')]
for ax, (el, az, ttl) in zip(axs, views):
    ax.set_xlim(-0.6, 1.1)
    ax.set_ylim(-0.85, 0.85)
    ax.set_zlim(-0.05, 1.0)
    ax.set_box_aspect((1.7, 1.7, 1.05))
    ax.view_init(el, az)
    ax.set_title(ttl)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    xx, yy = np.meshgrid([-0.6, 1.1], [-0.85, 0.85])
    ax.plot_surface(xx, yy, 0 * xx, alpha=0.15, color='tan')
plt.tight_layout()
plt.savefig(out, dpi=80)
