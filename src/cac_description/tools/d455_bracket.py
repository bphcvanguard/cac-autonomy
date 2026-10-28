"""FreeCADCmd script: D455 camera bracket for the rover's front top cross member.

All dimensions in mm, in the rover's base_link frame (x forward, y left, z up), so the exported
part drops straight into the URDF. Pose from /home/kpoola/cac-e2e/cammount (camera centre
(351.7, 0, 190.3) in base_link, pitched 23.0 deg nose-down).

One printed PLA part:
  saddle   U over the 30 x 30 member (top + front + back walls), clamped by 2 x M5 bolts that run
           fore-aft UNDER the member through the two walls
  web      riser from the saddle top up and forward to the camera plate
  plate    tilted 23 deg, 1/4-20 clearance hole for the D455 tripod thread, back lip so the camera
           cannot rotate about the screw

Usage: FreeCADCmd d455_bracket.py OUT_DIR   -> d455_bracket.step, d455_bracket.stl
"""
import math
import os
import sys

import FreeCAD as App
from FreeCAD import Vector as V
import Mesh
import Part

OUT = sys.argv[-1]

# member (base_link, mm)
MX0, MX1 = 246.2, 276.2          # back / front faces
MZ0, MZ1 = 69.9, 99.9            # bottom / top faces
WALL = 5.0                        # saddle wall thickness
CLAMP_W = 60.0                    # saddle length along y
DROP = 12.0                       # walls hang this far below the member for the M5 bolts
M5_CLEAR = 5.5

# camera (D455 body 124 (y) x 26 (x, depth) x 29 (z); centre and pitch from the study)
CAM_C = V(351.7, 0.0, 190.3)
PITCH = math.radians(23.0)
CAM_H = 29.0
PLATE_T = 6.0                     # plate thickness
PLATE_U = 30.0                    # plate length along camera x (camera depth 26 + margin)
PLATE_W = 70.0                    # plate width along y
LIP_H = 6.0                       # back lip height above the plate
LIP_T = 4.0
TRIPOD_CLEAR = 6.8                # 1/4-20 UNC clearance
CBORE_D, CBORE_DEPTH = 12.0, 2.0  # screw head counterbore under the plate
WEB_W = 30.0

ex = V(math.cos(PITCH), 0, -math.sin(PITCH))     # camera x (forward, tilted down)
ez = V(math.sin(PITCH), 0, math.cos(PITCH))      # camera z (up, tilted forward)
P = CAM_C - ez * (CAM_H / 2)                      # centre of the camera's bottom face


def at(u, v):                                     # point in the camera's xz plane, y = 0
    return P + ex * u + ez * v


def prism(points_xz, y0, y1):
    """Extrude a closed polygon in the x-z plane between y0 and y1."""
    pts = [V(x, y0, z) for x, z in points_xz] + [V(points_xz[0][0], y0, points_xz[0][1])]
    face = Part.Face(Part.makePolygon(pts))
    return face.extrude(V(0, y1 - y0, 0))


hw = CLAMP_W / 2
# saddle: outer U minus the member
outer = Part.makeBox(MX1 - MX0 + 2 * WALL, CLAMP_W, (MZ1 - MZ0) + WALL + DROP,
                     V(MX0 - WALL, -hw, MZ0 - DROP))
member = Part.makeBox(MX1 - MX0, CLAMP_W + 2, (MZ1 - MZ0) + DROP + 1,
                      V(MX0, -hw - 1, MZ0 - DROP - 1))
saddle = outer.cut(member)
# M5 bolt holes fore-aft under the member, at y = +-18
for y in (-18.0, 18.0):
    hole = Part.makeCylinder(M5_CLEAR / 2, MX1 - MX0 + 2 * WALL + 2,
                             V(MX0 - WALL - 1, y, MZ0 - DROP / 2), V(1, 0, 0))
    saddle = saddle.cut(hole)

# plate (tilted) as a polygon prism in the camera xz plane
pu, pt = PLATE_U / 2, PLATE_T
plate = prism([(at(-pu, -pt).x, at(-pu, -pt).z), (at(pu, -pt).x, at(pu, -pt).z),
               (at(pu, 0).x, at(pu, 0).z), (at(-pu, 0).x, at(-pu, 0).z)],
              -PLATE_W / 2, PLATE_W / 2)
lip = prism([(at(-pu, 0).x, at(-pu, 0).z), (at(-pu + LIP_T, 0).x, at(-pu + LIP_T, 0).z),
             (at(-pu + LIP_T, LIP_H).x, at(-pu + LIP_T, LIP_H).z),
             (at(-pu, LIP_H).x, at(-pu, LIP_H).z)], -PLATE_W / 2, PLATE_W / 2)
# web from the saddle top up to the plate underside
top_z = MZ1 + WALL
web = prism([(MX0 - WALL, top_z), (MX1 + WALL, top_z), (at(pu, -pt).x, at(pu, -pt).z),
             (at(-pu, -pt).x, at(-pu, -pt).z)], -WEB_W / 2, WEB_W / 2)

body = saddle.fuse(web).fuse(plate).fuse(lip).removeSplitter()
# 1/4-20 clearance through the plate along camera z, plus head counterbore from below
axis = ez
hole = Part.makeCylinder(TRIPOD_CLEAR / 2, PLATE_T + 40, P - axis * (PLATE_T + 30), axis)
cbore = Part.makeCylinder(CBORE_D / 2, 30 + CBORE_DEPTH, P - axis * (PLATE_T + 30), axis)
body = body.cut(hole).cut(cbore)

os.makedirs(OUT, exist_ok=True)
body.exportStep(os.path.join(OUT, 'd455_bracket.step'))
doc = App.newDocument('b')
obj = doc.addObject('Part::Feature', 'bracket')
obj.Shape = body
Mesh.export([obj], os.path.join(OUT, 'd455_bracket.stl'))
bb = body.BoundBox
print('volume_cm3 %.1f' % (body.Volume / 1000))
print('bbox_mm x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (bb.XMin, bb.XMax, bb.YMin, bb.YMax,
                                                          bb.ZMin, bb.ZMax))
print('valid', body.isValid(), 'solids', len(body.Solids))
