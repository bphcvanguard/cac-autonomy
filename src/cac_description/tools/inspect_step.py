"""FreeCADCmd script: list every leaf solid in a STEP with world placement, bbox, volume, CoM.

Bounding boxes are in mm.
"""
import json
import sys

import FreeCAD
import Import

path, out = sys.argv[-2], sys.argv[-1]
doc = FreeCAD.newDocument('d')
Import.insert(path, doc.Name)
rows = []
for o in doc.Objects:
    if not hasattr(o, 'Shape') or o.TypeId not in ('Part::Feature',):
        continue
    s = o.Shape
    if s.isNull() or s.Volume <= 0:
        continue
    gs = s.copy()
    gs.Placement = o.getGlobalPlacement()
    bb = gs.BoundBox
    parents = [p.Label for p in o.InListRecursive if p.TypeId == 'App::Part']
    rows.append({'name': o.Label, 'parents': parents, 'volume_mm3': gs.Volume,
                 'com_mm': list(gs.CenterOfMass) if hasattr(gs, 'CenterOfMass') else None,
                 'bbox_mm': [bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax],
                 'solids': len(gs.Solids)})
json.dump(rows, open(out, 'w'), indent=1)
print('parts', len(rows))
