"""FreeCADCmd: export every leaf solid as an STL in WORLD coordinates, scaled mm -> m."""
import json
import os
import re
import sys

import FreeCAD
import Import
import MeshPart

path, outdir = sys.argv[-2], sys.argv[-1]
os.makedirs(outdir, exist_ok=True)
doc = FreeCAD.newDocument('d')
Import.insert(path, doc.Name)
index, seen = [], {}
for o in doc.Objects:
    if (o.TypeId != 'Part::Feature' or not hasattr(o, 'Shape') or o.Shape.isNull()
            or o.Shape.Volume <= 0):
        continue
    s = o.Shape.copy()
    s.Placement = o.getGlobalPlacement()
    s.scale(0.001)                                   # mm -> m
    m = MeshPart.meshFromShape(Shape=s, LinearDeflection=0.0005, AngularDeflection=0.35)
    base = re.sub(r'[^A-Za-z0-9]+', '_', o.Label).strip('_') or 'part'
    seen[base] = seen.get(base, 0) + 1
    fn = f'{base}_{seen[base]}.stl' if seen[base] > 1 else f'{base}.stl'
    m.write(os.path.join(outdir, fn))
    index.append({'label': o.Label, 'file': fn, 'facets': m.CountFacets})
json.dump(index, open(os.path.join(outdir, 'index.json'), 'w'), indent=1)
print('exported', len(index))
