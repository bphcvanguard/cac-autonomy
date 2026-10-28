"""Shared mesh container."""
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Mesh:
    vertices: np.ndarray            # (N, 3) float
    faces: np.ndarray               # (M, 3) int, counter-clockwise seen from outside / above
    uvs: Optional[np.ndarray] = None      # (N, 2)
    normals: Optional[np.ndarray] = None  # (N, 3)
