"""Make cac_sim_gen importable when tests run from the source tree."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
