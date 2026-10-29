"""Import existing verified implementations without starting devices."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
for path in (PROJECT / 'camera', PROJECT / 'estun/ros2_ws/src/estun_segmented'):
    sys.path.insert(0, str(path))
