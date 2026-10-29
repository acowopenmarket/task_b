"""Adapt existing recognition outputs; never load a model or send robot commands."""
from coordinate_transform.geometry import finite_vector


def medical_center(item):
    """Adapter only; no model imports, camera opens, or robot_interface calls."""
    key = 'position_xyz' if 'position_xyz' in item else 'position_3d'
    p = finite_vector(item[key],3)*.001
    if p[2] <= 0:
        raise ValueError('Medical position has no valid measured depth')
    return p
