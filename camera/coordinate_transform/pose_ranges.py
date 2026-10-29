"""Component ranges with continuous Euler angles across the +/-pi boundary."""
import numpy as np


def pose_component_ranges(poses):
    values = np.asarray(poses, dtype=float)
    if values.ndim != 2 or values.shape[0] < 1 or values.shape[1] != 6 or not np.isfinite(values).all():
        raise ValueError('Expected a nonempty finite Nx6 pose series')
    continuous = values.copy()
    continuous[:, 3:] = np.unwrap(values[:, 3:], axis=0)
    return np.ptp(continuous, axis=0).tolist()
