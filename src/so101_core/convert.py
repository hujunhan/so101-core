"""The one place where LeRobot units meet model units.

LeRobot (use_degrees=True) reports the five arm joints in degrees, zero at the middle of the
range recorded at calibration, and the gripper in 0..100 % of its calibrated range. The model
("new calibration" URDF/MJCF) also puts each arm joint's zero at the middle of its range, so
the arm joints map as q = sign * radians(reading) + offset. The gripper maps linearly onto
the model's gripper limits.
"""

import numpy as np

from so101_core.config import JointMap
from so101_core.model import JOINTS, urdf_limits

_LO, _HI = urdf_limits()
_G = len(JOINTS) - 1  # gripper index


def lerobot_to_rad(reading: dict[str, float], jm: JointMap) -> np.ndarray:
    """LeRobot observation or action dict ('<joint>.pos' or '<joint>') -> joint vector in rad."""
    raw = np.array([reading.get(f"{j}.pos", reading.get(j)) for j in JOINTS], dtype=float)
    q = np.empty(len(JOINTS))
    q[:_G] = jm.sign[:_G] * np.deg2rad(raw[:_G]) + jm.offset[:_G]
    frac = raw[_G] / 100.0
    if jm.sign[_G] < 0:
        frac = 1.0 - frac
    q[_G] = _LO[_G] + frac * (_HI[_G] - _LO[_G]) + jm.offset[_G]
    return q


def rad_to_lerobot(q: np.ndarray, jm: JointMap) -> dict[str, float]:
    """Joint vector in rad -> LeRobot action dict ('<joint>.pos')."""
    q = np.asarray(q, dtype=float)
    out = np.empty(len(JOINTS))
    out[:_G] = np.rad2deg((q[:_G] - jm.offset[:_G]) / jm.sign[:_G])
    frac = (q[_G] - jm.offset[_G] - _LO[_G]) / (_HI[_G] - _LO[_G])
    if jm.sign[_G] < 0:
        frac = 1.0 - frac
    out[_G] = 100.0 * frac
    return {f"{j}.pos": float(v) for j, v in zip(JOINTS, out, strict=True)}
