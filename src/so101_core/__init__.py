"""Shared foundation for SO-101 arm projects.

Conventions (see README):
- SI units and radians inside; LeRobot degrees only at the hardware boundary (`convert`).
- Joint vectors are numpy arrays ordered as `model.JOINTS`.
- Every command to the real arm goes through `safety.SafetyLayer`.
"""

from so101_core.model import ARM_JOINTS, GRIPPER, JOINTS

__all__ = ["ARM_JOINTS", "GRIPPER", "JOINTS"]
