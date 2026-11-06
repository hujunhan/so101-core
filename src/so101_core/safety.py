"""Safety layer: every command to the real arm passes through `SafetyLayer.filter`.

It protects against two failure modes seen in practice:
- targets outside the joint limits (e.g. an unconstrained IK solution), which would drive a
  joint into its calibrated end stop and stall the servo;
- large jumps between consecutive commands, which the servo would execute at full speed.

It does not protect against collisions with the table, objects or the arm itself.
"""

from dataclasses import dataclass, field

import numpy as np

from so101_core.model import urdf_limits


@dataclass
class SafetyConfig:
    margin_rad: float = np.deg2rad(5.0)  # stay this far inside the URDF limits
    max_step_rad: float = np.deg2rad(2.0)  # largest change per command, per joint
    rate_hz: float = 30.0  # command rate; max speed = max_step_rad * rate_hz (60 deg/s by default)


@dataclass
class FilterResult:
    q: np.ndarray
    clipped: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))  # hit a limit
    step_limited: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))  # step was capped


class SafetyLayer:
    def __init__(self, cfg: SafetyConfig | None = None, limits: tuple[np.ndarray, np.ndarray] | None = None):
        self.cfg = cfg or SafetyConfig()
        lo, hi = limits if limits is not None else urdf_limits()
        self.lo = lo + self.cfg.margin_rad
        self.hi = hi - self.cfg.margin_rad
        if np.any(self.lo >= self.hi):
            raise ValueError("margin_rad leaves no usable range for some joint")

    def clip(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        qc = np.clip(q, self.lo, self.hi)
        return qc, qc != q

    def filter(self, q_target: np.ndarray, q_from: np.ndarray) -> FilterResult:
        """Clip the target to the limits, then cap the step from `q_from` (the last command).

        If `q_from` is already outside the safe range (e.g. the arm starts in its folded rest
        pose, inside the margin), the target may stay where it is or move inward, but never
        further out. This avoids a forced jump on the first command.
        """
        q_target = np.asarray(q_target, dtype=float)
        if not np.all(np.isfinite(q_target)):
            raise ValueError(f"non-finite joint target: {q_target}")
        lo = np.minimum(self.lo, q_from)
        hi = np.maximum(self.hi, q_from)
        qc = np.clip(q_target, lo, hi)
        clipped = qc != q_target
        step = qc - q_from
        capped = np.clip(step, -self.cfg.max_step_rad, self.cfg.max_step_rad)
        return FilterResult(q=q_from + capped, clipped=clipped, step_limited=capped != step)
