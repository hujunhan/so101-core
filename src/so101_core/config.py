"""Device and per-robot configuration.

`config/devices.yaml` is machine-specific and git-ignored (it holds adapter serial numbers);
`config/devices.example.yaml` documents the format. Lookup order: the `SO101_CONFIG_DIR`
environment variable, then `./config`, then `~/.config/so101`.
"""

import os
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path

import numpy as np
import yaml

from so101_core.model import JOINTS


def config_dir() -> Path:
    candidates = [os.environ.get("SO101_CONFIG_DIR"), "config", Path.home() / ".config" / "so101"]
    for c in candidates:
        if c and (Path(c) / "devices.yaml").exists():
            return Path(c)
    raise FileNotFoundError(
        "No devices.yaml found. Copy config/devices.example.yaml to config/devices.yaml and fill it in."
    )


@dataclass
class DeviceConfig:
    follower_port: str
    leader_port: str | None = None
    robot_id: str = "so101"  # LeRobot calibration file name for the follower
    teleop_id: str = "so101"  # LeRobot calibration file name for the leader
    cameras: dict[str, str] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path | None = None) -> "DeviceConfig":
        path = path or config_dir() / "devices.yaml"
        data = yaml.safe_load(Path(path).read_text())
        return cls(**data)


@dataclass
class JointMap:
    """Maps LeRobot readings to model joint angles: q = sign * reading_rad + offset.

    Defaults are identity. The joint check (`so101-jointcheck`) confirms or corrects the signs.
    """

    sign: np.ndarray = field(default_factory=lambda: np.ones(len(JOINTS)))
    offset: np.ndarray = field(default_factory=lambda: np.zeros(len(JOINTS)))
    rest: np.ndarray | None = None  # rest pose (rad), recorded on the real arm
    verified: str | None = None  # date of the last joint check

    @classmethod
    def load(cls, path: Path | None = None) -> "JointMap":
        if path is None:
            try:
                path = config_dir() / "joint_map.yaml"
            except FileNotFoundError:
                return cls()
        if not Path(path).exists():
            return cls()
        d = yaml.safe_load(Path(path).read_text()) or {}
        jm = cls()
        for i, j in enumerate(JOINTS):
            entry = d.get("joints", {}).get(j, {})
            jm.sign[i] = float(entry.get("sign", 1.0))
            jm.offset[i] = float(entry.get("offset_rad", 0.0))
        if d.get("rest_rad"):
            jm.rest = np.array([d["rest_rad"][j] for j in JOINTS], dtype=float)
        jm.verified = d.get("verified")
        return jm

    def model_limits(self, lo: np.ndarray, hi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Arm joint limits in model angles such that the readings stay inside [lo, hi] as well.

        The URDF limits fit LeRobot's readings (0 = the middle of each joint's calibrated range). With a zero
        offset, the model angle q = sign * reading + offset can be inside them while the reading is not (the
        joint's hard stop): an elbow offset of -7.3 deg would let the reading reach 99 deg against a 96.6 deg
        range. So the limits are the intersection of both. The gripper keeps its own linear map.
        """
        lo, hi = np.array(lo, float), np.array(hi, float)
        a, b = self.sign * lo + self.offset, self.sign * hi + self.offset
        out_lo, out_hi = lo.copy(), hi.copy()
        arm = slice(0, len(JOINTS) - 1)
        out_lo[arm] = np.maximum(lo[arm], np.minimum(a, b)[arm])
        out_hi[arm] = np.minimum(hi[arm], np.maximum(a, b)[arm])
        return out_lo, out_hi

    def save(self, path: Path, verified: bool = False) -> None:
        if verified:
            self.verified = date.today().isoformat()
        d = {
            "verified": self.verified,
            "joints": {
                j: {"sign": int(self.sign[i]), "offset_rad": round(float(self.offset[i]), 6)}
                for i, j in enumerate(JOINTS)
            },
        }
        if self.rest is not None:
            d["rest_rad"] = {j: round(float(self.rest[i]), 4) for i, j in enumerate(JOINTS)}
        Path(path).write_text(yaml.safe_dump(d, sort_keys=False))
