"""Robot description: joint names, model files and joint limits."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ARM_JOINTS: tuple[str, ...] = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
GRIPPER = "gripper"
JOINTS: tuple[str, ...] = ARM_JOINTS + (GRIPPER,)

ASSET_DIR = Path(__file__).parent / "assets" / "so101"
URDF_PATH = ASSET_DIR / "so101_new_calib.urdf"
MJCF_PATH = ASSET_DIR / "so101_new_calib.xml"
SCENE_PATH = ASSET_DIR / "scene.xml"


def urdf_limits(path: Path = URDF_PATH) -> tuple[np.ndarray, np.ndarray]:
    """Lower and upper position limits (rad) of `JOINTS`, read from the URDF."""
    root = ET.parse(path).getroot()
    found: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):  # top-level joints only; <transmission> also nests <joint>
        limit = joint.find("limit")
        if limit is not None and joint.attrib.get("type") == "revolute":
            found[joint.attrib["name"]] = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
    missing = [j for j in JOINTS if j not in found]
    if missing:
        raise ValueError(f"URDF {path} has no limits for {missing}")
    lo = np.array([found[j][0] for j in JOINTS])
    hi = np.array([found[j][1] for j in JOINTS])
    return lo, hi
