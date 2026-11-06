"""MuJoCo digital twin: shows a joint vector on the SO-101 model (kinematic only, no physics)."""

import mujoco
import numpy as np

from so101_core.model import JOINTS, SCENE_PATH


class Twin:
    def __init__(self, scene_path=SCENE_PATH):
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        self.qadr = np.array([self.model.joint(j).qposadr[0] for j in JOINTS])
        self.viewer = None

    def set(self, q: np.ndarray) -> None:
        self.data.qpos[self.qadr] = q
        mujoco.mj_forward(self.model, self.data)

    def body_pos(self, name: str) -> np.ndarray:
        return self.data.body(name).xpos.copy()

    def open_viewer(self) -> "Twin":
        import mujoco.viewer

        self.viewer = mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False)
        return self

    def show(self, q: np.ndarray) -> bool:
        """Set the pose and refresh the viewer. Returns False once the window is closed."""
        self.set(q)
        if self.viewer is None:
            return True
        if not self.viewer.is_running():
            return False
        self.viewer.sync()
        return True

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
