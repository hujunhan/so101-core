"""Hardware-free tests: model files, unit conversion, safety layer, and SafeArm against a fake bus."""

import numpy as np
import pytest

from so101_core.arm import SafeArm, StallError
from so101_core.config import JointMap
from so101_core.convert import lerobot_to_rad, rad_to_lerobot
from so101_core.model import JOINTS, urdf_limits
from so101_core.safety import SafetyConfig, SafetyLayer
from so101_core.twin import Twin

LO, HI = urdf_limits()


def test_limits_and_mjcf_agree():
    twin = Twin()
    for i, j in enumerate(JOINTS):
        lo, hi = twin.model.joint(j).range
        assert lo == pytest.approx(LO[i], abs=1e-4) and hi == pytest.approx(HI[i], abs=1e-4), j


def test_twin_zero_pose_is_upright():
    twin = Twin()
    twin.set(np.zeros(len(JOINTS)))
    base, gripper = twin.body_pos("base"), twin.body_pos("gripper")
    assert gripper[2] > base[2] + 0.1  # zero ("middle of range") pose reaches up


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_conversion_round_trip(sign):
    jm = JointMap()
    jm.sign[:] = sign
    jm.offset[:] = 0.01
    rng = np.random.default_rng(0)
    for _ in range(100):
        q = rng.uniform(LO, HI)
        assert np.allclose(lerobot_to_rad(rad_to_lerobot(q, jm), jm), q)


def test_conversion_identity_matches_degrees():
    reading = {f"{j}.pos": 10.0 for j in JOINTS}
    reading["gripper.pos"] = 0.0
    q = lerobot_to_rad(reading, JointMap())
    assert np.allclose(q[:5], np.deg2rad(10.0))
    assert q[5] == pytest.approx(LO[5])


def test_safety_clips_and_caps_step():
    s = SafetyLayer(SafetyConfig(margin_rad=0.1, max_step_rad=0.05))
    q0 = np.zeros(len(JOINTS))
    res = s.filter(HI + 1.0, q0)
    assert np.allclose(res.q, 0.05) and res.clipped.all() and res.step_limited.all()
    far = np.zeros(len(JOINTS))
    for _ in range(1000):
        far = s.filter(HI + 1.0, far).q
    assert np.allclose(far, HI - 0.1)


def test_safety_never_pushes_outward_from_outside_start():
    s = SafetyLayer(SafetyConfig(margin_rad=0.1, max_step_rad=0.05))
    start = LO + 0.02  # inside the margin, like the folded rest pose
    assert np.allclose(s.filter(start, start).q, start)  # staying put is allowed: no forced jump
    assert np.all(s.filter(LO - 1.0, start).q >= start)  # but never further out
    assert np.all(s.filter(np.zeros(len(JOINTS)), start).q > start)  # moving inward is fine


def test_safety_rejects_nan():
    with pytest.raises(ValueError):
        SafetyLayer().filter(np.full(len(JOINTS), np.nan), np.zeros(len(JOINTS)))


class FakeBus:
    """Simulates the servos: the position follows the goal with a first-order lag."""

    def __init__(self, jm, blocked_joint=None):
        self.jm = jm
        self.q = np.zeros(len(JOINTS))
        self.goal = self.q.copy()
        self.blocked = blocked_joint
        self.is_connected = False
        self.is_calibrated = True
        self.torque = False
        self.sent = []

    def connect(self):
        self.is_connected = True

    def disconnect(self, disable_torque=True):
        self.is_connected = False

    def enable_torque(self):
        self.torque = True

    def disable_torque(self):
        self.torque = False

    def sync_read(self, name, normalize=True, num_retry=0):
        assert name == "Present_Position"
        if self.torque:
            step = 0.5 * (self.goal - self.q)
            if self.blocked is not None:
                step[self.blocked] = 0.0
            self.q = self.q + step
        d = rad_to_lerobot(self.q, self.jm)
        return {k.removesuffix(".pos"): v for k, v in d.items()}

    def sync_write(self, name, values, normalize=True):
        assert name == "Goal_Position"
        if normalize:
            self.goal = lerobot_to_rad(values, self.jm)
            self.sent.append(self.goal.copy())


class FakeRobot:
    def __init__(self, bus):
        self.bus = bus

    def configure(self):
        self.bus.enable_torque()


def make_arm(blocked=None):
    jm = JointMap()
    bus = FakeBus(jm, blocked)
    cfg = SafetyConfig(rate_hz=1000.0)  # run fast in tests
    arm = SafeArm(safety=cfg, joint_map=jm, robot=FakeRobot(bus), confirm_release=lambda: None)
    return arm, bus


def test_move_to_reaches_goal_within_limits_and_step():
    arm, bus = make_arm()
    arm.connect(torque=True)
    goal = np.deg2rad([30, -40, 50, -20, 60, 30])
    q = arm.move_to(goal, speed_rad_s=np.deg2rad(2000))
    assert np.allclose(q, goal, atol=np.deg2rad(2))
    sent = np.array(bus.sent)
    assert np.all(np.abs(np.diff(sent, axis=0)) <= arm.safety.cfg.max_step_rad + 1e-9)
    arm.disconnect()
    assert not bus.torque


def test_move_to_stops_on_stall():
    arm, bus = make_arm(blocked=1)
    arm.connect(torque=True)
    with pytest.raises(StallError, match="shoulder_lift"):
        arm.move_to(np.deg2rad([0, 60, 0, 0, 0, 0]), speed_rad_s=np.deg2rad(2000), stall_s=0.01)
    assert np.allclose(bus.goal, bus.q)  # holding where it is, not pushing on


def test_exception_holds_then_releases():
    arm, bus = make_arm()
    with pytest.raises(KeyboardInterrupt), arm.connect(torque=True):
        arm.command(np.deg2rad([1, 1, 1, 1, 1, 1]))
        raise KeyboardInterrupt
    assert not bus.torque and not bus.is_connected
