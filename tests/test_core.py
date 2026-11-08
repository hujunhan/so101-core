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

    def __init__(self, jm, blocked_joint=None, sag=None):
        self.jm = jm
        self.sag = np.zeros(len(JOINTS)) if sag is None else sag  # steady-state error under gravity
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
            step = 0.5 * (self.goal - self.sag - self.q)
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


def make_arm(blocked=None, sag=None):
    jm = JointMap()
    bus = FakeBus(jm, blocked, sag)
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


def test_move_to_returns_promptly_despite_gravity_sag():
    """Regression: with a steady-state error > tolerance, move_to used to wait for the timeout."""
    import time

    sag = np.zeros(len(JOINTS))
    sag[2] = np.deg2rad(3.0)  # elbow sags 3 deg, like the real arm at the zero pose
    arm, bus = make_arm(sag=sag)
    arm.connect(torque=True)
    goal = np.deg2rad([0, 0, 30, 0, 0, 0])
    t0 = time.perf_counter()
    q = arm.move_to(goal, speed_rad_s=np.deg2rad(2000), settle_s=0.02, settle_timeout_s=5.0)
    assert time.perf_counter() - t0 < 1.0
    assert np.rad2deg(goal[2] - q[2]) == pytest.approx(3.0, abs=0.2)  # the residual is reported, not hidden


def _line(q_from, q_to, n):
    return np.linspace(q_from, q_to, n)


def test_follow_tracks_a_path_and_reports_what_the_filter_did():
    arm, bus = make_arm()
    arm.connect(torque=True)
    path = _line(
        np.zeros(6), np.deg2rad([10, -10, 10, 5, 0, 0]), 40
    )  # 0.25 deg per tick: inside the step cap
    res = arm.follow(path)
    assert res.q_cmd.shape == res.q_meas.shape == path.shape and len(res.t) == len(path)
    assert np.allclose(res.q_cmd, path) and res.clipped == 0 and res.step_limited == 0
    assert np.allclose(res.q_meas[-1], path[-1], atol=np.deg2rad(0.5))  # the fake servo catches up
    assert np.all(np.diff(res.t) > 0)


def test_follow_counts_clipped_and_step_limited_ticks():
    arm, bus = make_arm()
    arm.connect(torque=True)
    fast = _line(np.zeros(6), np.deg2rad([0, 0, 20, 0, 0, 0]), 5)  # 5 deg per tick, cap is 2
    assert arm.follow(fast).step_limited > 0
    arm2, _ = make_arm()
    arm2.connect(torque=True)
    out = np.tile(arm2.q_cmd, (4, 1))
    out[1:, 1] = np.deg2rad(110)  # beyond the shoulder_lift limit (the first row stays at the command)
    res = arm2.follow(out)
    assert res.clipped == 3
    assert np.all(res.q_cmd[:, 1] <= arm2.safety.hi[1] + 1e-9)


def test_follow_stops_on_stall():
    arm, bus = make_arm(blocked=1)
    arm.connect(torque=True)
    path = _line(np.zeros(6), np.deg2rad([0, 60, 0, 0, 0, 0]), 200)  # 0.3 deg per tick
    with pytest.raises(StallError, match="shoulder_lift"):
        arm.follow(path, stall_s=0.01)
    assert np.allclose(bus.goal, bus.q)  # holding where it is, not pushing on


def test_follow_tolerates_gravity_sag_below_the_stall_threshold():
    sag = np.zeros(len(JOINTS))
    sag[2] = np.deg2rad(3.0)
    arm, bus = make_arm(sag=sag)
    arm.connect(torque=True)
    start = arm.q_cmd.copy()  # the fake servo has already sagged a little by the time connect() returns
    res = arm.follow(_line(start, start + np.deg2rad([0, 0, 20, 0, 0, 0]), 60))
    residual = np.rad2deg(res.q_cmd[-1, 2] - res.q_meas[-1, 2])
    # sag (3 deg) plus the fake servo's lag of about one tick (0.33 deg) while the path is still moving
    assert residual == pytest.approx(3.33, abs=0.15)  # reported, not hidden; no false stall


def test_follow_refuses_a_path_that_starts_far_from_the_current_command():
    arm, _ = make_arm()
    arm.connect(torque=True)
    with pytest.raises(ValueError, match="move_to its start"):
        arm.follow(_line(np.deg2rad([20, 0, 0, 0, 0, 0]), np.deg2rad([30, 0, 0, 0, 0, 0]), 10))
    with pytest.raises(ValueError, match="shape"):
        arm.follow(np.zeros((10, 5)))


def test_follow_dense_reads_between_commands_without_changing_the_commands():
    arm, bus = make_arm()
    arm.safety.cfg.rate_hz = 200.0  # 5 ms per tick: room for many fake reads
    arm.connect(torque=True)
    path = _line(np.zeros(6), np.deg2rad([0, 0, 5, 0, 0, 0]), 20)
    res = arm.follow(path, dense=True)
    assert np.allclose(res.q_cmd, path) and len(res.t_sent) == len(path)
    assert len(res.t_dense) == len(res.q_dense) > 2 * len(path)  # more than one read per command
    assert np.all(np.diff(res.t_dense) > 0)
    assert np.all(res.t_sent < res.t)  # each command goes out before its first read
    assert res.t_dense[-1] < res.t_sent[-1] + 2 / arm.safety.cfg.rate_hz  # reads stop with the last tick
    assert arm.follow(path[-1:]).t_dense is None  # off by default


def test_position_p_reaches_the_lerobot_config(monkeypatch):
    lerobot = pytest.importorskip("lerobot.robots.so_follower")
    seen = {}
    monkeypatch.setattr(lerobot, "SO101Follower", lambda cfg: seen.update(cfg=cfg))
    from so101_core.config import DeviceConfig

    devices = DeviceConfig(follower_port="/dev/null", leader_port="/dev/null", robot_id="x", teleop_id="x")
    SafeArm(devices, joint_map=JointMap(), position_p=32)._build_robot()
    assert seen["cfg"].position_p_coefficient == 32
    SafeArm(devices, joint_map=JointMap())._build_robot()
    assert seen["cfg"].position_p_coefficient == 16  # LeRobot's default is kept


def test_offset_limits_keep_readings_in_range():
    from so101_core.config import JointMap

    jm = JointMap()
    jm.offset[2] = np.deg2rad(-7.3)
    lo, hi = jm.model_limits(LO, HI)
    assert lo[2] == LO[2] and np.isclose(hi[2], HI[2] - np.deg2rad(7.3))
    # readings at the new limits stay inside the URDF range
    assert (hi[2] - jm.offset[2]) <= HI[2] + 1e-12
    assert np.allclose(np.delete(lo, 2), np.delete(LO, 2)) and np.allclose(np.delete(hi, 2), np.delete(HI, 2))
