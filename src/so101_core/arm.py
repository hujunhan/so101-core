"""Safe access to the real SO-101 follower arm.

Differences from calling LeRobot's `SOFollower` directly:
- `connect()` writes Goal_Position = Present_Position *before* torque is enabled, so the arm
  cannot jump to a stale goal on power-up (LeRobot's `connect()` enables torque without this).
- It never runs LeRobot calibration: a missing or mismatched calibration is an error.
- Every command goes through `SafetyLayer` (joint-limit clipping and a per-command step cap).
- `move_to()` watches the following error. If the measured position lags the command by more
  than `stall_rad` for longer than `stall_s`, the arm is probably blocked (table, object or
  itself). It then holds the current position and raises `StallError` instead of pushing on.
- On any exception, including Ctrl-C, the context manager holds the current position, then
  releases torque only after the user confirms the arm is supported.
"""

import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from so101_core.config import DeviceConfig, JointMap
from so101_core.convert import lerobot_to_rad, rad_to_lerobot
from so101_core.model import JOINTS
from so101_core.safety import SafetyConfig, SafetyLayer


class StallError(RuntimeError):
    pass


@dataclass
class FollowResult:
    """What `SafeArm.follow` did. All arrays have one row per command."""

    t: np.ndarray  # (n,) seconds since the first command
    q_cmd: np.ndarray  # (n, 6) what was sent, after the safety filter
    q_meas: np.ndarray  # (n, 6) measured right after each command
    clipped: int  # commands where the filter clipped a target to the limits
    step_limited: int  # commands where the filter capped the step (the arm lags the path)
    t_sent: np.ndarray | None = None  # (n,) seconds since t0, just before each command was written
    # With dense=True: every read, including the extra ones between commands. t_dense is the midpoint
    # of each read (a read takes about 1 ms on the real bus), q_dense the measured q.
    t_dense: np.ndarray | None = None  # (m,)
    q_dense: np.ndarray | None = None  # (m, 6)


class Rate:
    def __init__(self, hz: float):
        self.dt = 1.0 / hz
        self.next = time.perf_counter()

    def sleep(self) -> None:
        self.next += self.dt
        delay = self.next - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        else:  # fell behind; don't try to catch up with a burst
            self.next = time.perf_counter()


def _confirm_release() -> None:
    try:
        input("\n>>> Support the arm with your hand, then press Enter to release torque... ")
    except EOFError:
        pass


class SafeArm:
    def __init__(
        self,
        devices: DeviceConfig | None = None,
        safety: SafetyConfig | None = None,
        joint_map: JointMap | None = None,
        robot=None,  # injected for tests; otherwise built from `devices` on connect()
        confirm_release: Callable[[], None] = _confirm_release,
        position_p: int
        | None = None,  # servo position gain; None keeps LeRobot's (16; the factory value is 32)
    ):
        self.devices = devices
        self.safety = SafetyLayer(safety)
        self.jm = joint_map or JointMap.load()
        self.robot = robot
        self.confirm_release = confirm_release
        self.position_p = position_p
        self.torque = False
        self.q_cmd: np.ndarray | None = None  # last commanded position (rad)
        self.last_filter = (
            None  # FilterResult of the last command(): which joints were clipped or step-capped
        )

    # -- connection -------------------------------------------------------------------------
    def _build_robot(self):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        gains = {} if self.position_p is None else {"position_p_coefficient": int(self.position_p)}
        cfg = SO101FollowerConfig(port=self.devices.follower_port, id=self.devices.robot_id, **gains)
        return SO101Follower(cfg)

    def connect(self, torque: bool) -> "SafeArm":
        """Connect. `torque=False` leaves the arm limp (to move it by hand)."""
        if self.robot is None:
            self.robot = self._build_robot()
        bus = self.robot.bus
        bus.connect()
        if not bus.is_calibrated:
            bus.disconnect(disable_torque=False)
            raise RuntimeError(
                "Motor calibration does not match the LeRobot calibration file. "
                "Recalibrate with LeRobot/LeLab first; so101-core never calibrates on its own."
            )
        if torque:
            self._sync_goal_to_present()
            self.robot.configure()  # writes PID and protection settings, then enables torque
            self._sync_goal_to_present()
            self.torque = True
            self.q_cmd = self.read()
        else:
            bus.disable_torque()
            self.torque = False
        return self

    def _sync_goal_to_present(self) -> None:
        bus = self.robot.bus
        present = bus.sync_read("Present_Position", normalize=False, num_retry=3)
        bus.sync_write("Goal_Position", present, normalize=False)

    def release(self, confirm: bool = True) -> None:
        """Disable torque. The arm goes limp, so by default ask the user to support it first."""
        if self.robot is None or not self.torque:
            return
        if confirm:
            self.confirm_release()
        self.robot.bus.disable_torque()
        self.torque = False

    def disconnect(self) -> None:
        if self.robot is not None and self.robot.bus.is_connected:
            self.release()
            self.robot.bus.disconnect(disable_torque=False)

    def __enter__(self) -> "SafeArm":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if exc_type is not None and self.torque:
            try:
                self.hold()
                print(f"\n[so101-core] {exc_type.__name__}: holding current position.")
            except Exception as e:  # bus may be gone; nothing more we can do safely
                print(f"[so101-core] could not hold position: {e}")
        self.disconnect()

    # -- state and commands -----------------------------------------------------------------
    def read(self) -> np.ndarray:
        """Measured joint positions (rad), ordered as JOINTS."""
        return lerobot_to_rad(self.robot.bus.sync_read("Present_Position", num_retry=3), self.jm)

    def hold(self) -> np.ndarray:
        q = self.read()
        self._send(q)
        self.q_cmd = q
        return q

    def _send(self, q: np.ndarray) -> None:
        action = rad_to_lerobot(q, self.jm)
        self.robot.bus.sync_write("Goal_Position", {k.removesuffix(".pos"): v for k, v in action.items()})

    def command(self, q_target: np.ndarray) -> np.ndarray:
        """Send one safety-filtered command. Call at `safety.cfg.rate_hz`. Returns what was sent."""
        if not self.torque:
            raise RuntimeError("torque is off; connect(torque=True) first")
        res = self.safety.filter(q_target, self.q_cmd)
        self._send(res.q)
        self.q_cmd = res.q
        self.last_filter = res
        return res.q

    def move_to(
        self,
        q_goal: np.ndarray,
        speed_rad_s: float = np.deg2rad(20.0),
        timeout_s: float = 15.0,
        settle_rad: float = np.deg2rad(0.3),
        settle_s: float = 0.2,
        settle_timeout_s: float = 2.0,
        stall_rad: float = np.deg2rad(15.0),
        stall_s: float = 0.5,
        on_tick: Callable[[np.ndarray, np.ndarray], None] | None = None,
    ) -> np.ndarray:
        """Ramp the command to `q_goal` at `speed_rad_s` (capped by the safety layer), then wait
        for the arm to settle. Returns the settled measured q.

        "Settled" means no joint moved more than `settle_rad` over `settle_s`; it does not mean
        "at the goal". Position-controlled servos under gravity keep a steady-state error (sag),
        so waiting for |q - goal| < tol can wait forever. Callers measure the residual
        (q - goal) themselves. `settle_timeout_s` caps the wait after the ramp ends.

        `on_tick(q_measured, q_commanded)` is called every cycle (e.g. to update the twin).
        """
        if not self.torque:
            raise RuntimeError("torque is off; connect(torque=True) first")
        hz = self.safety.cfg.rate_hz
        step = min(speed_rad_s / hz, self.safety.cfg.max_step_rad)
        goal, _ = self.safety.clip(np.asarray(q_goal, dtype=float))
        rate = Rate(hz)
        t0 = time.perf_counter()
        lag_since = None
        ramp_done_at = None
        history: list[tuple[float, np.ndarray]] = []
        while True:
            delta = np.clip(goal - self.q_cmd, -step, step)
            self.command(self.q_cmd + delta)
            q = self.read()
            if on_tick:
                on_tick(q, self.q_cmd)
            now = time.perf_counter()
            if np.max(np.abs(q - self.q_cmd)) > stall_rad:
                lag_since = lag_since or now
                if now - lag_since > stall_s:
                    worst = JOINTS[int(np.argmax(np.abs(q - self.q_cmd)))]  # before hold() resets q_cmd
                    self.hold()
                    raise StallError(f"{worst} is not following the command: blocked? Holding position.")
            else:
                lag_since = None
            if np.allclose(self.q_cmd, goal):
                ramp_done_at = ramp_done_at or now
                history.append((now, q))
                while history and now - history[0][0] > settle_s:
                    history.pop(0)
                window = np.array([h[1] for h in history])
                spread = np.max(np.ptp(window, axis=0)) if len(history) > 1 else np.inf
                if now - ramp_done_at >= settle_s and spread < settle_rad:
                    return q
                if now - ramp_done_at > settle_timeout_s:
                    return q  # still creeping; the caller sees the residual
            elif now - t0 > timeout_s:
                self.hold()
                raise TimeoutError("move_to could not finish the ramp in time; holding position")
            rate.sleep()

    def follow(
        self,
        q_path: np.ndarray,
        stall_rad: float = np.deg2rad(5.0),
        stall_s: float = 0.3,
        on_tick: Callable[[int, np.ndarray, np.ndarray], None] | None = None,
        dense: bool = False,
    ) -> FollowResult:
        """Send one command per row of `q_path` at the safety rate, reading the arm after each.

        For trajectories (drawing, replay), where `move_to`'s ramp and settling do not fit. Every command
        goes through the safety layer; the result counts the ticks where it clipped or capped the path,
        so a path that the layer had to change is visible. If the measured position lags the command by
        more than `stall_rad` for longer than `stall_s`, the arm is probably blocked: it holds and raises
        `StallError`. The default (5 deg, 0.3 s) is tighter than `move_to`'s, because a slow trajectory
        should not drift far from its command.

        `q_path` must start within one step cap of the current command (`move_to` its first row first);
        otherwise the first commands would silently change the start of the path.
        `on_tick(i, q_measured, q_commanded)` is called every cycle, e.g. to log.

        `dense=True` also reads the arm repeatedly between commands, until the next command is due, and
        returns every read with its time (`t_dense`, `q_dense`), for system identification at a finer time
        resolution than the command rate. The commands, the safety filter and the stall watch are the same.
        """
        if not self.torque:
            raise RuntimeError("torque is off; connect(torque=True) first")
        q_path = np.asarray(q_path, dtype=float)
        if q_path.ndim != 2 or q_path.shape[1] != len(JOINTS):
            raise ValueError(f"q_path must have shape (n, {len(JOINTS)}), got {q_path.shape}")
        if np.max(np.abs(q_path[0] - self.q_cmd)) > self.safety.cfg.max_step_rad:
            raise ValueError(
                "q_path starts more than one step cap from the current command; move_to its start first"
            )
        rate = Rate(self.safety.cfg.rate_hz)
        t0 = time.perf_counter()
        t, t_sent, cmd, meas = [], [], [], []
        t_dense, q_dense = [], []
        read_s = 0.0  # longest recent read, to stop dense reads before the next command is due
        clipped = step_limited = 0
        lag_since = None
        for i, q_target in enumerate(q_path):
            t_sent.append(time.perf_counter() - t0)
            q_sent = self.command(q_target)
            clipped += bool(self.last_filter.clipped.any())
            step_limited += bool(self.last_filter.step_limited.any())
            before = time.perf_counter()
            q = self.read()
            now = time.perf_counter()
            t.append(now - t0)
            cmd.append(q_sent)
            meas.append(q)
            if dense:
                read_s = max(0.9 * read_s, now - before)
                t_dense.append((before + now) / 2 - t0)
                q_dense.append(q)
                due = rate.next + rate.dt  # when rate.sleep() will send the next command
                while time.perf_counter() + 2 * read_s < due:
                    before = time.perf_counter()
                    q_dense.append(self.read())
                    after = time.perf_counter()
                    t_dense.append((before + after) / 2 - t0)
                    read_s = max(0.9 * read_s, after - before)
            if on_tick:
                on_tick(i, q, q_sent)
            if np.max(np.abs(q - q_sent)) > stall_rad:
                lag_since = lag_since or now
                if now - lag_since > stall_s:
                    worst = JOINTS[int(np.argmax(np.abs(q - q_sent)))]  # before hold() resets q_cmd
                    self.hold()
                    raise StallError(
                        f"{worst} is not following the path (tick {i}): blocked? Holding position."
                    )
            else:
                lag_since = None
            rate.sleep()
        return FollowResult(
            np.array(t),
            np.array(cmd),
            np.array(meas),
            clipped,
            step_limited,
            t_sent=np.array(t_sent),
            t_dense=np.array(t_dense) if dense else None,
            q_dense=np.array(q_dense) if dense else None,
        )
