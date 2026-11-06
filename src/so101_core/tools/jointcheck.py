"""so101-jointcheck: verify the joint mapping between the real arm and the model.

Phase A (torque OFF, nothing moves by itself):
  for each joint, you move it by hand and say whether the twin turns the same way.
  A "no" flips that joint's sign. Then you put the arm in its folded rest pose and it is recorded.
Phase B (torque ON, the arm moves; optional):
  each joint moves +5 deg and back, slowly, through the safety layer, and you confirm the real
  arm and the twin agree. Optionally the arm goes to the zero pose (middle of every range) so
  you can compare it with the twin by eye. It ends back at rest, then torque is released after
  you confirm you are supporting the arm.

Results go to config/joint_map.yaml (used by everything else) and config/jointcheck_<date>.yaml.
"""

import argparse
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from so101_core.arm import Rate, SafeArm
from so101_core.config import DeviceConfig, JointMap, config_dir
from so101_core.model import ARM_JOINTS, JOINTS
from so101_core.twin import Twin

SAFETY_BRIEF = """
Before Phase B (powered motion):
  - the arm base is clamped to the table
  - nothing within reach of the arm (about 45 cm)
  - your hand is near the arm's power plug; pulling it is the emergency stop
  - Ctrl-C at any time holds the current position
"""


def ask(prompt: str, choices: str = "yn") -> str:
    while True:
        a = input(f"{prompt} [{'/'.join(choices)}] ").strip().lower()
        if a in choices:
            return a


class LiveTwin:
    """Updates the twin from a reader function in a background thread while input() blocks."""

    def __init__(self, twin: Twin, read):
        self.twin, self.read, self.stop = twin, read, threading.Event()
        self.q = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        rate = Rate(30)
        while not self.stop.is_set():
            self.q = self.read()
            self.twin.show(self.q)
            rate.sleep()

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.thread.join()


def phase_a(arm: SafeArm, twin: Twin, log: dict) -> None:
    print("\n== Phase A: torque OFF. Move each joint by hand and watch the twin.")
    with LiveTwin(twin, arm.read) as live:
        for i, j in enumerate(JOINTS):
            a = ask(f"  {j}: does the twin turn the same way as the real joint? (s = skip)", "yns")
            log["phase_a"][j] = a
            if a == "n":
                arm.jm.sign[i] *= -1
                print(f"    -> flipped the sign of {j}; move it again to confirm.")
                a2 = ask(f"  {j}: same way now?", "yn")
                log["phase_a"][j] += f",{a2}"
                if a2 == "n":
                    raise SystemExit(f"{j} still disagrees after flipping: stop and investigate.")
        input("\n  Put the arm in its folded REST pose (as for power-off), then press Enter... ")
        time.sleep(0.2)
        arm.jm.rest = live.q.copy()
        print("  rest pose [deg]: " + ", ".join(f"{np.rad2deg(v):.1f}" for v in arm.jm.rest))


def phase_b(arm: SafeArm, twin: Twin, log: dict) -> None:
    print(SAFETY_BRIEF)
    if ask("Start Phase B (the arm will move)?") != "y":
        return
    arm.connect(torque=True)
    tick = lambda q, q_cmd: twin.show(q)  # noqa: E731
    start = arm.read()
    print("  Moving to a safe start pose: shoulder_lift and elbow_flex 20 deg toward mid-range...")
    lifted = start.copy()
    for i in (1, 2):  # rest is near the ends of these ranges; mid-range (0) is always inward
        lifted[i] -= np.sign(start[i]) * min(np.deg2rad(20), abs(start[i]))
    arm.move_to(lifted, on_tick=tick)
    # Baseline = what we commanded, not what we measured: commanding the measured (sagged) pose
    # would make the arm sag again by the same amount, and so on.
    base = arm.q_cmd.copy()
    for i, j in enumerate(JOINTS):
        delta = np.deg2rad(10 if j == "gripper" else 5)
        input(f"\n  {j}: press Enter to move +{np.rad2deg(delta):.0f} deg and back... ")
        target = base.copy()
        target[i] += delta
        t0 = time.perf_counter()
        q = arm.move_to(target, speed_rad_s=np.deg2rad(10), on_tick=tick)
        dt = time.perf_counter() - t0
        err = np.rad2deg(q[i] - target[i])
        print(f"    settled in {dt:.1f} s; measured - commanded: {err:+.2f} deg")
        a = ask(f"  {j}: did the real joint and the twin move the same way?")
        arm.move_to(base, speed_rad_s=np.deg2rad(10), on_tick=tick)
        log["phase_b"][j] = {
            "agree": a,
            "residual_deg": round(float(err), 2),  # measured - commanded after settling
            "move_s": round(dt, 2),
        }
        if a == "n":
            raise SystemExit(f"{j}: command path disagrees with the twin. Stopping (holding position).")
    if ask("\n  Optional: move to the ZERO pose (middle of every range) to compare with the twin?") == "y":
        zero = np.zeros(len(JOINTS))
        zero[-1] = base[-1]  # leave the gripper as it is
        arm.move_to(zero, on_tick=tick)
        twin.show(zero)  # twin shows the commanded pose, so differences are visible
        q = arm.read()
        resid = {j: round(float(np.rad2deg(v)), 2) for j, v in zip(ARM_JOINTS, q[:5], strict=True)}
        print(f"  measured - commanded [deg]: {resid}")
        log["zero_pose_residual_deg"] = resid
        log["zero_pose_note"] = input("  Describe any visible difference from the twin (or Enter): ")
        arm.move_to(base, on_tick=tick)
    print("  Returning to rest...")
    arm.move_to(arm.jm.rest, on_tick=tick)


def main() -> None:
    argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    ).parse_args()
    cdir = config_dir()
    log = {"date": datetime.now().isoformat(timespec="seconds"), "phase_a": {}, "phase_b": {}}
    twin = Twin().open_viewer()
    arm = SafeArm(DeviceConfig.load(), joint_map=JointMap.load())
    try:
        arm.connect(torque=False)
        phase_a(arm, twin, log)
        arm.jm.save(cdir / "joint_map.yaml")  # save signs and rest pose before anything moves
        arm.disconnect()
        arm = SafeArm(DeviceConfig.load(), joint_map=JointMap.load())
        with arm:
            phase_b(arm, twin, log)
        arm.jm.save(cdir / "joint_map.yaml", verified=bool(log["phase_b"]))
    finally:
        log["signs"] = {j: int(s) for j, s in zip(JOINTS, arm.jm.sign, strict=True)}
        Path(cdir / f"jointcheck_{datetime.now():%Y%m%d_%H%M}.yaml").write_text(
            yaml.safe_dump(log, sort_keys=False)
        )
        twin.close()
    print("\nDone. Joint map saved to", cdir / "joint_map.yaml")


if __name__ == "__main__":
    main()
