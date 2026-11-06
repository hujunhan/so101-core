"""so101-check: read-only health check of devices and joint state.

Nothing is written to the motors and torque is left as it is.
"""

import argparse
from pathlib import Path

import numpy as np

from so101_core.config import DeviceConfig, JointMap
from so101_core.convert import lerobot_to_rad
from so101_core.model import JOINTS, urdf_limits


def main() -> None:
    argparse.ArgumentParser(description=__doc__).parse_args()
    dev = DeviceConfig.load()
    jm = JointMap.load()
    ok = True

    print("== Devices")
    paths = {"follower": dev.follower_port, "leader": dev.leader_port, **dev.cameras}
    for name, p in paths.items():
        exists = bool(p) and Path(p).exists()
        ok &= exists
        print(f"  {'OK ' if exists else 'MISSING'} {name:10s} {p}")

    print("\n== Follower joints (read-only)")
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    robot = SO101Follower(SO101FollowerConfig(port=dev.follower_port, id=dev.robot_id))
    bus = robot.bus
    bus.connect()
    try:
        calibrated = bus.is_calibrated
        ok &= calibrated
        print(f"  calibration matches LeRobot file: {calibrated}")
        raw = bus.sync_read("Present_Position", normalize=False, num_retry=3)
        reading = bus.sync_read("Present_Position", num_retry=3)
        torque = bus.sync_read("Torque_Enable", normalize=False, num_retry=3)
        temp = bus.sync_read("Present_Temperature", normalize=False, num_retry=3)
        volt = bus.sync_read("Present_Voltage", normalize=False, num_retry=3)
    finally:
        bus.disconnect(disable_torque=False)

    q = lerobot_to_rad(reading, jm)
    lo, hi = urdf_limits()
    print(f"  joint map verified: {jm.verified or 'not yet (run so101-jointcheck)'}\n")
    print(
        f"  {'joint':14s} {'ticks':>6s} {'lerobot':>9s} {'q[deg]':>8s} {'URDF range[deg]':>17s} "
        f"{'torque':>6s} {'temp':>5s} {'V':>5s}"
    )
    for i, j in enumerate(JOINTS):
        inside = lo[i] <= q[i] <= hi[i]
        unit = "%" if j == "gripper" else "deg"
        print(
            f"  {j:14s} {raw[j]:6d} {reading[j]:7.1f}{unit:>2s} {np.rad2deg(q[i]):8.1f} "
            f"{np.rad2deg(lo[i]):7.1f}..{np.rad2deg(hi[i]):6.1f}{'' if inside else ' !':2s}"
            f"{'on' if torque[j] else 'off':>6s} {temp[j]:4d}C {volt[j] / 10:5.1f}"
        )
    print("\nAll checks passed." if ok else "\nSome checks FAILED (see above).")


if __name__ == "__main__":
    main()
