"""so101-twin: live MuJoCo twin of the real arm.

--source follower  the follower arm, torque OFF (move it by hand)
--source leader    the leader arm (always limp)
--source sim       no hardware; sweeps each joint through its range (check the viewer works)
"""

import argparse
import time

import numpy as np

from so101_core.arm import Rate
from so101_core.config import DeviceConfig, JointMap
from so101_core.convert import lerobot_to_rad
from so101_core.model import JOINTS, urdf_limits
from so101_core.twin import Twin


def open_bus(source: str, dev: DeviceConfig):
    if source == "follower":
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        dev_obj = SO101Follower(SO101FollowerConfig(port=dev.follower_port, id=dev.robot_id))
    else:
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

        dev_obj = SO101Leader(SO101LeaderConfig(port=dev.leader_port, id=dev.teleop_id))
    bus = dev_obj.bus
    bus.connect()
    if not bus.is_calibrated:
        bus.disconnect(disable_torque=False)
        raise RuntimeError(f"{source}: motor calibration does not match the LeRobot file")
    return bus


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["follower", "leader", "sim"], default="follower")
    ap.add_argument("--hz", type=float, default=30.0)
    args = ap.parse_args()

    twin = Twin().open_viewer()
    rate = Rate(args.hz)
    if args.source == "sim":
        lo, hi = urdf_limits()
        t0 = time.perf_counter()
        while twin.show((lo + hi) / 2 + (hi - lo) / 2 * 0.8 * np.sin(time.perf_counter() - t0)):
            rate.sleep()
        return

    dev = DeviceConfig.load()
    jm = JointMap.load() if args.source == "follower" else JointMap()
    bus = open_bus(args.source, dev)
    try:
        if args.source == "follower":
            if any(bus.sync_read("Torque_Enable", normalize=False).values()):
                input(">>> Follower torque is ON. Support the arm, then press Enter to release it... ")
            bus.disable_torque()
        print("Move the arm by hand; close the viewer window or press Ctrl-C to stop.")
        last = 0.0
        while True:
            q = lerobot_to_rad(bus.sync_read("Present_Position", num_retry=3), jm)
            if not twin.show(q):
                break
            if time.perf_counter() - last > 0.5:
                last = time.perf_counter()
                vals = (f"{j[:8]}={np.rad2deg(v):6.1f}" for j, v in zip(JOINTS, q, strict=True))
                print("  " + "  ".join(vals), end="\r")
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        bus.disconnect(disable_torque=False)
        twin.close()
        print()


if __name__ == "__main__":
    main()
