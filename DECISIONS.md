# Decisions

Key decisions for the SO-101 project series, newest last. Each entry: what was decided, the options, and why.

## 2026-11-06: project structure and order
- **Decided:** split the work into separate repos (M0–M5 in the README) plus this shared `so101-core`, instead of one large repo.
- **Options:**
  - one monorepo;
  - separate repos with copied utilities;
  - separate repos plus a thin shared core.
- **Why:**
  - Each repo stands on its own and can be shown alone.
  - The shared core keeps device paths, frames, units and the safety layer in one place, so the repos don't drift apart.

## 2026-11-06: repos are public
- **Decided:** all project repos are public.

## 2026-11-06: M4 extends vla-isp-probe
- **Decided:** the real-robot ISP-robustness study goes into the existing [vla-isp-probe](https://github.com/hujunhan/vla-isp-probe) repo, not a new one.
- **Why:** simulation findings and their real-robot check belong to one research story.

## 2026-11-06: global stack
- **Decided:** the conventions table in the README:
  - LeRobot, pinned;
  - no ROS 2 in the main line;
  - Pinocchio and MuJoCo;
  - rerun;
  - SI units and `T_<to>_<from>` transforms;
  - LeRobotDataset;
  - pre-registered evaluation protocols.
- **Deferred:**
  - ZED SDK vs raw UVC stereo (decide in M2);
  - the M3 task (decide when M3 starts);
  - any C++ component (only if it earns its time).

## 2026-11-06: hardware to buy
- A rigid ChArUco board (printed, mounted on acrylic or foam board), for M2.
- A dimmable desk lamp, for M4 illuminance control.
- A table clamp for the arm base.

## 2026-11-06: `SafeArm.follow` for trajectories, with a tighter stall watch
- **Decided:** add `follow(q_path)`: one safety-filtered command per row at the safety rate, a read after
  each, a log of commanded vs measured, counts of the ticks the filter clipped or step-capped, and a stall
  watch of 5 deg for 0.3 s (`move_to` keeps 15 deg for 0.5 s). The path must start within one step cap of the
  current command.
- **Why:** M1's drawing needs a trajectory at a fixed rate; `move_to` ramps to a goal and waits to settle, and
  `command()` has no stall detection. A slow trajectory should not be allowed to drift 15 deg from its
  command, and a path the safety layer had to change must be visible instead of silently distorted.
- **Options:** put the loop in the M1 repo (every later repo that follows a trajectory would repeat the
  safety logic); reuse `move_to` waypoint by waypoint (it settles at each one, so no continuous motion).
