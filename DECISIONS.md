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
