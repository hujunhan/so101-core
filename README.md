# so101-core

Shared foundation and roadmap for a series of robotics projects on the
[SO-101](https://github.com/TheRobotStudio/SO-ARM100) arm: kinematics, camera
calibration, robot learning, camera/ISP robustness of learned policies, and a
classical-vs-learned comparison.

Each project lives in its own repo. This repo holds what they all share
(hardware access, safety layer, conventions, digital twin) and the plan that
ties them together.

> **Status (2026-11-06):** M0 in progress: package, safety layer, MuJoCo twin and tools written and
> tested without hardware; first run on the real arm pending.

## Setup

| Part | What |
|---|---|
| Arm | SO-101 follower + leader (Feetech STS3215 servos), driven through [LeRobot](https://github.com/huggingface/lerobot) |
| Cameras | Stereolabs ZED 2 (stereo + IMU, scene view); a USB UVC camera on the wrist (MJPG 720p, manual exposure / white balance / gamma, no RAW) |
| Compute | Linux laptop with an RTX 3080 Laptop GPU (16 GB) |

## Roadmap

```
M0 so101-core ─→ M1 so101-kinematics ─→ M2 so101-calib ─┐
                                                        ├─→ M5 so101-plan-vs-learn
                       M3 so101-policy ─→ M4 vla-isp-probe (real robot) ─┘
```

| # | Repo | Scope | Demo / output |
|---|---|---|---|
| **M0** | `so101-core` (this repo) | Environment; device config by stable `/dev/.../by-id` paths; **safety layer** (joint-limit clipping, max step per command, rate limit, one-call torque off); per-joint direction check; MuJoCo digital twin driven by the real joint state | Move the real arm by hand, the MuJoCo twin follows |
| **M1** | `so101-kinematics` | Port my 2023 [arm_control](https://github.com/hujunhan/arm_control) (URDF parsing, FK, Jacobian IK) to the SO-101 and benchmark it against Pinocchio; joint-limited damped-least-squares IK; drawing shapes in the air; commanded-vs-measured tracking error | Real and simulated arm drawing side by side; comparison table (accuracy, solve time, limit violations) |
| **M2** | `so101-calib` | Wrist-camera intrinsics (ChArUco); ZED 2 check against factory calibration; **hand-eye calibration**, eye-in-hand and eye-to-hand; servo system identification (backlash, steady-state error, latency); an **error budget** splitting end-effector error into camera, kinematics and servo terms | Click a point in the ZED image and the tip goes there; error heat map over the workspace, in mm |
| **M3** | `so101-policy` | One manipulation task; 50–100 teleoperated demonstrations; ACT, then fine-tuned SmolVLA (pi0.5 as a stretch); **pre-registered evaluation protocol** with success rates, confidence intervals and a failure taxonomy | Video + results table + failure analysis |
| **M4** | [vla-isp-probe](https://github.com/hujunhan/vla-isp-probe), real-robot part | Run the M3 policy under (a) **physical camera settings**: exposure, white balance, gamma, sharpness and scene illuminance; (b) **software perturbations**: unprocess to pseudo-RAW, add calibrated noise, re-render. Measure the success-rate drop, then try ISP-aware augmentation | Same policy under six camera settings; robustness curves; sim-vs-real comparison with the existing LIBERO results |
| **M5** | `so101-plan-vs-learn` | Same task, same protocol: a **classical pipeline** (ZED detection → calibration → IK → grasp) against the **M3 learned policy**. Compare success, cycle time, robustness to lighting (reusing M4) and engineering cost | Write-up: planning vs learning on a low-cost arm |

**Why this order**
- M1 before M2 is a hard dependency: hand-eye calibration needs end-effector poses from forward kinematics.
- M4 needs a working policy from M3.
- M5 reuses everything before it.

**Out of scope for this hardware:** force / impedance control (the servos are position-controlled), dexterous hands, high-rate real-time control.

## M0: using this package

```bash
uv sync --extra hw                      # LeRobot pinned; drop `--extra hw` for sim-only use
cp config/devices.example.yaml config/devices.yaml   # then fill in your by-id paths
uv run so101-check                      # read-only: devices, calibration, joint table (never moves)
uv run so101-twin --source sim          # viewer only, no hardware
uv run so101-twin --source follower     # live twin; torque off, move the arm by hand
uv run so101-jointcheck                 # verify signs, record rest pose, then a powered +5 deg check
uv run pytest                           # hardware-free tests (fake servo bus)
```

| Module | Role |
|---|---|
| `model` | Joint names, URDF/MJCF paths (vendored from SO-ARM100, Apache-2.0), joint limits |
| `config` | `DeviceConfig` (git-ignored `config/devices.yaml`) and `JointMap` (per-joint sign, offset, rest pose) |
| `convert` | The only place LeRobot degrees / gripper % meet radians |
| `safety` | Clip to URDF limits minus a margin, cap the step per command; never pushes further out from outside the range |
| `arm` | `SafeArm`: sets goal = present before enabling torque, refuses to auto-calibrate, detects stalls from following error, holds then asks before releasing torque on Ctrl-C |
| `twin` | Kinematic MuJoCo twin and viewer |

## Shared conventions (decided once, used by every repo)

| Area | Choice | Why |
|---|---|---|
| Environment | Python 3.12 + uv; each repo has its own `pyproject.toml` and depends on `so101-core` through git | |
| Hardware, datasets, policies | LeRobot, **pinned to one commit**, wrapped rather than forked | Driver, dataset format and 20+ policies already exist |
| Middleware | **No ROS 2 in the main line**; an optional ROS 2 bridge can be added later | Not needed for a single arm; keeps the stack small |
| Kinematics | Pinocchio as the reference, my own code alongside it | Industry standard; LeRobot's placo is built on it |
| Simulation / twin | MuJoCo with the official SO-101 MJCF | Light, runs on a laptop |
| Vision | OpenCV (ChArUco). ZED SDK vs raw UVC stereo: decided in M2 | |
| Visualization | [rerun](https://rerun.io) for every demo | Ships with LeRobot; one consistent look |
| Units and frames | SI units and radians inside; degrees only at the hardware boundary. Transforms are named `T_<to>_<from>` and stored as 4×4 matrices in YAML. Devices are opened by `by-id` path only | Avoids sign and unit mistakes |
| Data | Every recording is a LeRobotDataset under one Hugging Face Hub namespace | |
| Experiments | Training runs in wandb. Real-robot evaluations: one CSV row per trial, against a protocol written **before** the runs | Same pre-registration practice as vla-isp-probe |
| Quality | pytest + ruff; GitHub Actions runs the simulation-only tests | |
| C++ / Rust | Not in the main line. A C++ (Ceres) calibration refinement is an optional M2 extension | Added only if it earns its time |
| Compute | Local RTX 3080 (16 GB) for ACT, Diffusion Policy, SmolVLA; rent a cloud GPU only if a pi0.5 fine-tune needs it | |

## How the work is done

I direct these projects and develop them with [Claude Code](https://claude.com/claude-code) as a coding agent.

- **Claude:** writes and runs the code, tests and analysis, and drafts plots and READMEs.
- **Me:** everything physical (wiring, safety supervision, calibration captures, teleoperation, resets, observation) and every key decision. For each decision I choose among options with stated trade-offs, and the choice is logged in the repo's `DECISIONS.md`.

Each module ends with an explain-back: interview-style questions until I can defend every design choice. The "decisions and lessons" section of each README is written by me.

Each session (1–2 h) follows the same pattern:
1. Recap and today's goal.
2. The physical part and the decisions.
3. Code, runs and analysis.
4. A short explain-back and a `DECISIONS.md` entry.

Every session ends at a point where the work can stop cleanly.

## License

MIT
