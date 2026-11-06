# Working notes for Claude

M0 of the SO-101 series and the shared base for later repos. The series plan (M0–M5) and M0 findings are in
README.md; architecture and per-file roles in docs/DESIGN.md; decisions in DECISIONS.md.

## Current state
- M0 done and verified on the real arm (joint map = identity, rest pose recorded).
- Active work is M1 in `~/code/mine/so101-kinematics`, which pins this repo by commit. Change this repo only
  for fixes or features later milestones need, then bump the pin there.

## How we work
- Claude writes and runs most of the code. The user does physical work on the arm, makes the key
  decisions and learns the concepts. Present options with a recommendation and let the user decide, then
  record the choice in DECISIONS.md (date, decided, why, options).
- Talk to the user in Chinese. Code, comments, commits and repo docs are in English.

## Rules
- The repo is public. Never commit personal information. `config/devices.yaml` holds adapter serials and
  stays gitignored; only `devices.example.yaml` is committed.
- All motion goes through `SafeArm` / `SafetyLayer`. Don't weaken the safety defaults without the user's
  decision. Tell the user before anything moves the arm.
- LeRobot is pinned to one commit (extra `hw`); keep it pinned.
- Tooling: uv, Python 3.12, ruff, pytest (hardware-free, fake bus). Run `uv run ruff check . && uv run pytest`
  before committing. Add a regression test for every hardware bug found.
- Update docs/DESIGN.md when the architecture changes.
