# Project Memory

This repository is the robotics practice car and manipulator codebase.

## Project Context

- Main platform: STM32H7 / H743
- OS: FreeRTOS with CMSIS-OS v1
- Main subsystems: chassis control, mode switching, CAN comms, USART comms, line tracking, vision comms, upper mechanism control
- Simulation / validation: MATLAB and MuJoCo

## Working Rules

- Check `git status` and `git diff --stat` before starting a new session or resuming old work.
- Read the relevant source files and `develop_log/` notes before changing control logic.
- Make one logical change at a time. Keep edits surgical and avoid unrelated refactors.
- After every edit, read the file back and confirm the change is exact.
- Prefer evidence over guesswork. Report what you verified, not what you assume.
- If something is unclear, call out the ambiguity instead of silently choosing.

## File and Log Policy

- `develop_log/*.md` is append-only. Do not rewrite whole logs when a small section changes.
- Do not use destructive deletes. If a file must be removed, move it to a recycle area instead.
- Do not touch `~/.local/share/opencode/` runtime data unless the user explicitly asks.
- Keep any new outputs in new directories. Do not overwrite existing experiment results.

## Search and Tooling

- Use `rg` / `rg --files` first for file and text search.
- Keep command output short when exploring large trees.
- Prefer PowerShell-native commands on Windows.

## Control-Stack Notes

- The current code structure already has chassis, mode switch, CAN, USART, PID, and protocol layers.
- Manipulator work should stay aligned with the existing architecture:
  - `freertos.c` for task registration
  - `control_def.h` for feature macros
  - `Application/arm_task.c/.h` for the arm task
  - `Application/arm_ik.c/.h` for IK and kinematics
- For the arm, prefer a DH-based workflow with symbolic derivation where possible:
  - DH table -> forward kinematics
  - Jacobian derivation
  - IK closed form if feasible
  - gravity compensation and control code generation

## Validation Expectations

- For control changes, validate against the smallest relevant surface first.
- For simulation changes, verify both the model and the control input path.
- For docs and logs, preserve existing structure and wording unless a correction is necessary.

