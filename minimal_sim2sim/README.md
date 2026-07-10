# minimal_sim2sim

This directory contains the validated low-risk MuJoCo rollout for the base-locked wheeled-leg policy.

## Purpose

The goal of this code is narrow and practical:

- load an exported policy
- build the expected observation vector
- run inference
- send actions into MuJoCo actuators
- verify that the loop behaves consistently enough to use as the reference baseline

It is not the final deployment stack and it is not a general task framework.

## Key files

- `run_base_locked.py`: the main executable baseline
- `minimal_sim2sim.py`: helper module kept alongside the runner

## Default assets

- robot model: `modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml`
- default policy root: `modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported`
- default policy file: `policy.pt`

## Current assumptions

- observation dimension: `27`
- action dimension: `6`
- action order: `HipL HipR KneeL KneeR HubL HubR`
- command observation: `[cmd_vx, cmd_vy, cmd_wz]`

## Typical commands

Run the default baseline:

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py
```

Run headless for a short duration:

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --no-viewer \
  --no-real-time \
  --duration 5
```

Point at a specific exported policy:

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --policy modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.pt
```

Switch to ONNX:

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --backend onnx \
  --policy modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.onnx
```

For the full list of startup, control, viewer, logging, and stabilization options, use:

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --help
```

Keep this baseline stable. New experiments should usually be layered above it rather than replacing it in place.
