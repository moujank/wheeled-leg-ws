# wheeled_leg Isaac Lab project

This directory is the active Isaac Lab extension project for the wheeled-leg robot. It is no longer a generic template in practice, even though some package metadata still comes from the upstream Isaac Lab extension template.

## Structure

```text
modules/wheeled_train/wheeled_leg/
├── source/wheeled_leg/
│   ├── setup.py
│   ├── config/extension.toml
│   └── wheeled_leg/
│       └── tasks/manager_based/wheeled_leg/
├── scripts/
│   ├── list_envs.py
│   ├── zero_agent.py
│   ├── random_agent.py
│   ├── rsl_rl/
│   ├── skrl/
│   └── rl_games/
├── logs/rsl_rl/
├── outputs/
├── tools/
└── .vscode/tools/setup_vscode.py
```

## What is implemented

### Task package

The main task package is:

`source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg`

It currently contains:

- base-locked locomotion variants
- arm-free locomotion, obstacle, and recovery tasks
- grasp-pose locomotion
- cartesian grasp
- loaded grasp obstacle
- terrain builders, assets, MDP fragments, and agent configs

### Script entry points

- `scripts/list_envs.py`: print all registered `Wheeled-...` environments
- `scripts/zero_agent.py`: run a zero-action smoke test
- `scripts/random_agent.py`: run a random-action smoke test
- `scripts/rsl_rl/train.py`: RSL-RL training
- `scripts/rsl_rl/play.py`: RSL-RL playback
- `scripts/rsl_rl/update_best_policy.py`: update the `best/` pointer for a chosen experiment
- `scripts/skrl/*.py` and `scripts/rl_games/*.py`: alternative RL library entry points

### Checked-in experiment artifacts

This repository already contains:

- `logs/rsl_rl/*/best/` folders with selected policy pointers and notes
- exported torch or ONNX policies for some experiments
- `outputs/` directories from Isaac Lab or Hydra runs

Treat those directories as working artifacts, not immutable release assets.

## Installation

Use a Python environment that already has Isaac Lab installed, then install the local extension in editable mode:

```bash
cd modules/wheeled_train/wheeled_leg
python -m pip install -e source/wheeled_leg
```

The package metadata in `source/wheeled_leg/config/extension.toml` still carries upstream template values such as `Extension Template`. That metadata does not change the runtime task registration, but it is worth cleaning up later.

## Registered tasks

The current registry in `source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/__init__.py` includes:

- `Wheeled-Base-Locked-simple`
- `Wheeled-Base-Locked-simple-Play`
- `Wheeled-Base-Locked-Curriculum`
- `Wheeled-Base-Locked-Curriculum-Play`
- `Wheeled-Arm-Free-Locomotion-v0`
- `Wheeled-Arm-Free-Locomotion-Play-v0`
- `Wheeled-Arm-Free-Recovery-v0`
- `Wheeled-Arm-Free-Recovery-Play-v0`
- `Wheeled-Arm-Free-Obstacle-v0`
- `Wheeled-Arm-Free-Obstacle-Play-v0`
- `Wheeled-Grasp-Pose-Locomotion-v0`
- `Wheeled-Grasp-Pose-Locomotion-Play-v0`
- `Wheeled-Cartesian-Grasp-v0`
- `Wheeled-Cartesian-Grasp-Play-v0`
- `Wheeled-Loaded-Grasp-Obstacle-v0`
- `Wheeled-Loaded-Grasp-Obstacle-Play-v0`

## Typical commands

### List environments

```bash
cd modules/wheeled_train/wheeled_leg
python scripts/list_envs.py
```

### Train with RSL-RL

```bash
cd modules/wheeled_train/wheeled_leg
python scripts/rsl_rl/train.py \
  --task=Wheeled-Base-Locked-simple \
  --agent=rsl_rl_cfg_entry_point
```

### Play a selected checkpoint

```bash
cd modules/wheeled_train/wheeled_leg
python scripts/rsl_rl/play.py \
  --task=Wheeled-Base-Locked-simple-Play \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=wheeled_base_locked_simple \
  --load_run=best \
  --checkpoint=model.pt
```

### Run quick environment smoke tests

```bash
python scripts/zero_agent.py --task=Wheeled-Base-Locked-simple
python scripts/random_agent.py --task=Wheeled-Base-Locked-simple
```

## Logging conventions

- training logs live under `logs/rsl_rl/<experiment_name>/`
- manually selected checkpoint pointers live under `logs/rsl_rl/<experiment_name>/best/`
- run-time outputs from Hydra or Isaac Lab live under `outputs/<date>/<time>/`

If a training run still writes to an unexpected default directory, check the current working directory first. `scripts/rsl_rl/train.py` writes logs relative to the directory where the script is launched.
