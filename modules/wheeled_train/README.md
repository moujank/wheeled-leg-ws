# wheeled_train

This directory hosts the training-side workspace for the wheeled-leg project. The active Isaac Lab project lives in `wheeled_leg/`.

## Structure

```text
modules/wheeled_train/
├── README.md
├── README.zh.md
└── wheeled_leg/
    ├── source/wheeled_leg/        # Installable Isaac Lab extension package
    ├── scripts/                   # Training, play, and utility entry points
    ├── logs/rsl_rl/               # Experiment logs, best checkpoints, exported policies
    ├── outputs/                   # Hydra / Isaac Lab run outputs
    └── tools/                     # Local analysis utilities
```

## What is implemented here

- custom manager-based wheeled tasks registered under the `Wheeled-...` namespace
- RSL-RL, SKRL, and RL-Games script entry points
- checked-in experiment artifacts for multiple tasks
- policy selection helper `scripts/rsl_rl/update_best_policy.py`

## Current task families

The active task package currently registers:

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

## Typical workflow

```bash
cd modules/wheeled_train/wheeled_leg
python -m pip install -e source/wheeled_leg
python scripts/list_envs.py
python scripts/rsl_rl/train.py --task=Wheeled-Base-Locked-simple
python scripts/rsl_rl/play.py --task=Wheeled-Base-Locked-simple-Play --load_run=best --checkpoint=model.pt
```

Use the README inside `wheeled_leg/` for the concrete project layout, install notes, and task/run conventions.
