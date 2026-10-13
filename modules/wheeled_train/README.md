# wheeled_train

[中文](README.zh.md) · [Parent directory](../README.md)

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

## Project documentation

Installation, registered tasks, training/play commands, and logging conventions are maintained in the [project README](wheeled_leg/README.md). See [POLICY_INDEX.md](wheeled_leg/logs/rsl_rl/POLICY_INDEX.md) for checkpoint selection.
