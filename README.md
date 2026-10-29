# wheeled-leg-ws

[中文](README.zh.md) · [Documentation index](docs/README.md) · [Modules](modules/README.md) · [TODO](docs/TODO.md)

This repository is the main workspace for a wheeled-leg robot stack. It contains the following implementation tracks:

- `modules/wheeled_train/wheeled_leg`: Isaac Lab training tasks, scripts, and experiment outputs.
- `modules/wheeled_mujoco`: MuJoCo assets and sim-to-sim validation tools that reuse policies exported from training.
- `modules/wheeled_navigation` and `modules/wheeled_sim_bridge`: measured Isaac lidar, online exploration with SLAM Toolbox / Nav2, and point-cloud exports. Execution is currently kinematic; RL tracking and balancing have not been validated. See the [navigation progress report](docs/NAVIGATION_PROGRESS.zh.md) and [reproduction guide](research/README.zh.md).

Other module directories primarily retain boundaries and documentation for future hardware, deployment, teleoperation and firmware work. Navigation and simulation bridging now contain implementations; this does not establish successful physical robot navigation.

## Workspace layout

```text
wheeled-leg-ws/
├── assets/                         # USD/URDF exports and raw robot assets used outside MuJoCo
├── docs/                           # Workspace docs, migration workflow, TODO, history
├── modules/
│   ├── wheeled_train/             # Isaac Lab training workspace
│   ├── wheeled_mujoco/            # MuJoCo sim2sim workspace
│   ├── wheeled_sim_bridge/        # Isaac sensors and ROS integration
│   ├── wheeled_deploy/            # Reserved on-robot deployment module
│   ├── wheeled_description/       # Reserved source-of-truth description module
│   ├── wheeled_navigation/        # Scan mapping, exploration and Nav2
│   ├── wheeled_teleop/            # Reserved teleoperation and UI module
│   ├── wheeled_hardware/          # Reserved hardware data module
│   └── wheeled_firmware/          # Reserved firmware module
├── sync_to_github.sh              # Push main + subtree split branches
├── update_git_history.sh          # Regenerate docs/GIT_HISTORY.md from git log
└── README.md / README.zh.md        # Workspace entry points
```

## Active code areas

### Training

`modules/wheeled_train/wheeled_leg` is the real Isaac Lab project in this workspace. It includes:

- task registration under `source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg`
- training and playback scripts for `rsl_rl`, `skrl`, and `rl_games`
- experiment outputs under `logs/` and `outputs/`
- helper scripts such as `scripts/list_envs.py` and `scripts/rsl_rl/update_best_policy.py`

### MuJoCo

`modules/wheeled_mujoco` is the real MuJoCo-side project. It includes:

- MuJoCo MJCF and converted robot assets under `asset/`
- a validated minimal base-locked rollout under `minimal_sim2sim/`
- importable package code under `source/wheeled_mujoco/`
- script entry points under `scripts/`

At the moment, `wheeled_base_locked` is the only fully runnable MuJoCo path. Other task names are registered as reserved configs so the training and MuJoCo naming stays aligned.

## Top-level assets and tools

- `assets/`: exported USD, URDF, mesh, and package-style robot assets used by the Isaac-side workflow.
- `modules/wheeled_train/wheeled_leg/tools/plot_joint_torque.py`: plotting utility for joint torque traces.
- `modules/wheeled_train/wheeled_leg/tools/plot_mean_reward.py`: plotting utility for reward curves or aggregate reward logs.
- `MUJOCO_LOG.TXT` and `thinking.txt`: working notes captured in the repository root.

## Common workflows

### Refresh the generated git history file

```bash
./update_git_history.sh
```

This rewrites [docs/GIT_HISTORY.md](docs/GIT_HISTORY.md) from the current local git history.

### Sync the workspace to GitHub

```bash
./sync_to_github.sh
```

If no commit message is provided, the script generates one from the current timestamp. The script also:

- removes tracked local VS Code database files from the git index
- commits staged changes on `main`
- pushes `main`
- splits `modules/wheeled_train` to branch `wheeled_train`
- splits `modules/wheeled_mujoco` to branch `wheeled_mujoco`

Provide a custom message when needed:

```bash
./sync_to_github.sh "sync: describe the current change"
```

## Notes

- This worktree is actively used for experimentation. Generated logs, checkpoints, exported policies, and local notes may coexist with source changes.
- The repository already contains non-README technical notes such as [migration workflow](docs/SIM2SIM_DYNAMIC_FLOW.md), [MuJoCo architecture](modules/wheeled_mujoco/docs/SIM2SIM_ARCHITECTURE.md), and [debug handoff](modules/wheeled_mujoco/docs/MUJOCO_DEBUG_HANDOFF.md). Those files carry workflow state and should be read together with the module READMEs when working on sim2sim.

Training and playback analysis tools are maintained in [the training tools directory](modules/wheeled_train/wheeled_leg/tools/README.md), including torque statistics and TensorBoard scalar analysis.
