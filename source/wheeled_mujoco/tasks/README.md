# wheeled_mujoco.tasks

[中文](README.zh.md) · [Parent directory](../README.md)

This folder stores MuJoCo task metadata that mirrors the training-side task families.

## What these files contain

Each `*_env_cfg.py` file defines:

- a stable MuJoCo task name such as `wheeled_base_locked`
- the corresponding training env config reference
- the expected training log name
- the default MuJoCo asset path
- the default exported policy path
- the nominal command ranges
- the runner kind required to execute the task

## Current status

- `wheeled_base_locked` and `wheeled_base_locked_hard` are mapped to the base-locked runner kind
- arm, grasp, obstacle, and recovery tasks are registered with asset/policy metadata only
- the registry is assembled in `__init__.py` and exposed through `TASK_REGISTRY`

This layer is metadata, not the runtime executor. Actual rollout dispatch happens in `wheeled_mujoco.sim2sim`.
