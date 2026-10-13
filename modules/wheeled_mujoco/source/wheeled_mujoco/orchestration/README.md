# wheeled_mujoco.orchestration

[中文](README.zh.md) · [Parent directory](../README.md)

This folder reserves task-selection and multi-task coordination logic.

## Current files

- `task_switcher.py`: small in-process registry and transition description helpers

## Current status

- no policy blending or task arbitration runtime is implemented yet
- no direct MuJoCo control mutation happens here today

## Intended role

A single runner executes one task rollout. The orchestration layer should eventually decide:

- when to switch tasks
- how to represent task transitions
- whether a higher-level controller should sequence locomotion, recovery, obstacle, and grasp behaviors

Keep this boundary above individual runners.
