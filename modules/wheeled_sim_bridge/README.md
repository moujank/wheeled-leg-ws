# wheeled_sim_bridge

[中文](README.zh.md) · [Parent directory](../README.md)

This directory is reserved for system integration and simulator bridging work. It currently contains documentation only.

## Intended scope

- ROS2 launch composition across training-side tools, MuJoCo, and future hardware interfaces
- command/state bridge nodes
- simulator abstraction layers so different backends can expose a stable robot-facing API
- orchestration of startup, reset, and diagnostics across components

## Current status

- no bridge package, launch tree, or runtime node implementation is present yet

The existing MuJoCo package under `modules/wheeled_mujoco` already contains placeholder ROS-facing interfaces. If this module becomes active later, it should define the cross-process boundary rather than duplicate sim-internal code.
