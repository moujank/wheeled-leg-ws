# wheeled_sim_bridge

[中文](README.zh.md) · [Parent directory](../README.md)

This directory now contains the standalone Isaac Sim scene/sensor demo and an optional ROS 2 bridge. See [the demo guide](../wheeled_navigation/DEMO.zh.md).

## Intended scope

- ROS2 launch composition across training-side tools, MuJoCo, and future hardware interfaces
- command/state bridge nodes
- simulator abstraction layers so different backends can expose a stable robot-facing API
- orchestration of startup, reset, and diagnostics across components

## Current status

- `run_navigation_demo.ps1`: Windows entry point
- `scripts/run_isaac_mapping.py`: create the scene, mount the robot/lidar, collect scans, validate planning and demo motion
- `scripts/ros2_sensor_bridge.py`: live clock, scan, ideal odometry, TF, map and path publishing; command subscription with timeout
- `scripts/view_navigation_scene.py`: open the exported scene in Isaac Sim's GUI
- Existing RL training and MuJoCo runtime code is unchanged. The new kinematic driver does not validate RL locomotion.

The existing MuJoCo package under `modules/wheeled_mujoco` already contains placeholder ROS-facing interfaces. If this module becomes active later, it should define the cross-process boundary rather than duplicate sim-internal code.
