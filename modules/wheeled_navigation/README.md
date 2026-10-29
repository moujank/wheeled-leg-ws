# wheeled_navigation

[中文](README.zh.md) · [Parent directory](../README.md)

This directory contains a runnable Isaac Sim scan-mapping and goal-planning baseline. See [the demo guide](DEMO.zh.md) for commands and validation conditions.

For blank-map online exploration and the actual SLAM Toolbox / Nav2 loop, see [the online exploration guide](ONLINE_EXPLORATION.zh.md). Earlier prescribed-tour demos remain as separate historical examples.

## Intended scope

- localization and odometry fusion
- mapping and map-serving utilities
- path planning and local motion planning
- interfaces between navigation goals and locomotion/task controllers

## Current status

- `scan_nav/`: log-odds occupancy mapping, conservative footprint-aware A*, and report generation
- `scripts/plan_goal.py`: plan another goal on a recorded scan map
- `artifacts/isaac_demo/`: measured simulator results and viewable scene
- `config/` and `launch/`: real SLAM Toolbox / Nav2 integration with an isolated local ROS runtime
- `scripts/run_autonomy.py`: blank-map online exploration, native or ROS version
- Online exploration and actual SLAM Toolbox / Nav2 with biased odometry have been run. Motion still integrates velocity kinematically; RL locomotion and balancing dynamics are not validated. Enabled loop closure is not evidence that a loop closure occurred.

When this module becomes active, keep it above raw locomotion control. It should consume robot state and produce higher-level mobility objectives.
