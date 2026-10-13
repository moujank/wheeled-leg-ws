# wheeled_mujoco.ros

[中文](README.zh.md) · [Parent directory](../README.md)

This folder reserves ROS-facing interfaces for future MuJoCo integration.

## Current files

- `interfaces.py`: lightweight protocol and node-spec definitions only

## Current status

- no ROS2 runtime dependency is imported here
- no publishers, subscribers, launch files, or message packages are implemented here yet

## Intended future roles

- command input node
- MuJoCo state publisher
- policy runner bridge
- task supervisor that switches or blends task policies

Keep this layer at the process boundary. Sim-internal control logic should stay in `sim2sim/` or lower-level runtime code.
