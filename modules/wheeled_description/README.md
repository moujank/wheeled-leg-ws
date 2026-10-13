# wheeled_description

[中文](README.zh.md) · [Parent directory](../README.md)

This directory is reserved for robot description source-of-truth data. It currently contains no implementation files beyond documentation.

## Intended scope

- canonical robot naming, frame conventions, and joint definitions
- source CAD-to-URDF or source URDF-to-derived-asset relationships
- documentation for how training, MuJoCo, and deployment assets are derived
- versioned ownership of description changes before they are exported elsewhere

## Current status

- no source description package is stored here yet
- exported assets currently live in `assets/` and `modules/wheeled_mujoco/asset/`

If this module becomes active later, it should describe how those exported assets are generated and validated rather than duplicating derived files blindly.
