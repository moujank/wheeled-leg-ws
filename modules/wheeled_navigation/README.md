# wheeled_navigation

This directory is reserved for navigation and autonomy work. It currently contains documentation only.

## Intended scope

- localization and odometry fusion
- mapping and map-serving utilities
- path planning and local motion planning
- interfaces between navigation goals and locomotion/task controllers

## Current status

- no navigation stack, maps, or launch files are present yet

When this module becomes active, keep it above raw locomotion control. It should consume robot state and produce higher-level mobility objectives.
