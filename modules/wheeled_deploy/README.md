# wheeled_deploy

[中文](README.zh.md) · [Parent directory](../README.md)

This directory is reserved for on-robot deployment code. It does not yet contain runtime implementation files.

## Intended scope

- package the trained policy and runtime dependencies for robot-side execution
- host deployment launchers, service wrappers, and startup scripts
- implement safety state machines, watchdogs, and recovery hooks
- bridge policy inference with hardware command/output interfaces

## Current status

- only documentation exists in this module
- no deploy pipeline, runtime daemon, or launch entry point has been committed yet

When deployment code is added later, keep this module focused on robot-side execution concerns rather than training or simulation utilities.
