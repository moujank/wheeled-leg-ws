# MuJoCo 文档索引

[模块入口](../README.zh.md) · [English](../README.md) · [工作区文档](../../../docs/README.md)

## 专题说明

| 文档 | 用途 |
| --- | --- |
| [SIM2SIM_ARCHITECTURE.md](SIM2SIM_ARCHITECTURE.md) | 后续代码组织、分层职责与扩展方案 |
| [MUJOCO_CONSISTENCY_CHECKLIST.md](MUJOCO_CONSISTENCY_CHECKLIST.md) | 2026-07-08 训练产物与 MuJoCo 的一致性排查 |
| [MUJOCO_DEBUG_HANDOFF.md](MUJOCO_DEBUG_HANDOFF.md) | 调试交接入口、状态更新规则与历史证据入口 |
| [ASSET_INDEX.md](ASSET_INDEX.md) | 资产、关节、actuator、sensor、qpos 索引查询记录 |
| [archive/MUJOCO_DEBUG_HISTORY.md](archive/MUJOCO_DEBUG_HISTORY.md) | 原调试说明与流水完整归档 |

## 按代码层阅读

| 目录 | 英文 | 中文 |
| --- | --- | --- |
| 最小闭环 | [README](../minimal_sim2sim/README.md) | [README.zh](../minimal_sim2sim/README.zh.md) |
| 命令行入口 | [README](../scripts/README.md) | [README.zh](../scripts/README.zh.md) |
| 源码目录入口 | [README](../source/README.md) | [README.zh](../source/README.zh.md) |
| Python 包职责 | [README](../source/wheeled_mujoco/README.md) | [README.zh](../source/wheeled_mujoco/README.zh.md) |
| 任务配置 | [README](../source/wheeled_mujoco/tasks/README.md) | [README.zh](../source/wheeled_mujoco/tasks/README.zh.md) |
| 运行时适配 | [README](../source/wheeled_mujoco/sim2sim/README.md) | [README.zh](../source/wheeled_mujoco/sim2sim/README.zh.md) |
| ROS 接口 | [README](../source/wheeled_mujoco/ros/README.md) | [README.zh](../source/wheeled_mujoco/ros/README.zh.md) |
| 任务编排 | [README](../source/wheeled_mujoco/orchestration/README.md) | [README.zh](../source/wheeled_mujoco/orchestration/README.zh.md) |

架构方案与历史试验记录分别描述计划和既有证据。运行参数应从当前 runner、所选策略及训练配置确认；整理文档不构成新的运行验证。
