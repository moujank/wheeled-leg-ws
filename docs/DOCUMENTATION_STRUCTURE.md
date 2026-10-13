# 文档结构与整理记录

[文档索引](README.md) · [模块导航](../modules/README.md)

整理日期：2026-10-12。初始盘点覆盖隐藏/忽略目录中的 Markdown，共 55 份；按 SHA-256 比较，完全相同的文件为 0 组。重复主要为任务列表、命令示例和上层概述，按职责合并。

## 目录与命名规则

- 工作区入口：根 `README.md` / `README.zh.md`。
- 跨模块专题、待办与生成历史：根 `docs/`。
- 模块入口：`modules/<module>/README.md` / `README.zh.md`。
- 模块专题：`modules/<module>/docs/`；历史证据放入其 `archive/`。
- 代码层说明：随目录保存 `README.md` / `README.zh.md`，上层提供导航。
- 实验说明：`logs/rsl_rl/POLICY_INDEX.md` 汇总，各 `best/notes.md` 保存独立 checkpoint 信息。
- Markdown 链接使用文件相对路径；代码/命令中的工作区路径保持原语义。
- 语言版本、不同实验与不同日期的试验记录分别维护，合并重复内容时保留特有信息。

## 合并与清理记录

| 原重复或混杂内容 | 整理结果 |
| --- | --- |
| 训练模块与工程 README 同时维护 16 个 task 和训练/回放命令 | 工程 README 维护详细内容，模块 README 链接工程与策略索引 |
| MuJoCo source/ 与 Python 包 README 重复职责和边界 | source/ 保留目录入口，包 README 汇总职责与子包链接 |
| 交接文档混合旧稳定结论、90D 失败和流水 | 原文完整归档，交接入口汇总证据范围及后续更新字段 |
| 7 月 8 日流水插入 7 月 4 日 text 代码块 | 移至历史文件末尾，保留两次记录内容并恢复代码块边界 |
| 旧资产 index_list.md 无标题，混合查询和历史输出 | 归入 ASSET_INDEX.md，补标题、步骤与历史范围说明 |
| 27D 说明被当成唯一观测维度 | 最小闭环 README 补充 27D / 30 × history 支持，旧流程标注基线日期 |
| 策略索引示例使用未注册的旧 base-locked v0 task | 按现有任务注册改为 simple / simple-Play，补充九个实验说明链接 |
| 机器本地绝对 Markdown 链接 | 转为文件相对链接 |
| 历史生成器输出根 GIT_HISTORY.md | 改为 docs/GIT_HISTORY.md，保持从工作区根执行 |

## 路径迁移对应表

| 原路径 | 新路径 |
| --- | --- |
| `SIM2SIM_DYNAMIC_FLOW.md` | [docs/SIM2SIM_DYNAMIC_FLOW.md](SIM2SIM_DYNAMIC_FLOW.md) |
| `GIT_HISTORY.md` | [docs/GIT_HISTORY.md](GIT_HISTORY.md) |
| `todo.md` | [docs/TODO.md](TODO.md) |
| `modules/wheeled_mujoco/SIM2SIM_ARCHITECTURE.md` | [modules/wheeled_mujoco/docs/SIM2SIM_ARCHITECTURE.md](../modules/wheeled_mujoco/docs/SIM2SIM_ARCHITECTURE.md) |
| `modules/wheeled_mujoco/MUJOCO_CONSISTENCY_CHECKLIST.md` | [modules/wheeled_mujoco/docs/MUJOCO_CONSISTENCY_CHECKLIST.md](../modules/wheeled_mujoco/docs/MUJOCO_CONSISTENCY_CHECKLIST.md) |
| `modules/wheeled_mujoco/MUJOCO_DEBUG_HANDOFF.md` | [modules/wheeled_mujoco/docs/archive/MUJOCO_DEBUG_HISTORY.md](../modules/wheeled_mujoco/docs/archive/MUJOCO_DEBUG_HISTORY.md) |
| `modules/wheeled_mujoco/asset/mjcf/index_list.md` | [modules/wheeled_mujoco/docs/ASSET_INDEX.md](../modules/wheeled_mujoco/docs/ASSET_INDEX.md) |
| `modules/wheeled_mujoco/readme.md` | [modules/wheeled_mujoco/README.md](../modules/wheeled_mujoco/README.md) |
| `modules/wheeled_mujoco/readme.zh.md` | [modules/wheeled_mujoco/README.zh.md](../modules/wheeled_mujoco/README.zh.md) |
| `modules/wheeled_mujoco/scripts/readme.md` | [modules/wheeled_mujoco/scripts/README.md](../modules/wheeled_mujoco/scripts/README.md) |
| `modules/wheeled_mujoco/scripts/readme.zh.md` | [modules/wheeled_mujoco/scripts/README.zh.md](../modules/wheeled_mujoco/scripts/README.zh.md) |
| `modules/wheeled_mujoco/source/readme.md` | [modules/wheeled_mujoco/source/README.md](../modules/wheeled_mujoco/source/README.md) |
| `modules/wheeled_mujoco/source/readme.zh.md` | [modules/wheeled_mujoco/source/README.zh.md](../modules/wheeled_mujoco/source/README.zh.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/readme.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/README.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/readme.zh.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/README.zh.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/tasks/readme.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/tasks/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/tasks/README.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/tasks/readme.zh.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/tasks/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/tasks/README.zh.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/readme.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/README.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/readme.zh.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/README.zh.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/ros/readme.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/ros/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/ros/README.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/ros/readme.zh.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/ros/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/ros/README.zh.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/readme.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/README.md) |
| `modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/readme.zh.md` | [modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/README.zh.md) |

新的 `modules/wheeled_mujoco/docs/MUJOCO_DEBUG_HANDOFF.md` 为交接摘要，原内容保存在上表所列历史归档。根 `todo.md` 的全部待办已整理到 `docs/TODO.md`。已跟踪文档的原始版本可通过 Git 历史恢复。

## 全部 Markdown 路径

以下清单包含忽略目录内的策略文档。按目录列出每份文件，方便检查各级文档归属。

整理后共 61 份 Markdown；新增导航及归档入口使总数增加，原历史证据保留。

| 路径 | 文档标题 |
| --- | --- |
| [README.md](../README.md) | wheeled-leg-ws |
| [README.zh.md](../README.zh.md) | wheeled-leg-ws |
| [docs/DOCUMENTATION_STRUCTURE.md](DOCUMENTATION_STRUCTURE.md) | 文档结构与整理记录 |
| [docs/GIT_HISTORY.md](GIT_HISTORY.md) | Git History |
| [docs/README.md](README.md) | 工作区文档索引 |
| [docs/SIM2SIM_DYNAMIC_FLOW.md](SIM2SIM_DYNAMIC_FLOW.md) | Wheeled-Leg Sim2Sim Dynamic Flow |
| [docs/TODO.md](TODO.md) | 项目待办 |
| [modules/README.md](../modules/README.md) | 模块导航 |
| [modules/wheeled_deploy/README.md](../modules/wheeled_deploy/README.md) | wheeled_deploy |
| [modules/wheeled_deploy/README.zh.md](../modules/wheeled_deploy/README.zh.md) | wheeled_deploy |
| [modules/wheeled_description/README.md](../modules/wheeled_description/README.md) | wheeled_description |
| [modules/wheeled_description/README.zh.md](../modules/wheeled_description/README.zh.md) | wheeled_description |
| [modules/wheeled_firmware/README.md](../modules/wheeled_firmware/README.md) | wheeled_firmware |
| [modules/wheeled_firmware/README.zh.md](../modules/wheeled_firmware/README.zh.md) | wheeled_firmware |
| [modules/wheeled_hardware/README.md](../modules/wheeled_hardware/README.md) | wheeled_hardware |
| [modules/wheeled_hardware/README.zh.md](../modules/wheeled_hardware/README.zh.md) | wheeled_hardware |
| [modules/wheeled_mujoco/README.md](../modules/wheeled_mujoco/README.md) | wheeled_mujoco |
| [modules/wheeled_mujoco/README.zh.md](../modules/wheeled_mujoco/README.zh.md) | wheeled_mujoco |
| [modules/wheeled_mujoco/docs/ASSET_INDEX.md](../modules/wheeled_mujoco/docs/ASSET_INDEX.md) | MuJoCo 资产与索引查询记录 |
| [modules/wheeled_mujoco/docs/MUJOCO_CONSISTENCY_CHECKLIST.md](../modules/wheeled_mujoco/docs/MUJOCO_CONSISTENCY_CHECKLIST.md) | MuJoCo Consistency Checklist |
| [modules/wheeled_mujoco/docs/MUJOCO_DEBUG_HANDOFF.md](../modules/wheeled_mujoco/docs/MUJOCO_DEBUG_HANDOFF.md) | MuJoCo Sim2Sim 调试交接 |
| [modules/wheeled_mujoco/docs/README.md](../modules/wheeled_mujoco/docs/README.md) | MuJoCo 文档索引 |
| [modules/wheeled_mujoco/docs/SIM2SIM_ARCHITECTURE.md](../modules/wheeled_mujoco/docs/SIM2SIM_ARCHITECTURE.md) | Wheeled MuJoCo Sim2Sim 架构与功能分工 |
| [modules/wheeled_mujoco/docs/archive/MUJOCO_DEBUG_HISTORY.md](../modules/wheeled_mujoco/docs/archive/MUJOCO_DEBUG_HISTORY.md) | MuJoCo Wheeled-Leg Sim2Sim 调试历史 |
| [modules/wheeled_mujoco/minimal_sim2sim/README.md](../modules/wheeled_mujoco/minimal_sim2sim/README.md) | minimal_sim2sim |
| [modules/wheeled_mujoco/minimal_sim2sim/README.zh.md](../modules/wheeled_mujoco/minimal_sim2sim/README.zh.md) | minimal_sim2sim |
| [modules/wheeled_mujoco/scripts/README.md](../modules/wheeled_mujoco/scripts/README.md) | wheeled_mujoco/scripts |
| [modules/wheeled_mujoco/scripts/README.zh.md](../modules/wheeled_mujoco/scripts/README.zh.md) | wheeled_mujoco/scripts |
| [modules/wheeled_mujoco/source/README.md](../modules/wheeled_mujoco/source/README.md) | wheeled_mujoco/source |
| [modules/wheeled_mujoco/source/README.zh.md](../modules/wheeled_mujoco/source/README.zh.md) | wheeled_mujoco/source |
| [modules/wheeled_mujoco/source/wheeled_mujoco/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/README.md) | wheeled_mujoco source package |
| [modules/wheeled_mujoco/source/wheeled_mujoco/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/README.zh.md) | wheeled_mujoco source package |
| [modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/README.md) | wheeled_mujoco.orchestration |
| [modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/orchestration/README.zh.md) | wheeled_mujoco.orchestration |
| [modules/wheeled_mujoco/source/wheeled_mujoco/ros/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/ros/README.md) | wheeled_mujoco.ros |
| [modules/wheeled_mujoco/source/wheeled_mujoco/ros/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/ros/README.zh.md) | wheeled_mujoco.ros |
| [modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/README.md) | wheeled_mujoco.sim2sim |
| [modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/README.zh.md) | wheeled_mujoco.sim2sim |
| [modules/wheeled_mujoco/source/wheeled_mujoco/tasks/README.md](../modules/wheeled_mujoco/source/wheeled_mujoco/tasks/README.md) | wheeled_mujoco.tasks |
| [modules/wheeled_mujoco/source/wheeled_mujoco/tasks/README.zh.md](../modules/wheeled_mujoco/source/wheeled_mujoco/tasks/README.zh.md) | wheeled_mujoco.tasks |
| [modules/wheeled_navigation/README.md](../modules/wheeled_navigation/README.md) | wheeled_navigation |
| [modules/wheeled_navigation/README.zh.md](../modules/wheeled_navigation/README.zh.md) | wheeled_navigation |
| [modules/wheeled_sim_bridge/README.md](../modules/wheeled_sim_bridge/README.md) | wheeled_sim_bridge |
| [modules/wheeled_sim_bridge/README.zh.md](../modules/wheeled_sim_bridge/README.zh.md) | wheeled_sim_bridge |
| [modules/wheeled_teleop/README.md](../modules/wheeled_teleop/README.md) | wheeled_teleop |
| [modules/wheeled_teleop/README.zh.md](../modules/wheeled_teleop/README.zh.md) | wheeled_teleop |
| [modules/wheeled_train/README.md](../modules/wheeled_train/README.md) | wheeled_train |
| [modules/wheeled_train/README.zh.md](../modules/wheeled_train/README.zh.md) | wheeled_train |
| [modules/wheeled_train/wheeled_leg/README.md](../modules/wheeled_train/wheeled_leg/README.md) | wheeled_leg Isaac Lab project |
| [modules/wheeled_train/wheeled_leg/README.zh.md](../modules/wheeled_train/wheeled_leg/README.zh.md) | wheeled_leg Isaac Lab 工程 |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/POLICY_INDEX.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/POLICY_INDEX.md) | RSL-RL Best Policy Index |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_arm_free_locomotion/best/notes.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_arm_free_locomotion/best/notes.md) | wheeled_arm_free_locomotion best policy |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_arm_free_obstacle/best/notes.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_arm_free_obstacle/best/notes.md) | wheeled_arm_free_obstacle best policy |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_arm_free_recovery/best/notes.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_arm_free_recovery/best/notes.md) | wheeled_arm_free_recovery best policy |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/notes.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/notes.md) | wheeled_base_locked best policy |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked_hard/best/notes.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked_hard/best/notes.md) | wheeled_base_locked_hard best policy |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked_simple/best/notes.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked_simple/best/notes.md) | wheeled_base_locked_simple best policy |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_cartesian_grasp/best/notes.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_cartesian_grasp/best/notes.md) | wheeled_cartesian_grasp best policy |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_grasp_pose_locomotion/best/notes.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_grasp_pose_locomotion/best/notes.md) | wheeled_grasp_pose_locomotion best policy |
| [modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_loaded_grasp_obstacle/best/notes.md](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_loaded_grasp_obstacle/best/notes.md) | wheeled_loaded_grasp_obstacle best policy |

| [modules/wheeled_train/wheeled_leg/tools/README.md](../modules/wheeled_train/wheeled_leg/tools/README.md) | 训练与回放分析工具 |

## 分析工具迁移

根 `tools/plot_joint_torque.py` 的能力已由训练目录增强版本覆盖，重复副本已移除；`tools/plot_mean_reward.py` 已迁入 `modules/wheeled_train/wheeled_leg/tools/`。工具用法统一维护于 [tools/README.md](../modules/wheeled_train/wheeled_leg/tools/README.md)。
