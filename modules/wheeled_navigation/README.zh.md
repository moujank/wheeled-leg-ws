# wheeled_navigation

[English](README.md) · [上级目录](../README.md)

本次改动、历史实验结果及限制见[工作总结](../../docs/NAVIGATION_PROGRESS.zh.md)。从干净克隆准备资产、运行和导出结果，优先使用[复现入口](../../research/README.zh.md)；本文下方早期演示说明保留了当时的实验背景。

该目录现已包含可运行的 Isaac Sim 激光扫描建图与目标点规划基线，运行方法及验证条件见[演示说明](DEMO.zh.md)。

从空白地图开始、在线选择探索目标，以及实际 SLAM Toolbox / Nav2 闭环，见[未知场景自主探索说明](ONLINE_EXPLORATION.zh.md)。这条新流程不使用预定路线；早期演示与视频仍保留，便于比较。

床头、床尾通道加宽后的独立场景与重新建图命令，见[通道加宽实验](OPEN_PASSAGES_DEMO.zh.md)。

## 预期职责

- 定位与里程计融合
- 地图构建与地图服务
- 全局规划与局部运动规划
- 将导航目标连接到运动控制或任务控制层的接口

## 当前状态

- `scan_nav/`：占据栅格建图、考虑机器人半径的 A* 规划和报告生成。
- `scripts/plan_goal.py`：在已保存地图上重新指定目标点。
- `artifacts/isaac_demo/`：实际运行生成的地图、路径、指标和可查看场景。
- `config/` 和 `launch/`：实际 SLAM Toolbox / Nav2 接入，配套隔离的本地 ROS 环境。
- `scripts/run_autonomy.py`：启动未知场景探索闭环，支持原生栅格版本和 ROS 版本。
- 已实际验证在线探索、真实 SLAM Toolbox / Nav2 与带误差里程计；执行器仍为速度积分，尚未验证 RL 路线执行或双轮足平衡动力学。回环是否实际发生需要额外位姿图证据。

后续启用该模块时，应保持在原始运动控制之上的层级，输入机器人状态，输出更高层的移动目标。

## Code-as-Room 室内演示

本地 Code-as-Room 家具房间的真实 PhysX 雷达采样、扫描建图和目标点 A* 规划，见 [CODE_AS_ROOM_DEMO.zh.md](CODE_AS_ROOM_DEMO.zh.md)。这条流程采用脚本设置位姿与仿真真值定位，包含高清预览、方法示意、建图动图与离线数据复查。

机器人沿路线扫描、让暗处房间逐渐显现的视频见 [CODE_AS_ROOM_VIDEO.zh.md](CODE_AS_ROOM_VIDEO.zh.md)。视频使用 `wheeled_arm_v2` 的弯腿、收臂姿态，并完整保留三个隔间之间的两道内部隔墙。
