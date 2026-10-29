# wheeled_sim_bridge

[English](README.md) · [上级目录](../README.md)

该目录现已包含独立 Isaac Sim 场景与传感器演示，以及可选的 ROS 2 桥接。运行方法见[演示说明](../wheeled_navigation/DEMO.zh.md)。

新增的未知场景在线探索与实际 SLAM Toolbox / Nav2 闭环，见[自主探索说明](../wheeled_navigation/ONLINE_EXPLORATION.zh.md)。`scripts/run_online_exploration.py` 挂载三维 PhysX 雷达，发布扫描、带误差里程计和时钟，并执行导航端输出的速度。

## 预期职责

- 在训练工具、MuJoCo 和后续硬件接口之间组织 ROS2 启动编排
- 实现命令 / 状态桥接节点
- 提供仿真后端抽象层，让不同后端暴露稳定一致的机器人接口
- 统一处理启动、复位和诊断流程

## 当前状态

- `run_navigation_demo.ps1`：Windows 启动入口。
- `scripts/run_isaac_mapping.py`：创建场景、挂载机器人与雷达、采集扫描并验证规划和简化运动。
- `scripts/ros2_sensor_bridge.py`：发布时间、扫描、理想里程计、TF、地图与路径，订阅带超时处理的命令。
- `scripts/view_navigation_scene.py`：在 Isaac Sim 窗口中打开导出的场景。
- 既有 RL 训练和 MuJoCo 执行代码保持原样。新增简化驱动不用于评价运动策略性能。

`modules/wheeled_mujoco` 里已经有预留的 ROS 接口骨架。后续如果启用本模块，重点应是跨进程边界和系统集成，不应重复仿真内部逻辑。
