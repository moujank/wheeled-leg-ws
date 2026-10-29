# wheeled-leg-ws

[English](README.md) · [文档索引](docs/README.md) · [模块导航](modules/README.md) · [待办](docs/TODO.md)

这个仓库是轮腿机器人项目的主工作区。目前有以下实现主线：

- `modules/wheeled_train/wheeled_leg`：Isaac Lab 训练任务、脚本与实验输出。
- `modules/wheeled_mujoco`：MuJoCo 资产与 sim2sim 验证工具，直接复用训练导出的策略。
- `modules/wheeled_navigation` 与 `modules/wheeled_sim_bridge`：Isaac Sim 传感器、SLAM Toolbox / Nav2 自主探索及点云展示。当前采用速度积分执行器，尚未接入 RL 策略验证平衡动力学。详见[本次导航工作总结](docs/NAVIGATION_PROGRESS.zh.md)和[复现入口](research/README.zh.md)。

硬件、部署、遥操作和固件等其他目录仍以模块边界与资料整理为主。导航与仿真桥接目录已包含实际实现，不能据此理解为实物自主导航已经验证完成。

## 工作区结构

```text
wheeled-leg-ws/
├── assets/                         # MuJoCo 之外使用的 USD / URDF 原始资产
├── docs/                           # 工作区文档索引、迁移流程、待办、历史
├── modules/
│   ├── wheeled_train/             # Isaac Lab 训练工作区
│   ├── wheeled_mujoco/            # MuJoCo sim2sim 工作区
│   ├── wheeled_sim_bridge/        # Isaac 传感器与 ROS 桥接
│   ├── wheeled_deploy/            # 预留真机部署模块
│   ├── wheeled_description/       # 预留统一描述真源模块
│   ├── wheeled_navigation/        # 扫描建图、在线探索与 Nav2
│   ├── wheeled_teleop/            # 预留遥操作与界面模块
│   ├── wheeled_hardware/          # 预留硬件资料模块
│   └── wheeled_firmware/          # 预留固件模块
├── sync_to_github.sh              # 推送 main 和 subtree 拆分分支
├── update_git_history.sh          # 基于 git log 重新生成 docs/GIT_HISTORY.md
└── README.md / README.zh.md        # 工作区入口
```

## 当前有效代码区

### 训练侧

`modules/wheeled_train/wheeled_leg` 是当前真实可用的 Isaac Lab 工程，包含：

- `source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg` 下的任务注册与环境配置
- `rsl_rl`、`skrl`、`rl_games` 的训练与回放脚本
- `logs/` 和 `outputs/` 下的实验产物
- `scripts/list_envs.py`、`scripts/rsl_rl/update_best_policy.py` 等辅助脚本

### MuJoCo 侧

`modules/wheeled_mujoco` 是当前真实可用的 MuJoCo 工程，包含：

- `asset/` 下的 MuJoCo MJCF 与转换后的机器人资产
- `minimal_sim2sim/` 下已经验证过的 base-locked 最小闭环
- `source/wheeled_mujoco/` 下的可导入 Python 包
- `scripts/` 下的命令行入口

目前只有 `wheeled_base_locked` 这条 MuJoCo 路径已经真正可运行。其他任务名已在配置层预留，用来保证训练侧和 MuJoCo 侧命名一致，但尚未完成完整运行实现。

## 顶层资产与工具

- `assets/`：供 Isaac 侧使用的 USD、URDF、mesh 以及 package 风格机器人资产。
- `modules/wheeled_train/wheeled_leg/tools/plot_joint_torque.py`：关节力矩曲线绘图工具。
- `modules/wheeled_train/wheeled_leg/tools/plot_mean_reward.py`：奖励曲线或平均奖励日志绘图工具。
- `MUJOCO_LOG.TXT` 与 `thinking.txt`：保存在仓库根目录的工作记录。

## 常用工作流

### 刷新自动生成的 Git 历史文件

```bash
./update_git_history.sh
```

该脚本会根据当前本地 git 历史重写 [docs/GIT_HISTORY.md](docs/GIT_HISTORY.md)。

### 同步工作区到 GitHub

```bash
./sync_to_github.sh
```

如果不提供提交说明，脚本会自动使用当前时间生成提交消息。脚本还会额外执行：

- 从 git 索引中移除本地 VS Code 数据库文件
- 在 `main` 上提交已暂存修改
- 推送 `main`
- 将 `modules/wheeled_train` 拆分到 `wheeled_train` 分支
- 将 `modules/wheeled_mujoco` 拆分到 `wheeled_mujoco` 分支

需要自定义提交消息时：

```bash
./sync_to_github.sh "sync: 描述当前改动"
```

## 说明

- 这个工作区处于持续实验状态。源码改动、生成日志、checkpoint、导出策略和临时记录会同时存在。
- 仓库中已经包含若干非 README 技术说明文件，例如 [迁移流程](docs/SIM2SIM_DYNAMIC_FLOW.md)、[MuJoCo 架构](modules/wheeled_mujoco/docs/SIM2SIM_ARCHITECTURE.md) 和 [调试交接](modules/wheeled_mujoco/docs/MUJOCO_DEBUG_HANDOFF.md)。做 sim2sim 相关工作时，应该与模块 README 一起阅读。

训练与回放分析工具统一位于 [训练工程 tools](modules/wheeled_train/wheeled_leg/tools/README.md)，使用说明包含力矩统计和 TensorBoard 标量分析。
