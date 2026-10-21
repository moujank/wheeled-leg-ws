# wheeled_leg Isaac Lab 工程

[English](README.md) · [上级目录](../README.zh.md)

该目录是轮腿机器人的 Isaac Lab 主工程。虽然部分包元数据仍沿用了上游 Isaac Lab 模板，但从实际内容看，它已经不是通用模板仓库，而是当前项目的真实训练工程。

## 目录结构

```text
modules/wheeled_train/wheeled_leg/
├── source/wheeled_leg/
│   ├── setup.py
│   ├── config/extension.toml
│   └── wheeled_leg/
│       └── tasks/manager_based/wheeled_leg/
├── scripts/
│   ├── list_envs.py
│   ├── zero_agent.py
│   ├── random_agent.py
│   ├── rsl_rl/
│   ├── skrl/
│   └── rl_games/
├── logs/rsl_rl/
├── outputs/
├── tools/
└── .vscode/tools/setup_vscode.py
```

## 当前已实现内容

多地形稳定运动基础任务为 `Wheeled-Leg-Terrain-Balance` 和 `Wheeled-Leg-Terrain-Balance-Play`，
使用 `wheeled_leg_v2` 资产，实验目录为 `logs/rsl_rl/wheeled_leg_terrain_balance/`。
地形课程、控制方案和训练命令见 [多地形稳定运动说明](TERRAIN_BALANCE.zh.md)。
左右协调奖励、镜像增强和 checkpoint 迁移说明见 [非 v0 左右协调训练](SYMMETRY.zh.md)。

`wheeled_leg_v2` 的独立训练与 play 任务为 `Wheeled-Leg-v1` 和 `Wheeled-Leg-Play-v1`，
实验目录为 `logs/rsl_rl/wheeled_leg_v1/`。训练、绘图和最佳策略管理命令见
[双轮足 v1 使用说明](WHEELED_LEG_V1.zh.md)。

越障与严格跳跃任务使用独立的 `asset/wheeled_leg_v2_agile.py` 资产类，直接定义各自的场景和 MDP，不继承 terrain-balance：

- `Wheeled-Leg-Obstacle-Jump` / `Wheeled-Leg-Obstacle-Jump-Play`：倒金字塔台阶与随机高度方格，任意方式越过 20 cm 障碍，实验目录 `logs/rsl_rl/wheeled_leg_obstacle_jump/`。
- `Wheeled-Leg-Jump` / `Wheeled-Leg-Jump-Play`：成功率课程：原地跳跃 → 移动跳跃 → 跳过长方体障碍并到达目标，实验目录 `logs/rsl_rl/wheeled_leg_jump/`。

两项任务均支持从现有 v1 / terrain-balance 策略通过 `--init_checkpoint` 初始化新实验，支持奖励绘图、回放力矩记录和最佳策略管理。配置、迁移训练、回放与分析命令见 [越障与跳跃说明](OBSTACLE_JUMP.zh.md)。

### 任务包

主任务包位于：

`source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg`

目前已经包含：

- base-locked locomotion 变体
- arm-free locomotion、obstacle、recovery 任务
- grasp-pose locomotion
- cartesian grasp
- loaded grasp obstacle
- terrain 构建、资产定义、MDP 片段和 agent 配置

### 脚本入口

- `scripts/list_envs.py`：列出所有已注册的 `Wheeled-...` 环境
- `scripts/zero_agent.py`：零动作冒烟测试
- `scripts/random_agent.py`：随机动作冒烟测试
- `scripts/rsl_rl/train.py`：RSL-RL 训练
- `scripts/rsl_rl/play.py`：RSL-RL 回放
- `scripts/rsl_rl/update_best_policy.py`：更新指定实验的 `best/` 指针
- `scripts/skrl/*.py` 与 `scripts/rl_games/*.py`：其他 RL 库入口

### 已提交的实验产物

当前仓库已经直接包含：

- `logs/rsl_rl/*/best/` 下的最佳策略指针与说明
- 若干实验的 torch 或 ONNX 导出策略
- `outputs/` 下的 Isaac Lab / Hydra 运行输出

这些目录应视为工作产物，而不是不可变的正式发布资产。

## 安装

请在已经安装 Isaac Lab 的 Python 环境中，以 editable 模式安装本地扩展：

```bash
cd modules/wheeled_train/wheeled_leg
python -m pip install -e source/wheeled_leg
```

`source/wheeled_leg/config/extension.toml` 中仍保留了上游模板元数据，例如 `Extension Template`。这不会影响任务注册运行，但后续值得单独清理。

## 已注册任务

当前 `source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/__init__.py` 中实际注册了：

- `Wheeled-Leg-Terrain-Balance`
- `Wheeled-Leg-Terrain-Balance-Play`
- `Wheeled-Leg-Obstacle-Jump`
- `Wheeled-Leg-Obstacle-Jump-Play`
- `Wheeled-Leg-Jump`
- `Wheeled-Leg-Jump-Play`
- `Wheeled-Base-Locked-simple`
- `Wheeled-Base-Locked-simple-Play`
- `Wheeled-Base-Locked-Curriculum`
- `Wheeled-Base-Locked-Curriculum-Play`
- `Wheeled-Arm-Free-Locomotion-v0`
- `Wheeled-Arm-Free-Locomotion-Play-v0`
- `Wheeled-Arm-Free-Recovery-v0`
- `Wheeled-Arm-Free-Recovery-Play-v0`
- `Wheeled-Arm-Free-Obstacle-v0`
- `Wheeled-Arm-Free-Obstacle-Play-v0`
- `Wheeled-Grasp-Pose-Locomotion-v0`
- `Wheeled-Grasp-Pose-Locomotion-Play-v0`
- `Wheeled-Cartesian-Grasp-v0`
- `Wheeled-Cartesian-Grasp-Play-v0`
- `Wheeled-Loaded-Grasp-Obstacle-v0`
- `Wheeled-Loaded-Grasp-Obstacle-Play-v0`

## 常用命令

### 列出环境

```bash
cd modules/wheeled_train/wheeled_leg
python scripts/list_envs.py
```

### 使用 RSL-RL 训练

```bash
cd modules/wheeled_train/wheeled_leg
python scripts/rsl_rl/train.py \
  --task=Wheeled-Base-Locked-simple \
  --agent=rsl_rl_cfg_entry_point
```

### 回放选定 checkpoint

```bash
cd modules/wheeled_train/wheeled_leg
python scripts/rsl_rl/play.py \
  --task=Wheeled-Base-Locked-simple-Play \
  --agent=rsl_rl_cfg_entry_point \
  --experiment_name=wheeled_base_locked_simple \
  --load_run=best \
  --checkpoint=model.pt
```

### 快速环境冒烟测试

`Wheeled-Leg-v0` 的最小 PPO 训练已完成一次，配置、产物和暂缓问题见 [运行验证记录](docs/WHEELED_LEG_V0_RUN_STATUS.md)。

```bash
python scripts/zero_agent.py --task=Wheeled-Base-Locked-simple
python scripts/random_agent.py --task=Wheeled-Base-Locked-simple
```

## 日志约定

- 训练日志位于 `logs/rsl_rl/<experiment_name>/`
- 人工选定的最优 checkpoint 指针位于 `logs/rsl_rl/<experiment_name>/best/`
- Hydra / Isaac Lab 运行输出位于 `outputs/<date>/<time>/`

如果训练仍然写入了意料之外的默认目录，先检查当前工作目录。`scripts/rsl_rl/train.py` 会相对于脚本启动目录写日志。

## 文档导航

[训练模块入口](../README.zh.md) · [策略索引](logs/rsl_rl/POLICY_INDEX.md) · [工作区文档](../../../docs/README.md)

各实验的 `best/notes.md` 保存独立的 checkpoint 来源和选择理由，由策略维护脚本管理，索引汇总于 `POLICY_INDEX.md`。

[分析工具说明](tools/README.md)：关节力矩绘图、等效扭矩统计与 TensorBoard 奖励/标量分析统一位于 `tools/`。
