# wheeled_leg Isaac Lab 工程

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

```bash
python scripts/zero_agent.py --task=Wheeled-Base-Locked-simple
python scripts/random_agent.py --task=Wheeled-Base-Locked-simple
```

## 日志约定

- 训练日志位于 `logs/rsl_rl/<experiment_name>/`
- 人工选定的最优 checkpoint 指针位于 `logs/rsl_rl/<experiment_name>/best/`
- Hydra / Isaac Lab 运行输出位于 `outputs/<date>/<time>/`

如果训练仍然写入了意料之外的默认目录，先检查当前工作目录。`scripts/rsl_rl/train.py` 会相对于脚本启动目录写日志。
