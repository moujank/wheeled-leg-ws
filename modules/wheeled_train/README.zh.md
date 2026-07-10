# wheeled_train

该目录承载轮腿项目的训练侧工作区。当前真实可用的 Isaac Lab 工程位于 `wheeled_leg/`。

## 目录结构

```text
modules/wheeled_train/
├── README.md
├── README.zh.md
└── wheeled_leg/
    ├── source/wheeled_leg/        # 可安装的 Isaac Lab 扩展包
    ├── scripts/                   # 训练、回放和工具入口
    ├── logs/rsl_rl/               # 实验日志、最优 checkpoint、导出策略
    ├── outputs/                   # Hydra / Isaac Lab 运行输出
    └── tools/                     # 本地分析工具
```

## 当前已实现内容

- 在 `Wheeled-...` 命名空间下注册的自定义 manager-based 任务
- `RSL-RL`、`SKRL`、`RL-Games` 的脚本入口
- 多个任务的实验产物已直接提交在仓库中
- 用于维护最优策略索引的 `scripts/rsl_rl/update_best_policy.py`

## 当前任务族

当前任务包实际注册了：

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

## 常见工作流

```bash
cd modules/wheeled_train/wheeled_leg
python -m pip install -e source/wheeled_leg
python scripts/list_envs.py
python scripts/rsl_rl/train.py --task=Wheeled-Base-Locked-simple
python scripts/rsl_rl/play.py --task=Wheeled-Base-Locked-simple-Play --load_run=best --checkpoint=model.pt
```

具体工程结构、安装说明和任务 / run 约定见 `wheeled_leg/` 目录下的 README。
