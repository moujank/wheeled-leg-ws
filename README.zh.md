# wheeled_train

[English](README.md) · [上级目录](../README.md)

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

## 工程文档

安装、任务清单、训练/回放命令及日志约定统一维护在 [工程 README](wheeled_leg/README.zh.md)。策略选择说明见 [POLICY_INDEX.md](wheeled_leg/logs/rsl_rl/POLICY_INDEX.md)。
