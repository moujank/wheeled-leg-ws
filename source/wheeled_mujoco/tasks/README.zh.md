# wheeled_mujoco.tasks

[English](README.md) · [上级目录](../README.zh.md)

该目录保存与训练侧任务族对齐的 MuJoCo 任务元数据。

## 这些文件包含什么

每个 `*_env_cfg.py` 文件都会定义：

- 一个稳定的 MuJoCo 任务名，例如 `wheeled_base_locked`
- 对应的训练环境配置引用
- 期望的训练日志名
- 默认 MuJoCo 资产路径
- 默认导出策略路径
- 名义控制命令范围
- 执行该任务所需的 runner 类型

## 当前状态

- `wheeled_base_locked` 和 `wheeled_base_locked_hard` 已映射到 base-locked runner 类型
- arm、grasp、obstacle、recovery 等任务目前只注册了资产 / 策略元数据
- 注册表在 `__init__.py` 中组装，并通过 `TASK_REGISTRY` 暴露

这一层只是元数据层，不负责真正执行 rollout。实际运行分发发生在 `wheeled_mujoco.sim2sim`。
