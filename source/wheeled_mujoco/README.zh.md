# wheeled_mujoco source package

[English](README.md) · [上级目录](../README.zh.md)

`wheeled_mujoco` 是当前工作区中 MuJoCo 侧代码的 Python 包边界。

## 职责

- 用 MuJoCo 任务配置镜像训练侧任务命名
- 解析仓库相对的资产和策略路径
- 对外暴露可供脚本调用的 runner 适配层
- 为后续 ROS 集成和多任务编排预留接口

## 关键约束

- 该包刻意不依赖 Isaac Lab 运行时导入
- 已验证的 base-locked rollout 仍来自 `minimal_sim2sim/run_base_locked.py`
- 当前只有 base-locked 路径真正可运行；其他任务配置主要用于保留命名以及资产 / 策略映射关系

## 导入链路

```text
scripts/*.py
  -> wheeled_mujoco.sim2sim
  -> wheeled_mujoco.tasks
  -> wheeled_mujoco.paths
```

## 子包导航

[任务配置](tasks/README.zh.md) · [运行时适配](sim2sim/README.zh.md) · [ROS 接口](ros/README.zh.md) · [编排](orchestration/README.zh.md)

共享路径工具位于 `paths.py`；一次性执行入口位于模块 `scripts/`。
