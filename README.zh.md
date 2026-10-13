# wheeled_mujoco

[English](README.md) · [模块文档索引](docs/README.md)

该模块是轮腿项目的 MuJoCo 侧工作区。

## 目录结构

```text
modules/wheeled_mujoco/
├── asset/                 # MJCF、URDF 镜像、mesh 资产和索引辅助脚本
├── minimal_sim2sim/       # 已验证的 base-locked 最小 MuJoCo 闭环
├── scripts/               # 命令行入口
├── source/                # 可导入 Python 包
└── docs/                  # 架构、一致性、交接、资产索引与历史记录
```

## 当前职责

- 提供 `wheeled_leg` 与 `wheeled_arm` 的 MuJoCo 机器人资产
- 运行 `modules/wheeled_train/wheeled_leg/logs/rsl_rl` 导出的策略
- 保持一条已经验证过的、低风险的 base-locked rollout 路径
- 为后续更多 MuJoCo 任务预留配置和脚本边界

## 当前运行状态

- 目前只有 `wheeled_base_locked` 这条任务路径完全可运行
- `wheeled_base_locked_hard` 复用了同一个 runner，但仍应视为待验证对象，不是锁定基线
- arm、grasp、obstacle、recovery 等任务名目前只是在 MuJoCo 配置层做了预留

## 调用链路

```text
scripts/run_task_sim2sim.py
  -> source/wheeled_mujoco/sim2sim
  -> source/wheeled_mujoco/tasks
  -> minimal_sim2sim/run_base_locked.py
  -> asset/ + wheeled_train 导出策略
```

## 常用命令

列出已注册的 MuJoCo 任务配置：

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py --list-tasks
```

通过脚本包装器运行已验证的 base-locked 路径：

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --no-viewer \
  --no-real-time \
  --duration 5
```

直接运行最小基线：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --duration 5
```

更详细的说明请继续查看 `minimal_sim2sim/`、`scripts/` 和 `source/` 下的 README。

## 文档导航

[最小闭环](minimal_sim2sim/README.zh.md) · [命令行入口](scripts/README.zh.md) · [源码包](source/wheeled_mujoco/README.zh.md) · [专题说明](docs/README.md)

历史验证结果对应当时的策略及参数；现有策略的状态应结合 [调试交接](docs/MUJOCO_DEBUG_HANDOFF.md) 与所选 checkpoint 确认。上述命令从工作区根目录执行。
