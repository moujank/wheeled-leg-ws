# minimal_sim2sim

该目录包含已经验证过的、低风险的 base-locked 轮腿策略 MuJoCo rollout 基线。

## 目标

这套代码的目标非常直接：

- 加载导出策略
- 构造预期观测向量
- 执行推理
- 将动作送入 MuJoCo actuator
- 验证整条闭环是否足够稳定，从而作为参考基线

它不是最终部署栈，也不是通用任务框架。

## 关键文件

- `run_base_locked.py`：主执行基线
- `minimal_sim2sim.py`：与 runner 放在同目录的辅助模块

## 默认资产

- 机器人模型：`modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml`
- 默认策略根目录：`modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported`
- 默认策略文件：`policy.pt`

## 当前假设

- 观测维度：`27`
- 动作维度：`6`
- 动作顺序：`HipL HipR KneeL KneeR HubL HubR`
- 命令观测：`[cmd_vx, cmd_vy, cmd_wz]`

## 常用命令

运行默认基线：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py
```

运行一个短时无界面版本：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --no-viewer \
  --no-real-time \
  --duration 5
```

指定导出策略文件：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --policy modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.pt
```

切换到 ONNX：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --backend onnx \
  --policy modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.onnx
```

如果需要完整查看启动、控制、viewer、日志与稳定化参数，请使用：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --help
```

这条基线应尽量保持稳定。新的实验通常应在它之上分层，而不是直接在原地替换。
