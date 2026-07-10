# wheeled_mujoco/scripts

该目录存放 MuJoCo 侧工作流的命令行入口。

## 当前脚本

- `run_base_locked_sim2sim.py`：已验证 base-locked runner 的直接 CLI 包装器
- `run_task_sim2sim.py`：任务分发入口，先解析已注册的 MuJoCo 任务配置，再把参数转发给对应 runner

## 当前行为

目前这两个脚本最终都会把 base-locked 执行路由到：

```text
minimal_sim2sim/run_base_locked.py
```

之所以保留 `source/wheeled_mujoco/sim2sim/base_locked.py` 这一中间层，是为了让未来更多任务共享稳定的包边界，而不是直接复制旧基线代码。

## 常用命令

查看完整 base-locked CLI：

```bash
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py --help
```

运行一次短时无界面的 base-locked rollout：

```bash
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py \
  --no-viewer \
  --no-real-time \
  --duration 5
```

列出已注册任务配置：

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py --list-tasks
```

查看选定任务 runner 的帮助：

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --runner-help
```

通过任务级包装器执行同一条 rollout：

```bash
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --no-viewer \
  --no-real-time \
  --duration 5
```

不要把可复用的仿真逻辑写在这个目录里；可复用代码应放到 `source/wheeled_mujoco/`。
