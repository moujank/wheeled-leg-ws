# wheeled_mujoco/scripts

该目录包含 MuJoCo sim2sim 工作流的命令行入口。

当前脚本：

- `run_base_locked_sim2sim.py`：当前 `base_locked` 策略 rollout 的完整 argparse 包装入口。它会调用 source 层适配器，并保持与 `minimal_sim2sim/run_base_locked.py` 一致的参数默认值。
- `run_task_sim2sim.py`：按任务分发的入口。它可以列出已注册的任务配置，目前 `base_locked` 任务会路由到 base-locked runner。

预期调用顺序：

```text
scripts/*.py -> source/wheeled_mujoco/sim2sim -> source/wheeled_mujoco/tasks -> asset/log files
```

当前 `wheeled_base_locked` 任务的实际运行路径是：

```text
scripts/run_base_locked_sim2sim.py
  -> source/wheeled_mujoco/sim2sim/base_locked.py
  -> minimal_sim2sim/run_base_locked.py
```

这意味着入口已经走的是新的项目结构，但底层仍然复用当前验证过的 base-locked MuJoCo 实现，因此启动、reset、观测、动作和控制器行为与最小调试基线保持一致。

运行示例：

```bash
# 查看完整 base-locked CLI
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py --help
```

```bash
# 通过新脚本路径执行启动回归
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py \
  --no-viewer \
  --no-real-time \
  --no-handoff-log-every-step \
  --duration 4.5 \
  --cmd-vx 0.0
```

```bash
# 对已验证的 base-locked 任务做短跑前向测试
python3 modules/wheeled_mujoco/scripts/run_base_locked_sim2sim.py \
  --no-viewer \
  --no-real-time \
  --no-handoff-log-every-step \
  --duration 6 \
  --cmd-vx 0.1
```

```bash
# 通过任务分发入口运行同一个 base-locked rollout
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --no-viewer \
  --no-real-time \
  --no-handoff-log-every-step \
  --duration 4.5 \
  --cmd-vx 0.0
```

```bash
# 通过任务分发入口启动 MuJoCo 可视化窗口。
# viewer 默认就是开启的，所以这里的 --viewer 只是显式写出。
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --viewer \
  --real-time \
  --duration 30 \
  --cmd-vx 0.5
```

```bash
# 查看选定任务底层 runner 的完整 CLI 帮助
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py \
  --task wheeled_base_locked \
  --runner-help
```

```bash
# 查看当前注册的 MuJoCo 任务列表
python3 modules/wheeled_mujoco/scripts/run_task_sim2sim.py --list-tasks
```

验证说明：

```text
新的脚本通路已经和原有启动回归做过对照检查。
运行 `scripts/run_base_locked_sim2sim.py` 时，4.5s 启动状态与最小 runner 一致：
base_pos=(+0.007, -0.002, +0.324), rpy_deg=(-0.4, -2.3, -2.8), base_contact=False, leg_contact=False。
```

不要把可复用的仿真逻辑写在这里。可复用逻辑应放到 `source/wheeled_mujoco` 下。
