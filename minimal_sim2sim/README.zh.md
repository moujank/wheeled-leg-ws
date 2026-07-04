# Minimal Sim2Sim

这个目录提供一套仅用于跑通最小技术路线的 MuJoCo 3.4.0 `sim2sim` 脚本。

当前只覆盖你正在训练的 `wheeled-base-locked` 任务：

- 机器人模型：`modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml`
- 策略目录：`modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported`
- 默认策略文件：`policy.pt`

设计目标：

- 优先跑通策略加载 -> 观测构造 -> 动作推理 -> MuJoCo actuator 控制闭环
- 不复用旧的类人机器人 `sim2sim.py` 结构
- 不依赖 `mujoco_viewer`
- 尽量和 `base_locked_locomotion_env_cfg.py` 的动作/观测定义对齐

## 运行

先确保 `mujoco` 环境里装了 PyTorch，这样默认的 `policy.pt` 才能直接加载：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mujoco
pip install torch --index-url https://download.pytorch.org/whl/cpu
```

如果命令行里不额外传参数，脚本默认行为是：

- 使用 `wheeled_leg.xml`
- 使用 `policy.pt`
- 使用 `torch` backend
- 使用 `isaac_play` 模式
- 打开 MuJoCo 官方 viewer
- 以较慢速度播放，可视化默认按 `0.5x` 速度运行，但不改变实际 `sim_dt`
- 启动或 reset 后先进行 `1s` 初始吊起，默认吊起高度为 `z=0.32`
- 默认流程为 `1s` 初始吊起 -> `1s` 平滑释放 -> `2s` 稳定站立 -> 再接入策略
- 首次启动默认使用确定性零速度初始状态，并在 `isaac_play` 下默认启用命令软启动与动作软启动
- reset 时对齐 Isaac Play 语义：随机 root 的 `x/y/yaw`、随机 root 速度，joint 初值回到任务默认零位
- 吊起结束后直接接入运动策略
- 默认前向目标速度为 `cmd_vx=0.5`
- 默认腿部动作 scale 为 `0.5`，轮子动作 scale 为 `10.0`，与当前 Isaac `base_locked` 训练配置一致
- 持续运行，直到你手动 `Ctrl+C` 结束

默认启动命令：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mujoco
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py
```

如果你想尝试“用策略自己把速度压到 0”的站立方式：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --stand-strategy policy_zero
```

如果你想显式使用之前的实验性“吊起-平滑过渡-低速运行”路径：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --mode experimental \
  --startup-tether 5 \
  --tether-release 1.5 \
  --cmd-vx 0.1
```

如果你想修改 Isaac Play 对齐模式下的前向测试命令：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --cmd-vx 0.5
```

如果你想进一步放慢或恢复默认播放速度：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --playback-speed 0.25
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --playback-speed 1.0
```

如果你想记录 MuJoCo 诊断日志，便于和 Isaac 输出逐项对齐：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --log-path /tmp/wheeled_mujoco_diag.jsonl
```

如果你想直接看策略接入瞬间前后 `0.5s` 的高频状态，并打印 27 维观测拆解：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --mode experimental \
  --startup-tether 1.0 \
  --tether-release 0.2 \
  --cmd-vx 0.1 \
  --verbose-obs \
  --log-path /tmp/wheeled_handoff_diag.jsonl
```

如果你想直接做轮子 action scale 的 A/B 对比：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --wheel-action-scale 6.0
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --wheel-action-scale 10.0
```

如果你只想在终端里跑、不打开可视化窗口：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer
```

如果你想关闭真实时间限速，让仿真尽可能快跑：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-real-time
```

如果你想显式切回零速策略命令：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --cmd-vx 0.0
```

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mujoco
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --duration 5
```

打开 MuJoCo 官方可视化窗口：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mujoco
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --viewer \
  --real-time \
  --mode isaac_play \
  --cmd-vx 0.5 \
  --duration 30
```

如果想显式指定策略路径：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --policy modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.pt
```

如果想测试 ONNX 导出：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --backend onnx \
  --policy modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.onnx
```

## 当前假设

- 观测维度为 `27`
- 动作维度为 `6`
- 动作顺序为 `HipL HipR KneeL KneeR HubL HubR`
- 腿部动作是位置偏移控制，scale=`0.5`
- 轮子动作是速度控制，scale=`10.0`
- 默认启动流程为 `1s tether -> 1s release -> 2s stand -> policy`
- 命令观测为 `[cmd_vx, cmd_vy, cmd_wz]`

## 注意

这不是最终部署脚本，只是一个最小闭环验证工具。后续如果你要做真正的仿真对齐，通常还需要继续补：

- viewer 或录像
- reset / fail-safe
- Isaac 与 MuJoCo 传感器定义严格对齐
- 指令采样、噪声、随机化复刻
- 更完整的日志和指标输出
