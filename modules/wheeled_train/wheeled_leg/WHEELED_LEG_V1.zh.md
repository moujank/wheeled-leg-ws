# wheeled_leg_v2 独立训练任务

环境配置：`source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/v2_wheeled_leg_v1.py`。
机器人资产：同目录下的 `asset/wheeled_leg_v2.py`，使用 `WHEELED_CFG`。

| 用途 | 注册任务名 | PPO 配置 |
| --- | --- | --- |
| 训练 | `Wheeled-Leg-v1` | `WheeledLegV1PPORunnerCfg` |
| Play | `Wheeled-Leg-Play-v1` | `WheeledLegV1PPORunnerCfg` |
| 教师学生蒸馏 | `Wheeled-Leg-Distillation-v1` | `DistillationRunnerCfg` |
| 学生 Play / 导出 | `Wheeled-Leg-Distillation-Play-v1` | `DistillationRunnerCfg` |

代码组织沿用 `v2_wheeled_leg_v0.py`，当前参数已根据 2026-10-13 的训练结果调整。
当前阶段训练平地站立与直线速度跟踪，暂不训练复杂地形和转向。
训练指令为前后速度 ±0.5 m/s，20% 的指令采样为站立，偏航速度为零。
初始髋角为 ±0.65 rad、膝角为 ±1.30 rad，根部高度 0.58 m。
腿部目标限幅为 HipL [0.3, 1.0]、KneeL [0.9, 1.7]，右侧为对应的负角度范围；轮毂目标速度限幅 ±6 rad/s。
任务内通过 `velocity_limit_sim` 设置执行器仿真速度上限 2π rad/s。
Actor 增加机身线速度反馈并移除轮毂累计转角，critic 使用完整状态与接触观测。
线速度跟踪权重由 50 调整为 3，指数奖励的速度容差参数由 0.5 收紧为 0.25 m/s，
使站立平衡与实际跟踪运动指令的奖励有更明显的区别；新增存活、腿关节速度和左右腿镜像约束。
训练默认 4096 个环境，play 默认 50 个环境。物理步长 0.002 s，动作间隔 4 个物理步，控制频率 125 Hz。
腿部使用位置动作，轮毂使用速度动作。复位时关节位置取默认位置的 0.95–1.05 倍，保持左右关节初始角度的符号。
训练关闭周期推扰，质量随机增量缩小到 0–0.5 kg。Play 使用默认姿态、零复位速度，关闭观测噪声与物理随机化，并默认记录环境 0 的全部关节施加力矩。

## 工作目录和训练

以下命令均在安装了 Isaac Lab、RSL-RL 和本地任务扩展的 Python 环境中运行。
必须先进入下面的目录：训练和 play 的日志根目录由当前工作目录决定。

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
python scripts/rsl_rl/train.py --task Wheeled-Leg-v1 --headless --logger tensorboard --run_name stable_flat
```

PPO 默认训练 3000 次迭代，每 50 次保存一次；可以通过 `--max_iterations` 覆盖。
采用 32 步 rollout、[128, 128, 64] 网络、经验归一化、初始噪声 0.35、固定学习率 3e-4 和熵系数 1e-4。

**这次修改改变了观测维度、网络结构和归一化方式，需要从头训练；不要恢复旧 checkpoint。**
奖励权重也已变化，新旧 run 的奖励绝对值不能直接比较。
先观察超时率是否上升、触地终止率是否下降、平均 episode length 是否接近 2500 步，
再结合速度误差和 play 动作检查策略质量。偏航速度指令为零时，yaw 误差反映机身是否存在非预期旋转。
默认实验名为 `wheeled_leg_v1`。如使用 `--experiment_name` 覆盖，下面的工具命令也需使用新的实验名。

训练输出的绝对路径：

```text
/home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_leg_v1/
├── <YYYY-MM-DD_HH-MM-SS>_stable_flat/
│   ├── events.out.tfevents.*
│   ├── model_<iteration>.pt
│   ├── params/{env,agent}.{yaml,pkl}
│   ├── plots/mean_reward.png
│   └── play_joint_torque/<play_timestamp>/
│       ├── joint_torque.csv
│       ├── metadata.txt
│       ├── joint_torque_timeseries.png
│       └── joint_torque_rated_summary.csv
└── best/
    ├── model.pt -> ../<selected_run>/model_<iteration>.pt
    └── notes.md
```

图像、力矩 CSV 和 best 指针由下面对应命令生成。未指定 `--run_name` 时，run 目录只有时间戳。

## 绘制训练奖励

`plot_mean_reward.py` 的 `--task` 参数是实验目录名，不是 Gym 注册任务名。
默认选最新一个含 TensorBoard 事件文件的训练目录。

```bash
python tools/plot_mean_reward.py --task wheeled_leg_v1 --smoothing 20
```

也可以精确指定某次训练：

```bash
python tools/plot_mean_reward.py --run-dir logs/rsl_rl/wheeled_leg_v1/<run> --smoothing 20
```

## Play 和绘制关节力矩

将 `<run>`、`<iteration>` 替换为实际训练目录和 checkpoint 编号：

```bash
python scripts/rsl_rl/play.py --task Wheeled-Leg-Play-v1 --load_run <run> --checkpoint model_<iteration>.pt --num_envs 1
```

正常结束 play 后，CSV 保存至该训练目录的 `play_joint_torque/<play_timestamp>/joint_torque.csv`。
脚本启动和结束时会打印完整路径。使用该路径绘图：

```bash
python tools/plot_joint_torque.py logs/rsl_rl/wheeled_leg_v1/<run>/play_joint_torque/<play_timestamp>/joint_torque.csv
```

默认在 CSV 同目录生成力矩图和 RMS 等效额定扭矩汇总。

## 更新最佳策略并回放

`update_best_policy.py` 用来登记你选定的 checkpoint，不会自动评估或搜索最高奖励。

```bash
python scripts/rsl_rl/update_best_policy.py \
  --experiment wheeled_leg_v1 \
  --run <run> \
  --checkpoint model_<iteration>.pt \
  --task Wheeled-Leg-v1 \
  --play-task Wheeled-Leg-Play-v1 \
  --comment "速度跟踪与稳定性评估通过"

python scripts/rsl_rl/play.py --task Wheeled-Leg-Play-v1 --load_run best --checkpoint model.pt --num_envs 1
```

通过 best 回放时，力矩日志会回写到原始训练 run 目录；导出的 JIT/ONNX 策略按现有 play 脚本写入所加载 checkpoint 所在目录的 `exported/`。

## 教师学生蒸馏

蒸馏环境沿用 v1 的场景、动作、指令、奖励、终止和随机化设置。学生在环境中执行动作，
冻结的 PPO Actor 为访问到的状态提供确定性动作标签，以 Huber 损失训练两层 GRU 学生。
当前学生仍使用完整的 34 维观测，包含机身线速度；该配置用于学习时序策略，学生网络大于教师网络。

| 项目 | 学生 | 教师 |
| --- | --- | --- |
| 观测组 | `policy` | `teacher` |
| 输入维度 | 34 | 34，与当前 PPO Actor 一致 |
| 网络 | 两层 GRU，隐藏维度 247；MLP `[512, 256, 128]`，ELU | MLP `[128, 128, 64]`，ELU，无循环层 |
| 输出 | 6 维动作 | 6 维确定性动作标签 |
| 观测噪声 | 训练时开启，Play 关闭 | 始终关闭 |
| 参数及归一化统计 | 学生参数训练，学生统计持续更新 | 从 PPO Actor 加载，训练时固定 |

34 维输入顺序为：`base_lin_vel(3)`、`base_ang_vel(3)`、`projected_gravity(3)`、
`velocity_commands(3)`、`joint_pos(4)`、`joint_vel(6)`、`joint_effort(6)`、`actions(6)`。
PPO Critic 额外接收 7 维接触观测，共 41 维；教师不使用 Critic 输入或价值输出。
教师必须与 PPO Actor 的观测顺序、缩放、裁剪、动作顺序及隐藏层保持一致。
当前旧版 `best/model.pt` 如果仍是 28 维 Actor，则不能直接用于该 34 维配置。
应先使用当前 `Wheeled-Leg-v1` 训练兼容的教师；入口会明确拒绝旧维度检查点。

在本文工作目录、已激活 Isaac Lab Python 环境下启动新的学生实验：

```bash
python scripts/rsl_rl/train.py \
  --task Wheeled-Leg-Distillation-v1 \
  --teacher_checkpoint logs/rsl_rl/wheeled_leg_v1/<teacher_run>/model_<iteration>.pt \
  --headless --logger tensorboard --run_name student_gru
```

`--teacher_checkpoint` 接受相对或绝对路径，加载教师 Actor 和 Actor 归一化统计，
学生及其优化器从头初始化，迭代从 0 开始。不要将此参数与 `--resume` 或 PPO 的
`--init_checkpoint` 混用。默认日志目录为 `logs/rsl_rl/wheeled_leg_v1_distillation/`。
默认 32 步 rollout、3 个学习 epoch、固定学习率 3e-4、梯度裁剪 1.0，
每 16 步累计梯度后更新。当前 RSL-RL 的 `max_grad_norm` 只裁剪学生 MLP 参数，未覆盖 GRU 参数；
其 recurrent 蒸馏实现还会每步分离隐藏状态，因此 `gradient_length=16` 表示梯度累计，不能视为 16 步 BPTT。
训练时学生采样噪声为 0.1，Play 使用学生的确定性输出，并在 episode 结束时清空相应 GRU 状态。
旧 MLP 学生检查点不能直接恢复到此 GRU 结构，需要开始新的学生实验。

继续已有学生实验时，检查点已经包含教师、学生、两个归一化器和优化器状态：

```bash
python scripts/rsl_rl/train.py \
  --task Wheeled-Leg-Distillation-v1 \
  --resume --load_run <student_run> --checkpoint model_<iteration>.pt \
  --headless --logger tensorboard
```

回放学生并导出 JIT / ONNX：

```bash
python scripts/rsl_rl/play.py \
  --task Wheeled-Leg-Distillation-Play-v1 \
  --load_run <student_run> --checkpoint model_<iteration>.pt --num_envs 1
```

导出文件包含学生 GRU、学生 MLP 和学生归一化器，位于学生检查点旁的 `exported/`。
JIT 导出使用内部隐藏状态，episode 切换时需清空 `policy.hidden_state`（本机导出器未导出可调用的 GRU `reset()`）；
ONNX 使用显式 `h_in` / `h_out`，部署时需传递并在 episode 切换时清零隐藏状态。
训练记录中的 `Loss/behavior` 表示动作拟合误差，还需结合回放、速度跟踪和终止率评价学生控制效果。
CPU 回归检查可以在项目根目录运行，覆盖真实 RSL-RL rollout、教师固定、学生更新和检查点恢复：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_distillation.py tests/test_symmetry.py -q
```
