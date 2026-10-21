# 训练与回放分析工具

[训练工程](../README.zh.md) · [工作区文档](../../../../docs/README.md)

工作区原 `tools/` 已合并到本目录。力矩绘图以本目录增强版本为基础，奖励绘图从根目录迁入；两者均可在无图形界面的环境中导出图片。

## 环境与路径

依赖：Python 3.10+、`matplotlib`；奖励分析还需要 `tensorboard`。工具不依赖 Isaac Lab 运行时，安装绘图依赖后可独立使用：

```bash
python3 -m pip install matplotlib tensorboard
cd modules/wheeled_train/wheeled_leg
```

以下命令均从训练工程目录执行。也可从任意目录通过脚本完整路径运行；奖励工具的默认 `logs/rsl_rl/` 根据脚本位置解析，显式输入和输出路径相对于当前工作目录。

## 关节力矩

```bash
python3 tools/plot_joint_torque.py /path/to/joint_torque.csv
python3 tools/plot_joint_torque.py /path/to/joint_torque.csv \
  --start-step 100 --end-step 1000 --joints HipL HipR --env-index 0 \
  --rated-method rolling-rms --rolling-window-seconds 0.5 \
  --output /path/to/torque.png --summary-output /path/to/torque_summary.csv
```

CSV 必须包含 `step`、`sim_time_s`、`env_index` 及关节列，列顺序可变化。默认读取全部关节；多环境数据必须使用 `--env-index` 指定环境。筛选后的时间必须严格递增，数值必须有限。

| 选项 | 功能 |
| --- | --- |
| `--start-step` / `--end-step` | 按闭区间筛选回放 step |
| `--joints NAME ...` | 选择关节并指定绘图顺序 |
| `--env-index` | 选择环境，防止混合不同机器人轨迹 |
| `--rated-method rms` | 默认：全段 RMS 及正负参考线 |
| `--rated-method percentile --rated-percentile 95` | 绝对力矩百分位及参考线 |
| `--rated-method rolling-rms --rolling-window-seconds 1` | 按推断采样周期绘制滚动 RMS |
| `--rated-method none` | 仅绘制原始曲线，保留旧根目录工具的绘图能力 |
| `--output` / `--summary-output` | 指定图片与统计 CSV 路径，自动创建父目录 |

默认在输入 CSV 同目录输出 `joint_torque_timeseries.png` 和 `joint_torque_rated_summary.csv`。统计包含样本数、RMS、平均绝对力矩、P95 与峰值。RMS 是仿真载荷的等效连续扭矩需求，电机选型还需考虑效率、热模型和安全裕量。

## 奖励与 TensorBoard 标量

```bash
python3 tools/plot_mean_reward.py --task wheeled_base_locked_hard --list-tags
python3 tools/plot_mean_reward.py --task wheeled_base_locked_hard --smoothing 20
python3 tools/plot_mean_reward.py --run-dir /path/to/run \
  --tag Train/mean_reward --start-step 100 --end-step 5000 --smoothing 20 \
  --output /path/to/reward.png --csv-output /path/to/reward.csv
```

当前 RSL-RL 日志一般没有 `Train/mean_reward`，而是分别记录
`Episode_Reward/<term>`。因此不指定 `--tag` 时，工具会自动把所有
`Episode_Reward/*` 在相同训练 step 上相加，并绘制派生的
`Episode_Reward/total`；仍可用 `--tag Episode_Reward/mirrored_legs` 绘制单项奖励。

`--task` 使用实验目录名，自动按目录名选择最新且含 TensorBoard 事件文件的 run，跳过 `best/` 和非事件目录。`--run-dir` 可精确选择某次训练，并优先于 `--task`。

工具加载所选目录的全部事件文件，沿用 TensorBoard 的恢复训练清理规则；剩余重复 step 保留 wall time 最新的记录，并按 step 排序。支持任意 scalar tag、标签列表、迭代闭区间筛选、因果滑动平均和图上 Top-5 峰值标注。CSV 导出筛选后的 step、wall time、tag、原始值和平滑值。

默认图像写入 `<run>/plots/mean_reward.png`，可用 `--output-name` 改文件名，或用 `--output` 指定完整路径。平滑在所选区间内计算。完整参数见两个工具的 `--help`。

20 cm 越障与严格跳跃分别使用 `logs/rsl_rl/wheeled_leg_obstacle_jump/` 和 `logs/rsl_rl/wheeled_leg_jump/`。奖励工具的 `--task` 支持实验目录名，也支持这两个任务的训练/PLAY Gym ID；训练与回放的 `--task` 填 Gym 注册名称。完整命令见 [越障与跳跃说明](../OBSTACLE_JUMP.zh.md)：

```bash
python3 tools/plot_mean_reward.py --task wheeled_leg_obstacle_jump --smoothing 20
python3 tools/plot_mean_reward.py --task wheeled_leg_jump --smoothing 20
python3 tools/plot_joint_torque.py \
  "logs/rsl_rl/wheeled_leg_obstacle_jump/<run>/play_joint_torque/<timestamp>/joint_torque.csv"
```

严格跳跃可单独分析 `Episode_Reward/jump_flight`、`Episode_Reward/jump_landing` 和 `Episode_Reward/goal_success`。课程和实际成功统计通过 `--list-tags` 查询，再传入 `--tag` 绘图。`update_best_policy.py` 对这两个默认实验名自动填入训练/PLAY ID；完整命令见 [越障与跳跃说明](../OBSTACLE_JUMP.zh.md)。
