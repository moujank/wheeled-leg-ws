# Terrains 越障训练修订与使用说明

本次继续使用 `Wheeled-Leg-Terrains` / `Wheeled-Leg-Terrains-Play`，保留同一个以速度命令和高度扫描为输入的策略。优先解决持续前进、真实越障、课程升级和奖励冲突；困难地形采用保守速度。推荐从既有 Terrains 实验的 `model_5000.pt` 开始新的微调实验。

## 为什么奖励 94 仍会在困难地形摔倒

用户提供的末轮数据描述的是低速分布上的表现：速度上限约 0.25 m/s，平均命令约 0.10–0.14 m/s，速度阶段约 0.0005，高速专用惩罚始终为 0。线速度奖励 2.9394 接近权重 3 的上限，存活奖励约 1，不能据此推断已经学会高难度越障。

旧的正负对称速度采样包含低速和站立。初始速度范围 `[-0.25, 0.25]` 加上 10% 站立时，平均命令速度的期望约 0.1125 m/s，却要求达到 0.15 m/s 才升级速度课程，导致几乎无法升级。重采样还会带来转向和反向运动，而高度扫描主要覆盖前方。旧地形升级允许较短位移，容易把出生平台附近的运动当作通行能力。

地形平均等级也只是当前环境分配，不是通行成功率；平地的等级数字甚至不改变几何难度。末轮地形均值约 4.68，但最高等级占比仅约 1.76%，粗糙地形均值约 3.21。时间耗尽比例 96.75% 只说明多数回合没有触发跌倒终止，无法区分成功通过与原地停滞。

现有 TensorBoard 记录还显示训练中途发生过明显退化：第 5000 轮奖励约 89.15、地形均值约 6.09；第 5800–6200 轮地形均值接近 0，接触或倾斜终止大幅增加；随后从较简单地形恢复。末轮奖励更高并不代表保留了更好的越障技能。上述记录不能单独确定退化的唯一原因。

## 本次实现

- 命令改为整回合持续向世界 +X 前进，5% 环境练习站立。初始移动命令为 0.22–0.25 m/s，以 0.30 m/s² 逐步加速；航向反馈修正偏航。当前训练目标是前向通行，不能据此声称具备任意方向地形导航能力。
- 同一策略学习两种训练分布：每五个环境中的一个保留在 0–2 级，用于速度阶段 0.25 / 0.30 / 0.35 / 0.40 m/s；其余环境逐步学习困难地形。3 级及以上速度封顶 0.25 m/s。速度升级要求至少 8 秒有效移动、移动跟踪误差不超过 0.07 m/s、停滞比例低于 10%，连续 4 个合格回合升级，连续 2 个失败回合降级。
- 地形课程使用积分命令距离和实际前向位移。成功要求至少前进 2.6 m、达到命令距离的 65%、横向偏移小于 1.25 m、移动跟踪误差小于 0.10 m/s，并满足时长、姿态和无跌倒条件。连续 2 次成功升级，跌倒或充分暴露后的通行失败降级，站立回合中立。最高行成功后保留在最高行，避免导入器的越界随机回退。
- 添加前向进展、受支撑上升奖励，以及受命令约束的停滞惩罚。受支撑上升使用轮接触与支撑高度新高，限制反复弹跳刷奖励。越障时放宽俯仰、垂直运动、腿部偏离和部分对称约束，减少“必须抬腿但抬腿又受罚”的冲突；平稳运动仍有约束。
- 扩大腿部最终位置目标裁剪范围，为抬轮和改变支撑姿态留空间。保持 82 维 actor、89 维 critic、6 维动作以及观测顺序，兼容现有 Terrains 权重。
- PPO 使用固定学习率 `1e-4` 微调。通过 `--init_checkpoint` 加载网络和观测归一化，重新开始优化器、迭代计数与课程；归一化样本计数设上限，使其能适应新分布。
- Play 增加 `--terrain_level`、`--terrain_speed`、`--max_steps`，可在固定等级和速度下打印各地形完成尝试的通行统计。

轮速动作的 `scale=4` 是倍率，最终裁剪为 ±6 rad/s。轮半径约 0.0935 m，对应理想滚动速度约 0.56 m/s；执行器速度限制约 2π rad/s。当前执行器与动作限制不支持把 2 m/s 当作可行训练目标。20 cm 障碍也超过约 18.7 cm 的轮直径，需要主动腿部动作，不能预期仅靠滚动通过；本次修改尚未证明机器人能够稳定跨越所有最大难度障碍。

## checkpoint 选择依据

选择现有文件：

```text
logs/rsl_rl/wheeled_leg_terrains/2026-10-17_17-43-03/model_5000.pt
```

对 `model_5000.pt` 和 `model_7999.pt` 都做了真实 CPU 仿真评估：修订后的 Play 环境，seed=42，24 个环境，固定第 7 行（行号 0–9）、0.25 m/s、2600 个控制步（约 20.8 秒）。结果为满足上述通行条件的成功次数 / 已完成尝试次数：

| 地形 | model_5000 | model_7999 |
|---|---:|---:|
| flat | 3/3 | 3/3 |
| slope | 3/3 | 3/3 |
| slope_inverted | 0/3 | 3/3 |
| pyramid_stairs | 3/3 | 0/12 |
| pyramid_stairs_inverted | 1/10 | 0/8 |
| rough | 0/9 | 0/15 |
| wave | 3/3 | 0/5 |
| discrete_obstacles | 2/2 | 1/2 |

第 5000 轮在上台阶、波浪和离散障碍上更适合作为越障微调起点，因此优先采用它。它在下坡、下台阶和粗糙地形仍有短板。这是单个种子、小样本短评估，失败较快的环境会产生更多尝试，不能将分母视为等量独立测试，也不能声称该模型在所有地形上更好。成功表示稳定向前通行至少 2.6 m，不能保证碰到了地图中的每一块随机障碍。

评估使用修订后的环境，包括更宽腿部目标范围和持续前向命令；它衡量当前微调起点的表现，并非对历史训练环境的精确复现。无需先回到平地策略重新训练。这里也没有足够证据要求拆成两个网络。

## 启动新训练

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
/home/moujank/miniconda3/envs/env_isaaclab/bin/python scripts/rsl_rl/train.py \
  --task Wheeled-Leg-Terrains --headless --num_envs 4096 --max_iterations 8000 \
  --init_checkpoint logs/rsl_rl/wheeled_leg_terrains/2026-10-17_17-43-03/model_5000.pt
```

这里使用 `--init_checkpoint`，启动新的日志目录。不要同时传 `--resume`；旧优化器状态与旧课程进度不适合此次任务分布和奖励修改。新奖励包含新增项，绝对值不能直接与旧的 94 分比较。

## 固定条件验收

将下面的 checkpoint 路径替换为新训练生成的实际文件：

```bash
/home/moujank/miniconda3/envs/env_isaaclab/bin/python scripts/rsl_rl/play.py \
  --task Wheeled-Leg-Terrains-Play --num_envs 48 \
  --checkpoint logs/rsl_rl/wheeled_leg_terrains/新实验目录/model_N.pt \
  --terrain_level 7 --terrain_speed 0.25 --max_steps 25000
```

建议分别评估行 0、4、7、9，困难行先固定 0.25 m/s；0.35 m/s 仅在容易行验证。重复使用 `--seed 42`、`--seed 43`、`--seed 44`，每次固定一种设置，避免同时改变速度与地形难度。25000 步对应约 200 秒仿真时间。加 `--headless` 可不显示界面。

当前 `--terrain_speed` 支持 `(0.20, 0.40]` m/s，通过正常加速斜坡达到目标。较低速度没有纳入此通行验收带，避免 20 秒回合中命令暴露不足。指定固定等级后课程不会移动地形行。

训练时重点观察：

- `Curriculum/terrain_levels/<地形>_attempts`、`successes`、`top_attempts`、`top_successes` 是累计计数。用一段时间的计数差求成功率；分母为零时表示未覆盖，不能报告 100% 成功。
- 平均地形等级现为 `Curriculum/terrain_levels/mean_level`；分布均值继续在 `Curriculum/terrain_distribution/...`。均值只是分配信息。
- `Metrics/base_velocity/moving_tracking_error_mean`、`moving_time_s`、`command_distance_m`、`stall_fraction` 用于区分跟踪、有效移动和停滞。
- 结合各地形固定评估成功率、接触/倾斜终止率判断效果。现有 `base_contact` 终止包含机身、Hip 和 Knee，不能只解释成机身触地。

## 已完成的验证与边界

- 项目回归测试 57 项全部通过。覆盖命令加速度、采样、部分重置、速度与地形升级、失败降级、最高行保留、固定评估等逻辑。
- 已从 `model_7999.pt` 加载权重，在真实 CPU Isaac Sim 环境完成 8 个环境、256 个 transition、一次 PPO 更新；确认新管理器、奖励、网络尺寸和 `1e-4` 学习率可运行。该测试模型不是推荐 checkpoint。
- 两个原有 checkpoint 均完成上述固定条件真实仿真评估，实际复现了困难地形通行失败。
- 尚未运行修订后的完整 8000 轮训练，因此这里只能确认实现与短程运行正常，不能宣称新策略已经解决高难度越障。后续应依据固定条件、多种子评估挑选新 checkpoint。

测试命令：

```bash
cd /home/moujank/project/wheeled-leg-ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/moujank/miniconda3/envs/env_isaaclab/bin/python \
  -m pytest -q modules/wheeled_train/wheeled_leg/tests
```
