# 多地形轮足稳定运动：Terrain Balance

本任务使用 `asset/wheeled_leg_v2.py` 中的 `WHEELED_CFG`，目标是在多种温和地形中学习
稳定站立、前后运动和小角速度转向。环境配置独立定义在 `terrain_balance_env_cfg.py`，
没有继承版本化任务配置，便于后续单独调整。

| 用途 | Gym 任务名 | 配置类 |
| --- | --- | --- |
| 训练 | `Wheeled-Leg-Terrain-Balance` | `WheeledLegTerrainBalanceEnvCfg` |
| Play | `Wheeled-Leg-Terrain-Balance-Play` | `WheeledLegTerrainBalanceEnvCfg_PLAY` |
| PPO | 独立实验 `wheeled_leg_terrain_balance` | `TerrainBalancePPORunnerCfg` |

## 基础机器人和控制方案

- 资产来自 `asset/wheeled_leg_v2.py`，任务内使用 `.replace()` 设置初始姿态和仿真速度限制。
- 初始姿态：HipL/HipR 为 ±0.65 rad，KneeL/KneeR 为 ±1.30 rad。
  根部高度为相对地形 origin 的 0.58 m；地形 origin 包含当地高度。
- 保留资产的腿部位置 PD 和轮毂速度控制，以及原有扭矩上限。
  使用 `velocity_limit_sim=2π rad/s` 设置各执行器的仿真速度上限。
  本任务单独将髋/膝阻尼从 1.5 调到 2.5，髋/膝刚度仍为 20/30，轮毂阻尼仍为 1.5。
- 四个腿关节使用位置目标，动作缩放 0.25；两个轮毂使用速度目标，动作缩放 4。
  以下限幅作用于缩放并加默认姿态后的最终目标，而非原始策略动作：

| 关节 | 目标限幅 |
| --- | --- |
| HipL / HipR | [0.2, 1.1] / [-1.1, -0.2] rad |
| KneeL / KneeR | [0.8, 1.8] / [-1.8, -0.8] rad |
| HubL / HubR | ±6 rad/s |

- Actor 为 28 维本体状态观测：机身线速度、角速度、重力方向、速度指令、四个腿关节相对位置、
  六个关节相对速度和上一次动作。轮毂累计转角不进入观测。
- Critic 包含完整 Actor 状态和接触信息，关闭观测噪声；开启经验归一化。
- 基础方案不使用地形高度扫描，依靠本体状态和接触反馈应对温和起伏。
  较高台阶、间隙和需要提前规划的障碍，应在此基础上增加地形感知后再训练。
- 物理频率 500 Hz、动作频率 125 Hz；训练回合最长 20 s（2500 个控制步）。
- 指令范围：前后速度 ±0.45 m/s，侧向速度 0，偏航角速度 ±0.25 rad/s。
  20% 的采样指令为站立，速度指令每 20 s 重采样，回合复位时也重新采样。

## 地形和课程

地形配置：`terrains/terrain_balance.py`。每块地形 8×8 m，8 行难度、20 列地形类型，
所有训练环境从第 0 行开始。课程模式第 0 行的生成难度位于最低的 1/8 区间，
所以初始地形是温和地形，不保证全部为零起伏。

| 地形 | 比例 | 配置范围（随难度增加） |
| --- | ---: | --- |
| 平地 | 30% | 平面 |
| 缓坡 / 反向缓坡 | 各 15% | 坡度比 0–0.12，约 0–6.8° |
| 浅随机起伏 | 20% | 采样幅度 0–1.5 cm，采样间距 0.25 m，插值后离散到 2.5 mm |
| 波浪 | 10% | `amplitude_range=(0, 0.04)`，2 个波周期 |
| 低台阶 / 反向低台阶 | 各 5% | 单级高度 0.5–4 cm，踏面宽 0.5 m |

内置随机起伏生成器不使用 difficulty 参数，因此此处提供了随难度放大起伏幅度的函数。
难度课程通过 `mdp/curriculums.py` 实现，在复位前评估上一回合：

- 无触地/倾斜失败，并因达到时限或地形边界结束；
- 平面指令速度至少 0.1 m/s，实际回合至少持续 5 s；
- 相对地形 origin 的平面位移超过自适应门槛；
- 平均线速度指数跟踪得分 >0.75，平均偏航速度指数跟踪得分 >0.7；
- 同时满足时升一级；发生失败时降一级；其他情况保持当前难度。

跟踪得分按实际回合时长和奖励权重归一化，不依赖奖励绝对大小。
指令在回合中保持不变。若指令平面速度为 v、偏航速度为 ω、实际时长为 T，
理想圆弧的终点距离为 `v*T*abs(sinc(ω*T/(2π)))`，其中 sinc 使用 PyTorch 定义。
位移门槛为该距离的 60%，限制在 0.75–2 m，兼顾慢速和转弯回合。
实际位移以地形 origin 为参考，复位位置的 ±0.05 m 扰动远小于最低门槛。
站立回合不会仅因存活升级，失控滑行也不会仅因位移升级。
小范围绕圈若净位移不足 0.75 m，也不会升级。若以后改为回合内多次重采样指令，
应同时替换这一基于恒定指令的距离估计。
通过最高难度后，Isaac Lab 会把该环境重新分配到随机难度，继续复习。
机身到地形块边缘不足 0.25 m 时进行截断复位，避免进入邻接地形影响课程评估。

## 奖励和随机化

线速度跟踪权重为 3、容差参数为 0.25 m/s；偏航速度跟踪权重为 1；存活奖励权重为 1。
保留触地、关节限位、能耗、加速度、动作变化惩罚；增加腿关节速度和左右腿镜像约束。
姿态惩罚权重为 -1.5，腿部默认姿态偏差权重为 -0.2，避免过强约束妨碍坡面与低台阶适应。
机身倾斜超过 60°，或机身/髋/膝触地时终止。

针对 8000 轮训练后仍存在的腿部偏置和机身上下抖动，调整如下：

| 项目 | 原值 | 当前值 / 含义 |
| --- | ---: | --- |
| 机身局部 z 速度平方 | -2 | -6，抑制竖直运动 |
| 机身 roll/pitch 角速度平方 | -0.2 | -0.5，抑制姿态摆动 |
| 腿关节速度平方 | -0.005 | -0.02 |
| 镜像位置偏差 L1 | -0.1 | -0.4，每对关节允许 0.03 rad 死区 |
| 镜像速度偏差平方 | 无 | -0.01，惩罚 `qdot_L + qdot_R` |
| 腿部动作变化平方 | 无 | 额外 -0.07，与全动作 -0.03 叠加 |
| 腿关节加速度平方 | 无 | 额外 -5e-7，与全关节 -1e-7 叠加 |

位置和速度镜像约束只选择对应的髋/膝关节，使用左右相反的轴向符号；
轮毂速度不受镜像约束，仍可差速转向。镜像约束为软惩罚，允许起伏地形需要的腿部差异。
未引入固定世界坐标下的机身高度惩罚，因为机器人通过坡面和台阶时需要随地形改变高度。
z 速度惩罚仍会对必要的升降产生一定代价，因此需结合坡面、台阶通过表现检查当前权重。

训练摩擦系数轻度随机化，机身质量增量为 0–0.5 kg，复位关节位置为默认值的 0.95–1.05 倍，
初始位姿和速度采用小范围扰动；关闭周期推扰。先验证多地形稳定性，再增加推扰和地形幅度。

PPO：32 步 rollout，Actor/Critic `[128, 128, 64]`，初始噪声 0.35，熵系数 1e-4，
固定学习率 3e-4，clip 0.15，每轮 3 个学习 epoch，默认 5000 次迭代，每 50 次保存。

## 训练、输出和回放

在已经安装 Isaac Lab 和本地任务扩展的 Python 环境中执行：

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg

python scripts/rsl_rl/train.py \
  --task Wheeled-Leg-Terrain-Balance \
  --num_envs 2048 \
  --max_iterations 5000 \
  --run_name baseline \
  --headless
```

从头训练新策略。输出根目录由工作目录决定，按上述命令运行时为：

```text
/home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_leg_terrain_balance/<时间戳>_baseline/
```

该目录包含 TensorBoard 事件、`model_<iteration>.pt` 和 `params/`。
重点观察 `Curriculum/terrain_levels`、速度误差、触地/倾斜终止和确定性回放。
新增 `Curriculum/terrain_distribution/` 下的指标：

- `flat_mean_level`、`slope_mean_level`、`slope_inverted_mean_level`、`rough_mean_level`、
  `wave_mean_level`、`low_steps_mean_level`、`low_steps_inverted_mean_level`：各地形当前平均等级；
- `nonflat_mean_level`：排除平地后的平均等级；
- `level_0_fraction`、`top_level_fraction`：当前最低行/最高行的环境占比。

这些指标是当前地形分配的统计，不能直接当作回合成功率。
边界截断也是 time_out，因此本任务中平均回合长度不一定接近 2500 步；
稳定走到边界后提前复位是正常行为。不要用较高 time_out 比例单独判断收敛。

Play 默认 50 个环境、固定前进速度 0.4 m/s，从全部难度行随机分配初始地形，
关闭难度升级、观测噪声和物理随机化。保留 curriculum 地形生成模式，以保持各难度行有序。
多地形检查建议至少 20 个环境，覆盖所有地形列；单环境仅用于局部调试。
环境 0 总是平地，与当前平均等级无关。50 个环境时，按环境编号大致分配为：
0–14 平地、15–22 坡、23–29 反向坡、30–39 随机起伏、40–44 波浪、45–47 台阶、48–49 反向台阶。
可用 `env.viewer.origin_type=asset_root env.viewer.env_index=45 env.viewer.asset_name=robot`
跟随台阶环境；配合 `env.torque_log_env_index=45` 记录同一个机器人的力矩。

```bash
python scripts/rsl_rl/play.py \
  --task Wheeled-Leg-Terrain-Balance-Play \
  --load_run <时间戳>_baseline \
  --checkpoint model_<iteration>.pt \
  --num_envs 50
```

现有策略的观测/动作维度、动作缩放及网络结构均兼容当前配置，可继续训练：

```bash
python scripts/rsl_rl/train.py \
  --task Wheeled-Leg-Terrain-Balance \
  --num_envs 2048 \
  --resume \
  --load_run 2026-10-13_17-03-10 \
  --checkpoint model_7999.pt \
  --max_iterations 3000 \
  --run_name symmetry_smooth_v2 \
  --headless
```

已核对该 run 下最新文件为 `model_7999.pt`；此版本 runner 使用最后一次迭代的编号保存。
`--max_iterations 3000` 在 resume 时表示追加 3000 轮。
加载 checkpoint 恢复网络、归一化统计和优化器，但不恢复每个环境的地形等级；
新训练环境仍从第 0 行开始。当前奖励和阻尼已改变，需要继续优化后再比较表现，
新旧总奖励也不能直接比较。Play 使用确定性策略，优先以此确认可视化抖动，
因为训练使用带探索噪声的动作，末期 std≈0.12 对腿目标仍对应约 0.03 rad 的采样噪声。

比较旧 checkpoint 原有表现时应使用原来的阻尼；例如在 Play 命令中追加
`env.scene.robot.actuators.hip_act.damping=1.5 env.scene.robot.actuators.knee_act.damping=1.5`。
对更新后的策略检查平地站立（x/y/yaw=0）、直行、转弯以及各地形高难度行的表现；
可用 `env.scene.terrain.max_init_terrain_level=7` 允许采样全部 8 行，
但该选项指定的是随机采样上限，不保证所有环境都在第 7 行。

实际训练记录 `2026-10-13_17-03-10` 的 TensorBoard 显示：
前 100 轮平均等级 0，第 1000–1099 轮为 1.96，第 4000–4099 轮为 4.63，
第 7900–7999 轮为 4.66。因此旧课程已经升级，单看平地环境无法判断课程是否生效；
旧日志没有按地形统计，不能据此确认台阶和随机起伏各自达到了什么等级。

首次训练早期如果只能通过低难度，可限制回放的初始行：

```bash
python scripts/rsl_rl/play.py \
  --task Wheeled-Leg-Terrain-Balance-Play \
  --load_run <时间戳>_baseline \
  --checkpoint model_<iteration>.pt \
  --num_envs 50 \
  env.scene.terrain.max_init_terrain_level=0
```

## 奖励图、关节力矩和最佳策略

```bash
python tools/plot_mean_reward.py --task wheeled_leg_terrain_balance --smoothing 20
```

Play 自动记录环境 0 的全部关节施加力矩，保存到所加载训练目录的
`play_joint_torque/<play_timestamp>/joint_torque.csv`；用启动/退出时打印的路径绘图：

```bash
python tools/plot_joint_torque.py \
  logs/rsl_rl/wheeled_leg_terrain_balance/<run>/play_joint_torque/<play_timestamp>/joint_torque.csv

python scripts/rsl_rl/update_best_policy.py \
  --experiment wheeled_leg_terrain_balance \
  --run <run> \
  --checkpoint model_<iteration>.pt \
  --task Wheeled-Leg-Terrain-Balance \
  --play-task Wheeled-Leg-Terrain-Balance-Play \
  --comment "多地形速度跟踪与稳定性评估通过"

python scripts/rsl_rl/play.py \
  --task Wheeled-Leg-Terrain-Balance-Play \
  --load_run best --checkpoint model.pt --num_envs 50
```

最佳策略工具登记你选定的 checkpoint，不会自动评估地形通过率。
不同地形应分别检查，不能仅按总奖励选择策略。当前配置为基础训练方案，
实际可通过的难度和动作稳定性仍需要在仿真中训练、回放验证。
