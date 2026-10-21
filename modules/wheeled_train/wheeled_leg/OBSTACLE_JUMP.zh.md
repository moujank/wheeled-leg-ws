# wheeled_leg_v2：20 cm 越障与严格跳跃

[训练工程](README.zh.md) · [分析工具](tools/README.md)

两个任务独立定义场景、动作、观测、奖励、终止和课程，直接继承 `ManagerBasedRLEnvCfg`，不继承 terrain-balance。`v2_wheeled_leg_obstacle_jump.py` 负责任意方式越障，`v2_wheeled_leg_jump.py` 负责原地跳跃到移动跳跃越障的训练。

## 任务入口

| 任务 | 训练 Gym ID | 回放 Gym ID | 默认实验目录 |
| --- | --- | --- | --- |
| 任意方式越过 20 cm 障碍 | `Wheeled-Leg-Obstacle-Jump` | `Wheeled-Leg-Obstacle-Jump-Play` | `logs/rsl_rl/wheeled_leg_obstacle_jump/` |
| 严格跳跃并到达目标 | `Wheeled-Leg-Jump` | `Wheeled-Leg-Jump-Play` | `logs/rsl_rl/wheeled_leg_jump/` |

保留原有 Gym ID，避免已有训练、回放和工具命令失效。两个任务训练各自的策略和 PPO 实验。

## 独立资产与控制

`asset/wheeled_leg_v2_agile.py` 新增 `WheeledLegAgileAssetCfg(ArticulationCfg)`，显式配置现有 `wheeled_leg_v2.usd`、刚体属性、求解器、初始姿态和执行器，不复制 terrain-balance 的资产配置。

初始 root z 为 0.58 m，髋关节为 ±0.65 rad，膝关节为 ±1.30 rad。动作是四个髋/膝位置目标和两个轮毂速度目标；腿动作缩放 0.65，轮动作缩放 5.0，允许更大的蹲伏、伸腿和轮速变化。

| 设置 | 越障 | 严格跳跃 |
| --- | --- | --- |
| 力矩上限 | 150 N·m | 150 N·m |
| 腿关节仿真速度上限 | 10 rad/s | 10 rad/s |
| 轮关节仿真速度上限 | 12 rad/s | 12 rad/s |
| 髋 stiffness / damping | 120 / 6 | 140 / 4 |
| 膝 stiffness / damping | 180 / 8 | 220 / 5 |
| 轮 stiffness / damping | 0 / 3 | 0 / 2 |
| 求解器 position / velocity iterations | 8 / 4 | 8 / 4 |

速度上限和伺服增益是任务的仿真设置，尚未验证与实机电机性能一致。增加控制能力为学习蹲伏和瞬时伸腿提供空间，最终能否完成任务仍需通过训练和回放评估。

## 任意方式越障：主动上台阶、持续稳定、急停和抗推（当前配置）

上一版长时稳定配置使用髋/膝阻尼 12/16、全程角速度和动作变化惩罚、低速 0.25–0.45 m/s，并从初期加入随机停车与推力。这些设置可能压制上台阶所需的屈伸和主动俯仰。本版降低腿部阻尼、提高位置跟踪刚度，增加真实前进与受支撑抬升奖励，并仅在近处向上台阶放宽相关代价；保留落地恢复、停车和长回合要求。没有新增训练曲线，具体改善幅度需要新实验确认。

地形仍只有倒金字塔台阶和随机高度方格，各占 50%。8 行课程把单级台阶/方格最大高度从 4 cm 提升到 20 cm；PLAY 为单行满难度地形。随机方格有平坦重置区和连续底板；倒金字塔从中央低平台出发，外沿相对中央平台的累计高度可能大于 20 cm。

回合上限为 **30 s**，机器人在相对地形原点 x=0 与 x=2.6 m 之间持续往返。到达路点不重置、不结束回合；到达 x 的 ±0.3 m 范围后指令归零，需横向偏差小于 0.7 m、双轮落地、倾角小于 20°、三维线速度小于 0.2 m/s、三维角速度小于 0.6 rad/s，连续保持 0.6 s 才奖励一次。随后目标切换，先原地转向，再继续前进。yaw 指令最大 ±0.8 rad/s，前进速度按航向对齐程度缩小，扫描器面向下一段前进方向。

速度每 **2.5–4 s** 重采样，在 **0.45–0.65 m/s** 慢速段与 **0.75–1.0 m/s** 快速段间交替，不做斜坡平滑，保留突然加减速。随机零 twist 指令的概率最终为 **15%**，保持至下一次采样；路点停车始终生效。随机停车和推力共用渐增时序：前 **40000 控制步**不加入，之后 **80000 步**从零线性增至满量。若每轮采集 32 步，约前 1250 轮专注前进和上台阶，第 3750 轮达到完整鲁棒性训练强度。时序按本次进程的全局控制步计数，重启会重新开始；快慢切换和路点制动不受此延迟影响。

局部台阶识别使用现有 54 条射线，转换到机身 yaw 坐标，以双轮较低轮底高度为参考。仅选择轮组中心前后 -0.08 至 0.65 m、横向 ±0.45 m 内的有效向上高度差（2.5–35 cm），并要求前进指令 >0.15 m/s、yaw 指令绝对值 <0.7 rad/s、机身倾角 <45°。高度差从约 2 cm 至 6 cm 渐增攀爬权重。远处粗糙地形、下台阶、无效射线、停车和转向都不触发这一放松，不用整片扫描的粗糙程度代替实际接近台阶。

| 项目 | 当前设置与目的 |
| --- | --- |
| 越障执行器 | 髋 stiffness/damping=120/6，膝=180/8，轮=0/3；允许主动屈伸，150 N·m 上限与动作尺度不变 |
| 前进进度 | 权重 1.5 → **4.0**，仍按当前前向命令速度封顶，返程有效，停车不能刷前进奖励 |
| 受支撑抬升 | 新增权重 **4.0**；每个轮子有接触向上支撑力 >5 N、接地且正在前进、倾角 <30°时，只奖励本回合新的轮底支撑高度；不奖励原地抖腿或纯腾空 |
| 停滞 | 新增权重 **-1.0**；直行指令 >0.2 m/s 时，前向速度低于 0.15 m/s 逐渐计罚，停车和显著转向不计罚，代价封顶 |
| 稳定路点 | 权重 150，0.008 s 控制步下每次奖励 1.2，单次脉冲 |
| XY / yaw 跟踪 | 权重 4.5 / 1.0；XY std 从 0.35 放宽为 **0.5 m/s**，容许短暂蹲伏和抬升时的速度偏差 |
| 超速 | 权重 -1.5 → **-0.75**，普通余量 0.2 m/s，攀爬时渐增至 0.4 m/s；停车另有专门制动代价 |
| 急停 | 零 twist 指令期间惩罚三维线速度与角速度，权重 -2.0 |
| 姿态 | 权重 -3.0；roll 死区 8°，pitch 死区在攀爬时从 12°渐增至 24°，pitch 代价渐减至 30%；roll 约束保留，粗糙地形仍有至少 35% 基础权重 |
| 横滚/俯仰角速度 | 权重 **-0.2**；普通死区 0.3 rad/s，攀爬时仅 pitch 死区渐增至 1.5 rad/s、代价渐减至 30% |
| 双轮接地弹跳 | 权重 -1.0，垂直速度死区 0.2 m/s；攀爬时再减至 10%，腾空关闭；平地与恢复阶段仍抑制无效弹跳 |
| 动作变化 | 权重 -0.04 → **-0.015**；攀爬时四个腿目标变化代价渐减至 30%，轮速目标平滑始终保留 |
| 关节加速度 | 保留 -2e-8，抑制高频反向动作 |

受支撑抬升按两轮各自的历史最高接地轮底高度计数，累计超过 1 cm 才结算增量，防止接触噪声；每次每轮增量最多计 22 cm。该项返回高度增量/控制周期，因此奖励权重 4.0 表示双轮平均抬升 20 cm 时约奖励 0.8。停车抬升也会消耗高度记录，不能先抬高再切换命令补领奖励；回到旧高度反复弹跳不能重复获得该奖励。它鼓励真实台阶支撑，不预设固定左右相位，也不禁止必要的小跳。

原有平地垂直速度/腿活动代价保留。机身碰撞、倾角超过 60°、超出地形边界立即结束失败回合。指令、动作顺序及 **82/89 维 Actor/Critic 观测**保持兼容。

随机推力仍由 `mdp/stability.py::RandomBodyForcePulse` 实现：每环境独立等待 4–7 s，在机身施加世界水平面随机方向的力，持续 0.08–0.20 s，到期主动清零，部分环境重置不影响其他环境。满强度 **F=机器人总质量×(0.5–3.0 m/s²)**，每回合最初 2 s 不推。PLAY 从开始启用完整随机停车概率和满强度推力，便于评估；可用 `env.events.body_push=null` 关闭推力做对照。

地形升级使用 **terrain_success**：本回合至少完成一个稳定路点，并且没有失败终止；正常回合仍持续至 30 s 超时。连续两次成功升一级，失败降一级，最高级不回绕。这样不再把返程转向是否完成作为接触更高台阶的前提。另保留 **long_episode_success**：至少完成两个稳定路点、存活至少 20 s 且没有失败终止。顶层 outcomes 使用 terrain_success，不能直接与上一版的往返成功率或更早的短回合单程成功率比较。

`Metrics/base_velocity/` 包含 `tilt_deg`（平均倾角）、`roll_pitch_speed`、`stop_fraction`、`stop_speed`、`stop_position_drift`、`stop_settled_fraction`、`waypoints`、`long_episode_success`；本版增加 `terrain_success`、`climb_fraction`（局部攀爬权重的回合均值）和 `moving_stall_fraction`（停滞代价的回合均值）。`climb_fraction` 表示接近上台阶的上下文，不是通过率。停车指标包含制动过程；没有停车样本时停车均值为零，需结合 stop_fraction。原有按实际回合采样数统计的 episode_vel_xy_error、垂直速度和双轮腾空比例保留。

建议启动独立实验从头训练。已经明显保守的 checkpoint 可做迁移对照，但不建议仅靠继续原实验期待自动改变步态：

```bash
conda activate env_isaaclab
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
python scripts/rsl_rl/train.py --task Wheeled-Leg-Obstacle-Jump \
  --headless --logger tensorboard --num_envs 2048 --max_iterations 12000 \
  --run_name obstacle_active_stairs --seed 43
```

迁移实验可加 `--init_checkpoint <旧越障model路径>`，加载兼容权重和观测统计，重新建立优化器；这不能保证消除旧策略的保守局部最优。运行中的训练进程不会自动加载源码修改。

```bash
# 满难度 20 cm、30 s 往返、突然变速/急停、满强度推力
python scripts/rsl_rl/play.py --task Wheeled-Leg-Obstacle-Jump-Play \
  --load_run '<新run>' --checkpoint model_11999.pt --num_envs 16

# 同一 checkpoint 关闭推力做对照
python scripts/rsl_rl/play.py --task Wheeled-Leg-Obstacle-Jump-Play \
  --load_run '<新run>' --checkpoint model_11999.pt --num_envs 16 \
  env.events.body_push=null

# 更长时间验证，路点继续循环
python scripts/rsl_rl/play.py --task Wheeled-Leg-Obstacle-Jump-Play \
  --load_run '<新run>' --checkpoint model_11999.pt --num_envs 16 \
  env.episode_length_s=60.0
```

分别检查两种地形的顶层通过率、terrain_success 与 long_episode_success、碰撞/倾覆、平均回合秒数（步数×0.008）、停滞与实际前进速度，并结合视频确认腿部抬升和支撑。上台阶时可以暂时俯仰，跨过台阶后应恢复；不能单看总奖励或把高腾空比例当作步态提升。新增奖励、增益和课程是训练设计，尚未证明已经学会 20 cm 上台阶。

## 2026-10-15 早期越障训练诊断（历史配置）

以下记录对应本次长时稳定性调整之前的单程任务；18 s、2.8 m 成功即重置和旧阻尼仅用于说明当时的实验。当前参数以上节为准。

分析对象为 `2026-10-14_17-25-22`，2048 环境、8000 轮。TensorBoard 中以下是各窗口的均值；这些是当前课程混合分布上的数据，不是固定 20 cm 评测。

| 轮次窗口 | 总奖励 | 回合成功比例 | 机身接触 | 越界 | 平均地形级别 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1300–1500 | 20.86 | 95.17% | 4.49% | 0.16% | 4.81 |
| 1800–2000 | 15.87 | 54.11% | 30.86% | 14.55% | 5.40 |
| 3900–4100 | 13.35 | 43.58% | 15.21% | 35.49% | 4.58 |
| 5100–5300 | 13.90 | 53.78% | 35.79% | 3.78% | 5.01 |
| 7000–7200 | 19.07 | 88.96% | 5.51% | 3.53% | 5.00 |
| 7800–7999 | 16.78 | 72.17% | 18.94% | 5.37% | 5.24 |

原配置允许 0.5–0.9 m/s 命令，却在 1.5 m/s 才封顶前进奖励，而且通过奖励较大、速度跟踪较弱。策略有快速冲过目标的激励；原配置没有区分平地与障碍处的腿部活动，因此混合走跳并不违反目标。仅凭奖励曲线不能认定执行器震荡，但较低阻尼也给快速屈伸留下空间，这次提高越障专用阻尼作为可验证的调参假设。

原课程忽略 3 s 内失败，而末尾平均回合约 1.8–2.1 s，早期摔倒不降级会使环境停留在尚未掌握的难度。顶层成功后的随机回低层又使分布持续变化。曲线回落既与分布变化有关，也包含通过率和方向控制的真实退化，不能简单解释成“任务变难所以正常”。当前修改没有调整 PPO 网络或学习率。

`Episode_Reward/*` 是累计加权奖励除以最大回合长度 18 s，不能直接看作奖励权重或实际逐回合总分。原生 `Metrics/base_velocity/error_vel_xy` 也按最大命令周期 18 s 归一化，短回合下的 0.11 不是实际平均速度误差 0.11 m/s。新增 `episode_vel_xy_error` 按实际采样步数计算均值，以 m/s 记录。

新增 `Metrics/base_velocity/` 指标：`forward_speed`（m/s）、`vertical_speed_abs`（m/s）、`leg_speed_abs`（四个腿关节平均绝对 rad/s）、`dual_wheel_air_fraction`（双轮同时离地超过 16 ms 的时间占比）、`flat_fraction` 和 `flat_air_fraction`（平地且双轮腾空占整个回合时间的比例）。当 `flat_fraction > 0` 时，后两项之比可估计平地腾空占比；应结合垂直速度与视频判断，单轮越障不等于双轮跳跃。

`Curriculum/traversal_outcomes/<地形名>_top_attempts` 与 `_top_successes` 在课程升级前累计两种地形各自顶层回合计数，重启实验后归零。对同一窗口计算 `Δsuccesses / Δattempts`，可估计该窗口的顶层通过率（TensorBoard 每轮记录的计数经过轮内均值，会有一个 rollout 的边界误差）。不能把顶层环境占比当通过率。PLAY 也保留这项统计；PLAY 的单行地形全是 20 cm。

建议先建立独立新实验，以新奖励从头训练，再用相同种子、相同满难度地形比较两种地形的成功率、机身接触、越界和步态指标。新旧奖励量级不同，总奖励不可直接比较：

```bash
python scripts/rsl_rl/train.py --task Wheeled-Leg-Obstacle-Jump \
  --headless --logger tensorboard --num_envs 2048 --max_iterations 8000 \
  --run_name obstacle_rolling_v2 --seed 43
```

也可单独做迁移实验，在上述命令后添加 `--init_checkpoint logs/rsl_rl/wheeled_leg_obstacle_jump/2026-10-14_17-25-22/model_7999.pt`，加载策略和观测统计但重新建立优化器；不要把这次奖励/阻尼修改当作原实验直接续训。迁移仍可能保留旧的走跳习惯。

## 严格跳跃课程

平面上设置横向长方体障碍，尺寸为 X 方向厚度 0.10 m、Y 方向长度 3.0 m、Z 方向高度 0.20 m。障碍位于局部 x = 1.3 m，移动目标位于 x = 2.6 m，出发点为环境原点。蓝色和绿色标记分别显示出发点和目标。有效横向通道半宽 0.60 m，障碍覆盖整个通道。

课程改变障碍露出地面的高度：用同一个运动学长方体上下移动；原地和无障碍移动阶段将其隐藏到地下。

| stage | 学习内容 | 双轮轮底目标离地高度 / 障碍高度 |
| --- | --- | --- |
| 0 | 原地小跳 | 4 cm / 无障碍 |
| 1 | 原地跳跃 | 8 cm / 无障碍 |
| 2 | 原地高跳 | 12 cm / 无障碍 |
| 3 | 移动跳跃，跨过虚拟障碍位置后到达目标 | 26 cm / 无实体障碍 |
| 4–8 | 移动跳过长方体并到达目标 | 目标至少 26 cm；障碍依次为 4、8、12、16、20 cm |

每个环境独立统计成功率。当前阶段至少完成 20 个回合且成功率 EMA ≥ 0.70 后升一级，最高 stage 8。启动训练默认 stage 0；PLAY 默认 stage 8 并关闭晋级。原地参考周期为 2.8 s，起跳参考时刻为 1.2 s，之前 0.35 s 参考蹲伏 10 cm。移动时接近障碍 0.75 m 触发一次蹲伏/起跳参考；接近速度命令为 0.95 m/s，到达目标后减速。

成功判据来自实际接触与几何状态，而不是只满足机身高度参考：

1. 双轮先稳定着地至少 0.10 s，再同时腾空；有效腾空至少 0.08 s。
2. 原地阶段要求轮底离地高度达到目标、直立并在原点 0.25 m 内稳定落地。
3. 移动阶段要求两轮在同一次连续腾空中从障碍前方穿过障碍范围，到达障碍后方；经过障碍时轮底必须高于障碍顶面至少 1.5 cm，且保持在有效通道内。
4. 接触障碍、用腿/机身支撑、绕行、两次跳跃拼接、直接驱动到障碍后方均不能获得越障成功。
5. 双轮稳定落地至少 0.12 s、姿态直立后，移动到目标 0.25 m 范围内并减速到 0.35 m/s 以下，才结束成功回合。

奖励提供高度参考、腾空、起跳速度、有效落地和目标进度信号；恢复阶段约束持续垂直速度，原地阶段惩罚漂移。课程、目标与障碍状态追加 12 项观测，包括目标相对位置、障碍距离/高度/厚度、阶段、起跳时序、目标高度、实际机身高度和双轮腾空时间。

| 配置 | Actor 观测 | Critic 观测 | 动作 | 默认迭代 |
| --- | --- | --- | --- | --- |
| 越障 | 82 | 89 | 6 | 7000 |
| 严格跳跃 | 40 | 47 | 6 | 5000 |

两个 runner 使用 `[128, 128, 64]` 网络、观测归一化、固定学习率 2.5e-4 和 log-std 噪声。

## 训练与迁移

```bash
conda activate env_isaaclab
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg

python scripts/rsl_rl/train.py --task Wheeled-Leg-Obstacle-Jump \
  --headless --logger tensorboard --run_name obstacle_jump

python scripts/rsl_rl/train.py --task Wheeled-Leg-Jump \
  --headless --logger tensorboard --run_name jump
```

默认 4096 个环境，显存不足可加 `--num_envs 1024`。建议先从头训练，确认小跳课程能晋级；也可以在命令后添加以下参数，从已有 v1 / terrain-balance 初始化：

```bash
--init_checkpoint logs/rsl_rl/wheeled_leg_terrain_balance/best/model.pt
```

迁移保持原来的 28/35 项观测前缀和动作顺序，越障允许追加 54 列、跳跃允许追加 12 列。新增输入权重置零，迁移策略和归一化统计，使用新优化器并从迭代零开始；新的动作尺度、增益和地形会改变闭环行为，因此迁移不保证比从头训练更好。旧版仅原地跳跃的 32/39 维 checkpoint 不兼容新版 40/47 维策略。

恢复新版实验使用 `--resume --load_run '<run>' --checkpoint model_1000.pt`，不能与 `--init_checkpoint` 同时使用。RSL-RL checkpoint 不保存这个任务的每环境课程级别和成功率；恢复训练默认重新从 stage 0 开始。若已验证某个阶段，可显式添加 Hydra 覆盖，例如：

```bash
python scripts/rsl_rl/train.py --task Wheeled-Leg-Jump --headless \
  --resume --load_run '<run>' --checkpoint model_1000.pt \
  env.commands.base_velocity.initial_stage=3
```

## 回放、奖励和力矩分析

```bash
python scripts/rsl_rl/play.py --task Wheeled-Leg-Obstacle-Jump-Play --num_envs 1
python scripts/rsl_rl/play.py --task Wheeled-Leg-Jump-Play --num_envs 1

# 单独检查已学到的原地跳跃能力
python scripts/rsl_rl/play.py --task Wheeled-Leg-Jump-Play --num_envs 1 \
  env.commands.base_velocity.initial_stage=2

python tools/plot_mean_reward.py --task Wheeled-Leg-Obstacle-Jump --smoothing 20
python tools/plot_mean_reward.py --task Wheeled-Leg-Jump --list-tags
python tools/plot_mean_reward.py --task wheeled_leg_jump \
  --tag Episode_Reward/jump_landing --smoothing 20
```

奖励工具支持以上两个任务的训练/PLAY Gym ID，也继续支持实验目录名与 `--run-dir`。只选择含 TensorBoard 事件的 run，跳过 `best/`。课程统计和命令统计也可通过 `--tag` 绘制；先用 `--list-tags` 确认事件文件中的完整标签，例如 `Curriculum/jump_stage/mean_stage`、`Metrics/base_velocity/strict_success`。总奖励和高度参考奖励不能代替实际成功率。

两个 PLAY 配置默认 20 个环境、关闭观测噪声和材质随机化；当前越障 PLAY 额外保留动态速度和满强度随机推力。启用已有 `play.py` 力矩记录。环境 0 的文件保存在所加载 run 的 `play_joint_torque/<timestamp>/joint_torque.csv`：

```bash
python tools/plot_joint_torque.py \
  'logs/rsl_rl/wheeled_leg_jump/<run>/play_joint_torque/<timestamp>/joint_torque.csv' \
  --env-index 0 --rated-method rolling-rms --rolling-window-seconds 0.5
```

## 最佳策略管理

`update_best_policy.py` 根据两个默认实验名自动填入训练和 PLAY Gym ID；自定义实验名时仍可显式使用 `--task`、`--play-task`。它管理人工选定的 checkpoint，没有根据奖励自动选择策略。

```bash
python scripts/rsl_rl/update_best_policy.py \
  --experiment wheeled_leg_obstacle_jump --run '<run>' --checkpoint model_1000.pt \
  --comment '20 cm 地形通过与姿态恢复评估通过'

python scripts/rsl_rl/update_best_policy.py \
  --experiment wheeled_leg_jump --run '<run>' --checkpoint model_1000.pt \
  --comment 'stage 8 连续双轮腾空越障、落地和目标到达评估通过'

python scripts/rsl_rl/play.py --task Wheeled-Leg-Jump-Play \
  --load_run best --checkpoint model.pt --num_envs 1
```

分别建立 `best/model.pt` 相对符号链接和 `best/notes.md`，保留源 checkpoint。示例中的 run/checkpoint 必须替换为实际文件。

## 验证范围

CPU 测试覆盖原地飞行到稳定落地、连续双轮越障到达目标、单轮抬起拒绝、低高度/绕行/腿接触拒绝、两次跳跃拼接拒绝、直接驱动越过拒绝、部分重置、环境原点高度不变性、成功课程晋级与失败终止拒绝，以及 12/54 列追加观测的策略迁移输出一致性。

本轮越障调整后共 18 项 CPU 测试通过，新增覆盖命令速度封顶/倒退、平地代价的障碍放松与地形绝对高度不变性、缺失射线、首次重置、早期失败降级、连续两次成功晋级、顶层不回绕、按地形类型累计顶层成败。越障 TRAIN 用 4 环境运行 300 步；20 cm PLAY 用 16 环境运行 300 步并完成两轮小规模 PPO 更新。新增运动指标、奖励和观测均为有限值，严格跳跃的髋/膝阻尼仍为 4/5。两轮 PPO 只验证更新流程，没有衡量训练收敛或步态改善。

Isaac Lab GPU 冒烟检查验证了四个注册配置及 PPO 配置；使用 4 个环境，20 cm 越障 PLAY 完成 300 步，严格跳跃 TRAIN/PLAY 分别在 stage 0、3、8 各完成 300 步。观测与奖励均为有限值，实际观测维度为 82/89 和 40/47；stage 8 实体障碍顶面为 0.20 m。PLAY 的原点/目标标记也完成运行检查。测试使用固定动作，没有训练出策略。

合成日志检查验证两个任务的 Gym ID/PLAY 别名、奖励 PNG/CSV、力矩 PNG/统计 CSV，以及最佳策略相对链接和自动生成的任务命令，全部在 `/tmp/` 中完成，没有更新真实最佳策略。

这些检查验证实现条件，不代表已经训练出能完成 20 cm 越障的策略。正式训练应分别检查倒金字塔和随机方格的顶层通过率，以及跳跃 stage 0→8 的晋级、腾空、落地和目标到达情况。

上一版长时稳定性验证（历史记录）：25 项 CPU 测试通过，其中新增 7 项覆盖连续稳定停车、路点单次奖励、往返不重置、部分环境重置隔离、快慢阶跃/零指令、粗糙地形姿态代价、腾空时放松接地弹跳代价、推力的质量缩放/渐增/保护期/到期清零，以及到达过路点后再摔倒不能升级。16 环境 TRAIN 和满难度 20 cm PLAY 各运行 700 步，验证外力实际施加并清零、82/89 维观测/奖励/指标有限；TRAIN 另完成两轮 PPO 更新。日志位于 `/tmp/wheeled_stability_train_smoke.log` 与 `/tmp/wheeled_stability_play_smoke.log`。这些固定动作短时检查验证集成，不代表已有策略能连续稳定 30 s；稳定性、急停距离和抗推成功率仍需正式训练后评估。

本次主动上台阶版本验证：31 项 CPU 测试通过，新增 6 项覆盖局部台阶识别的航向/地形原点不变性、无效射线/下台阶、停车和转向不放宽约束、仅放宽攀爬 pitch/腿目标变化、停滞门控、受支撑抬升的接触与单次计数/渐进升高/部分重置，以及鲁棒性延迟和单程通过后的存活升级。四个任务注册配置和 PPO 配置均通过加载校验；越障 TRAIN 与满难度 20 cm PLAY 各用 16 环境运行 700 步，推力施加/到期清零、82/89 维观测、奖励和新增状态均通过检查，TRAIN 另完成 2 次 PPO 更新。严格跳跃增益加载检查仍为髋 140/4、膝 220/5。日志为 `/tmp/wheeled_stair_train_smoke.log` 和 `/tmp/wheeled_stair_play_smoke.log`。这些检查没有训练出爬阶策略，无法替代正式训练后的通过率、步态及长时稳定性评估。
