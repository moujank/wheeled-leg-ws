# wheeled_arm_v2 平地运动基线：v2_wheeled_arm_v1

任务目标是在机械臂六个关节全部参与策略控制时，实现平地站稳、前后行驶和转向，同时使机械臂尽量维持收缩姿态并减小抖动。训练和 Play 配置均完整定义，直接继承 Isaac Lab 的 `ManagerBasedRLEnvCfg`，没有导入或继承任何现有任务环境。

## 参考依据与资产修正

参考当前 `v2_wheeled_leg_v1.py` 的结构、奖励与双腿协调项，以及 `logs/rsl_rl/wheeled_leg_v1/2026-10-17_10-52-09_v1_stable_baseline/params/` 中的平地基线参数：2048 个环境、固定学习率 `3e-4`、较低探索噪声和动作变化惩罚。当前 v1 代码已将前后速度扩大到 ±1.1 m/s，本任务先采用 ±0.5 m/s 建立带臂运动基线。

`asset/wheeled_arm_v2.py` 的配置按本地实际 USD 校对后修改：

| 项目 | 训练配置与依据 |
| --- | --- |
| 初始基座高度 | 0.58 m，与 v1 实际环境一致；原资产为 0.7 m |
| 腿部初始姿态 | HipL/R = ±0.65，KneeL/R = ±1.30 rad |
| 实际自由度 | 4 个髋膝关节、2 个轮关节、6 个机械臂关节，共 12 个 |
| `Claw` | 实际 USD 没有该关节，移除无匹配的初始值 |
| 髋/膝力矩上限 | 120 N·m，按该 USD 的驱动力上限配置 |
| 轮力矩上限 | 17 N·m，按该 USD 配置；不使用原配置统一的 150 N·m |
| 机械臂力矩上限 | 27 N·m，按该 USD 配置 |
| 髋/膝/轮速度上限 | `velocity_limit_sim = 2π` rad/s；避免旧 `velocity_limit` 在隐式执行器中被忽略 |
| 机械臂速度上限 | `velocity_limit_sim = 2.0` rad/s，限制快速甩动 |
| PD 增益 | 髋 20/1.5、膝 30/1.5、轮 0/1.5、机械臂 40/6（刚度/阻尼） |

上述力矩值是 USD 的驱动力上限，不代表已核实的真实电机额定连续力矩。腿与轮的力矩约束和旧 v1 训练中统一 150 N·m 的设置有差异，因此新任务使用独立实验，从头学习。

机械臂默认收缩姿态为：

```python
Arm_R  =  0.0
Arm_Y1 = -0.65
Arm_Y2 =  1.20
Arm_P1 =  0.0
Arm_Y3 =  0.0
Arm_P2 =  0.0
```

这些值定义在 `ARM_FOLDED_JOINT_POS` 中，均在实际 USD 的软限位内。按同一机器人 URDF 做正向运动学计算，腕部 `Arm_P2` 关节原点相对基座的位置从零位时的 `(0.3365, 0.0014, 0.5948)` m 变为约 `(0.1251, 0.0014, 0.3548)` m，到基座原点的距离从 0.6834 m 减为 0.3762 m。机械臂质心的前向偏移也从约 0.158 m 减到 0.0035 m。这是几何检查，尚未经过 GPU 动力学与碰撞验证。

当前自碰撞开关与 v1 平地基线一致，为 `False`。机械臂与地面的碰撞有效，机械臂触地会终止回合。这个基线没有用机器人内部自碰撞约束防止关节组合穿过机身；需要依赖自碰撞进行实机姿态验证时，应先验证碰撞几何及折叠姿态，再单独启用和评估。

## 环境、动作和奖励

- 训练任务：`Wheeled-Arm-v1` → `WheeledArmV1EnvCfg`。
- 评估任务：`Wheeled-Arm-Play-v1` → `WheeledArmV1EnvCfg_PLAY`。
- Runner：`WheeledArmV1PPORunnerCfg`，独立实验目录 `logs/rsl_rl/wheeled_arm_v1/`。
- 物理频率 500 Hz、策略频率 125 Hz；训练回合 20 s，Play 回合 40 s。
- 平地、前后速度 ±0.5 m/s、偏航速度 ±0.5 rad/s，训练中 20% 为静止指令；初期没有随机推力。

动作按三个组组成：

| 动作组 | 维数 | 控制方式 |
| --- | --- | --- |
| `joint_pos` | 4 | 髋膝默认姿态加 `0.25 × action`，沿用 v1 的最终目标限幅 |
| `joint_vel` | 2 | 轮速度目标 `4 × action`，最终限幅 ±6 rad/s |
| `joint_pos_arm` | 6 | 收缩姿态加 `0.15 × action`，位置 PD 控制 |

机械臂不是锁定关节，策略可控制全部六个自由度。目标位置限制为 `Arm_R` ±2.8 rad，其余五个机械臂关节 ±1.4 rad，均处于 USD 软限位内；限幅作用于“默认姿态加动作偏移后的最终目标”。动作组中的六个机械臂关节顺序显式固定。

策略观测为 58 维：基座线速度、角速度、重力投影、速度命令、10 个非轮关节的相对位置、全部 12 个关节的相对速度与力矩、12 维上一步动作。critic 额外获得 13 个刚体的接触状态，共 71 维。连续转动的轮角不进入位置观测。

保留 v1 的速度跟踪、存活、姿态、能耗、双腿协调、轮滑与单侧支撑奖励。额外加入以下机械臂惩罚，均有独立 TensorBoard 标签：

| 项目 | 函数/对象 | 权重 |
| --- | --- | --- |
| `arm_folded_pose` | 六个机械臂关节相对默认收缩姿态的 L1 偏差 | -0.8 |
| `arm_joint_vel` | 机械臂关节速度平方和 | -0.04 |
| `arm_joint_acc` | 机械臂关节加速度平方和 | -1e-6 |
| `arm_action_rate` | 仅 `joint_pos_arm` 动作组的相邻动作差平方和 | -0.05 |

全动作变化惩罚仍为 -0.03，因此机械臂动作还会受到单独的平滑约束。原来的双腿镜像奖励继续只作用于双腿/轮；PPO 的镜像数据增强和镜像损失关闭，因为现有映射没有覆盖单侧串联机械臂。

训练时腿与轮使用缩放重置，机械臂采用收缩姿态附近 ±0.03 rad 的偏移重置，使零参考角关节也能获得小幅扰动。Play 关闭观测噪声、质量/摩擦随机化，使用名义姿态和零初始速度。

## 启动训练

在可以正常运行 Isaac Sim 的 GPU 环境执行：

```bash
conda activate env_isaaclab
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg

python scripts/rsl_rl/train.py \
  --task Wheeled-Arm-v1 \
  --num_envs 2048 \
  --max_iterations 5000 \
  --run_name flat_arm_folded \
  --logger tensorboard \
  --headless
```

新任务动作维数、观测维数和机械臂动力学均有变化，应从头训练。不要直接用原来 6 维动作的 v1 checkpoint 做 `--resume`。初次建议先将环境数和迭代数分别改为 16 和 2，确认资产加载、接触、姿态和动作正常，再开始完整训练。

PPO 使用 `[128, 128, 64]` 的 actor/critic、观测归一化、log 标准差、初始噪声 0.35、固定学习率 `3e-4`、每轮 32 步、3 个学习 epoch 和 4 个 minibatch，每 50 轮保存一次。

## 三个工具的使用

下面的 `RUN` 和 `CHECKPOINT` 需要替换为实际存在的训练记录。优先结合静止平衡、前后跟踪、转向、机械臂姿态和力矩来选择模型。

```bash
RUN=2026-10-20_12-00-00_flat_arm_folded
CHECKPOINT=model_4950.pt

python scripts/rsl_rl/update_best_policy.py \
  --experiment wheeled_arm_v1 \
  --run "$RUN" \
  --checkpoint "$CHECKPOINT" \
  --comment "平地运动稳定，机械臂保持收缩且抖动较小"
```

该工具已增加新任务的默认映射，会生成 `best/model.pt` 软链接和含正确训练/Play 命令的 `best/notes.md`。

绘制最新一次训练的奖励：

```bash
python tools/plot_mean_reward.py --task Wheeled-Arm-v1 --smoothing 20
```

精确查看指定运行和机械臂指标：

```bash
python tools/plot_mean_reward.py \
  --run-dir "logs/rsl_rl/wheeled_arm_v1/$RUN" \
  --tag Episode_Reward/arm_folded_pose \
  --output-name arm_folded_pose.png \
  --smoothing 20
```

评估最佳模型并记录 12 个关节力矩：

```bash
python scripts/rsl_rl/play.py \
  --task Wheeled-Arm-Play-v1 \
  --num_envs 1 \
  --load_run best \
  --checkpoint model.pt
```

`play.py` 按实际 `robot.joint_names` 写入所有关节列，不需要为机械臂硬编码列名。记录位于源训练运行目录下的 `play_joint_torque/<timestamp>/joint_torque.csv`；通过 `best` 软链接加载时也会解析回源运行目录。以终端打印的实际 CSV 路径为准，替换下面的 `<timestamp>`：

```bash
python tools/plot_joint_torque.py \
  "logs/rsl_rl/wheeled_arm_v1/$RUN/play_joint_torque/<timestamp>/joint_torque.csv"
```

只查看机械臂：

```bash
python tools/plot_joint_torque.py \
  "logs/rsl_rl/wheeled_arm_v1/$RUN/play_joint_torque/<timestamp>/joint_torque.csv" \
  --joints Arm_R Arm_Y1 Arm_Y2 Arm_P1 Arm_Y3 Arm_P2 \
  --output /tmp/wheeled_arm_v1_arm_torque.png \
  --summary-output /tmp/wheeled_arm_v1_arm_torque_summary.csv
```

## 已完成的验证与待验证项

CPU 测试检查实际 URDF 关节覆盖、动作/执行器/重置互不遗漏、目标与默认角位于软限位内、Train 与 Play 配置独立、任务注册、机械臂动作平滑惩罚的切片范围，以及原生 RSL-RL 对 58/71 维观测和 12 维动作的一次 PPO 更新。另使用 USD API 核对了实际 USD 关节限位与驱动力上限，用临时 TensorBoard 日志和 12 关节 CSV 跑通三个工具。

```bash
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest tests -q
```

本次验证环境的 NVIDIA 驱动不可用，尚未执行 Isaac Sim 动力学回合或长期训练。默认姿态、PD 增益、探索噪声和奖励权重是训练起点；需要在实际仿真中观察碰撞、关节饱和、机械臂抖动与指令跟踪后再调整。调整顺序建议为先确认站稳和初始接触，再看速度跟踪，然后评估收缩程度与力矩峰值；不要只依据总奖励判断是否收敛。
