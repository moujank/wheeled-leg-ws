# 非 v0 任务的左右协调训练

本次改动覆盖当前仍有有效配置和注册的五个训练任务：`Wheeled-Leg-v1`、`Wheeled-Leg-Terrain-Balance`、`Wheeled-Leg-Terrains`、`Wheeled-Leg-Obstacle-Jump` 和 `Wheeled-Leg-Jump`，以及它们的 `Play` 配置。`Wheeled-Leg-v0` 的环境、奖励、输入输出维度和 PPO 基类保持不变。

`mdp/symmetry.py` 根据运行时解析出的关节、动作项、接触传感器和扫描射线顺序建立镜像置换，不依赖资产内部的默认排列。该资产的 USD 中左右轮轴方向相反，所以轮关节和轮动作镜像时交换左右并取负；基座线速度、角速度、重力、速度指令、腿关节和左右接触标签使用各自的物理反射规则。地形扫描根据射线的实际 `(x,y)` 坐标反射，支持 `xy` 和 `yx` 两种网格排序。

每个非 v0 任务都加入下列可观测奖励项：

- `mirrored_legs`、`mirrored_leg_velocity`：左右髋/膝位置和速度的镜像误差；
- `mirrored_leg_action`、`mirrored_leg_action_rate`：目标动作及其变化率的镜像误差；
- `straight_wheel_balance`：直行时两轮物理滚动速度差；
- `wheel_slip`：逐轮接触掩码下的轮胎接触点水平滑动；
- `single_side_support`：两轮承重不均和持续单轮承重；
- `straight_yaw_drift`：直行时实际偏航速率与指令偏航速率的误差。

奖励门控按任务区分：v1 使用最严格的直行约束；Terrain-Balance/Terrains 根据两轮高度差逐步放松腿部镜像；Obstacle-Jump 在局部上升台阶时降低腿部和承重约束；Jump 在腾空阶段降低约束，起跳和落地阶段仍保持同步。转弯不会被直行轮速和偏航约束惩罚，落地的第一只轮有短暂承重宽限期。

五个非 v0 PPO runner 默认启用 RSL-RL 原生 symmetry augmentation 和小系数 mirror loss。启用 empirical normalization 时，训练脚本在加载 checkpoint 后把归一化统计量与镜像置换合并，并重参数化首层，使旧策略的物理输出保持一致；统计量随后使用原样本及其镜像共同更新。没有新增观测项，也没有改变 v1 的关节驱动、动作尺度或动作维度，因此 v1 checkpoint 可以直接 warm-start 到 Terrains。

从 v1 迁移到 Terrains 的首个 rollout 会立刻使用 v1 的 actor：共同的 28 维本体感觉输入和归一化统计保持原策略，新增 54 维高度扫描的 actor 权重为零，以保持初始动作连续。高度扫描策略需要 PPO 在 Terrain 环境中逐步学出，不能仅靠不含地形观测的 v1 checkpoint 预先获得。critic 的接触观测位于 Terrains 高度扫描之后，迁移时按观测项名称映射，避免把接触权重错误接到高度射线上。

先训练 v1：

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
/home/moujank/miniconda3/envs/env_isaaclab/bin/python scripts/rsl_rl/train.py \
  --task Wheeled-Leg-v1 --headless --num_envs 4096 --max_iterations 3000 \
  --run_name v1_bilateral
```

选择 v1 训练目录中表现较好的 `model_*.pt` 后，启动多地形递进训练。`--init_checkpoint` 会创建新的 Terrains 实验并只迁移兼容的 actor/critic 前缀、normalizer 和策略权重；不要同时使用 `--resume`：

```bash
/home/moujank/miniconda3/envs/env_isaaclab/bin/python scripts/rsl_rl/train.py \
  --task Wheeled-Leg-Terrains --headless --num_envs 4096 --max_iterations 7000 \
  --init_checkpoint logs/rsl_rl/wheeled_leg_v1/<run>/model_<N>.pt \
  --run_name terrains_from_v1
```

若需要 Terrain-Balance 或障碍/跳跃课程，替换 `--task` 即可。训练中建议同时查看 `Episode_Reward/mirrored_legs`、`mirrored_leg_velocity`、`wheel_slip`、`single_side_support`、`straight_yaw_drift`，以及左右髋膝误差、两轮法向力和实际偏航速率；总回报下降本身不能说明左右协调变差。

已在无仿真 CPU 环境测试 46 项，包括五种观测维度、乱序关节/传感器、两种扫描网格排序、归一化 checkpoint 迁移、奖励门控和一个完整 PPO update。另用 2 个环境运行了 1 次 `Wheeled-Leg-v1` Isaac Sim 迭代，成功完成 runner 创建、镜像映射、rollout、PPO update 和 checkpoint 写入；完整训练效果仍需在目标 GPU 上通过回放和课程成功率评估。

仓库中 `late/` 以外的旧 Base-Locked、Arm-Free、Grasp 注册仍引用已删除的顶层配置，当前不把这些失效注册称为已覆盖任务；修复它们的注册和资产路径后，可按同一镜像函数补充对应专用关节映射。
