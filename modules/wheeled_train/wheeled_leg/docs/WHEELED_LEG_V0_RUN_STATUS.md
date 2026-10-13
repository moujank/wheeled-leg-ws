# Wheeled-Leg-v0 运行验证与暂缓问题

验证日期：2026-10-12。目标：先完成一次环境初始化、采样、PPO 更新和 checkpoint 保存。

## 配置基准

- 训练任务：`Wheeled-Leg-v0`，环境类：`v2_wheeled_leg_v0.WheeledLegEnvCfg`。
- 回放任务：`Wheeled-Leg-Play-v0`，环境类：`v2_wheeled_leg_v0.WheeledLegEnvCfg_PLAY`。
- 两个任务均绑定 `WheeledLegPPORunnerCfg`；该类继承 `PPORunnerCfg`，仅覆盖实验名为 `wheeled_leg_v0`。
- PPO 参数沿用基础配置：actor/critic 为 `[32, 32]`，关闭经验归一化，初始学习率 `1e-3`、自适应调度、每环境每次采样 16 步、默认 150 次迭代。
- PPO 配置不定义物理步长。环境时序按基础底盘文件统一为 `sim.dt=0.002`、`decimation=4`，即物理 500 Hz、动作更新 125 Hz。
- 当前场景已经引用 `asset/wheeled_leg_v2.py`，使用 v2 底盘 USD；USD 路径基于资产配置文件解析，不依赖启动目录。

## 已处理的运行阻塞

1. 训练注册原先指向不存在的 `wheeled_leg_env_cfg.py`，已指向目标 v2 配置文件。
2. 增加 Play 注册，避免回放时误用训练配置。
3. 任务中未使用的 `wheeled_arm_v2` 导入触发该资产文件的语法错误，已移除无用导入；机械臂资产文件本身尚未修复。
4. v2 USD 的四个髋、膝关节限位在运行时读为 `[0, 0]`，原默认位置 `0.52 rad` 不合法。暂时改为合法零位，以完成运行验证，未修复 USD 限位。
5. v2 接触传感器刚体名为 `HipL/HipR/KneeL/KneeR`，原小写正则无法匹配。奖励和终止配置均改为 `Hip.*`、`Knee.*`，保留接触惩罚与终止机制。

## 最小训练结果

从训练工程目录运行：

```bash
cd /home/moujank/project/wheeled-leg-ws/modules/wheeled_train/wheeled_leg
/home/moujank/miniconda3/envs/env_isaaclab/bin/python scripts/rsl_rl/train.py \
  --task Wheeled-Leg-v0 \
  --num_envs 4 \
  --max_iterations 1 \
  --headless
```

沙箱内 GPU 不可见且 Isaac Sim 缓存目录不可写；获得运行授权后，在沙箱外识别到 RTX 5080，并完成验证。不要据沙箱内结果判断主机没有 GPU。

- 程序退出码为 0，完成 1 次 PPO 更新，共采样 64 个环境步。
- 实际 policy 观测为 27 维，critic 观测为 10 维，动作为 6 维。
- 产物：[运行目录](../logs/rsl_rl/wheeled_leg_v0/2026-10-12_20-51-24/)、[checkpoint](../logs/rsl_rl/wheeled_leg_v0/2026-10-12_20-51-24/model_0.pt)、[PPO 配置](../logs/rsl_rl/wheeled_leg_v0/2026-10-12_20-51-24/params/agent.yaml)。
- checkpoint 模型张量均为有限值；TensorBoard 的 value loss 约 `2.2290`、surrogate loss 约 `-0.0231`。
- 平均 reward 约 `-1.7239`，平均 episode 长度为 1 步，接触终止占比为 1.0。

这次验证确认运行链路和 PPO 更新可执行，没有证明策略具备运动能力，也没有验证 Play 回放。接触终止和资产关节限位问题需在长时间训练前处理。

## 暂缓问题

| 问题 | 当前影响 | 后续处理 |
| --- | --- | --- |
| v2 髋、膝关节限位为 `[0, 0]` | 零位可以初始化，但关节运动受限；本次出现每步接触终止 | 修复并核对 USD 限位、关节轴、初始高度和接触几何，再恢复屈腿姿态。源 URDF 还存在 `lower=3.14`、`upper=-3.14` 的倒置限位及 `velocity=0`，应一并核对 |
| heading 开启但 `ang_vel_z=(0, 0)` | 朝向控制输出被截断为零；Play 的 heading=0 不会自动纠正朝向 | 确认需要朝向跟踪还是仅直线速度跟踪，再调整指令 |
| 零默认姿态使用 `reset_joints_by_scale` | 关节位置随机化退化为零位 | 资产限位修复后，确认采用默认姿态缩放还是位置偏置 |
| critic 只有线速度和接触状态 | 缺少指令、姿态及关节状态，可能影响价值估计 | 后续重新设计 critic 观测 |
| Play 保留材质、质量、根部重置随机化及 2% 站立采样 | 不是确定性的持续固定速度评估 | 根据评估目标单独设置 |
| 未使用的 `asset/wheeled_arm_v2.py` 有语法错误 | 当前底盘任务不再导入它，因此不阻止本次运行；机械臂任务仍可能受影响 | 实际启用机械臂任务前修复 |

[训练工程说明](../README.zh.md)
