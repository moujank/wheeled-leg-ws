# MuJoCo Consistency Checklist

[文档索引](README.md) · [调试交接](MUJOCO_DEBUG_HANDOFF.md) · [资产查询](ASSET_INDEX.md)

范围：2026-07-08 训练 run 的一致性排查记录。以下“当前”对应该次记录，其他 checkpoint 应重新核对其 `params/env.yaml`。

本文档只记录 MuJoCo 侧的一致性排查清单、当前结论和后续优先级，不包含 IsaacLab 训练代码修改建议。目标是先判断 `1.0 m/s` 失败究竟是 MuJoCo 接入问题，还是训练资产本身与 MuJoCo 部署资产已经发生结构性分叉。

## 1. 当前目标

目标按优先级排序如下：

1. 保持当前启动基线不退化。
2. 继续做 MuJoCo 侧一致性排查，不再扩大 handoff 控制实验面。
3. 优先确认资产和控制语义是否与本次训练产物一致。
4. 如果 MuJoCo 侧再怎么对齐也无法稳定到 `1.0 m/s`，再输出训练资产配置调整报告，但不改 IsaacLab 训练代码。

## 2. 当前结论

当前已经确认的事实：

1. `1.0 m/s` 对本次训练产物不是超出命令分布。
2. 当前真正的问题，已经不再是“启动立刻倒下”，而是策略接入后高速前进会进入明显的前倾失稳。
3. MuJoCo 侧已经有一些回归性修正，但这些修正还不足以解释 `0.23-0.24 m/s` 以上就易倒、以及更高目标速度无法稳定的问题。
4. 当前最可疑的是“资产/执行语义不一致”，而不是单纯调反馈增益。

## 3. 已确认的训练产物基线

本次训练保存下来的参数来自：

- [env.yaml](../../wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/2026-07-08_17-08-25/params/env.yaml)

已确认的关键项：

- `init_state.pos.z = 0.34`
- `init_state.joint_pos = 0`
- `hip stiffness/damping = 20 / 1.5`
- `knee stiffness/damping = 30 / 1.5`
- `hub stiffness/damping = 0 / 1.5`
- `velocity_limit = 6.283185307179586 rad/s`
- `effort_limit = 150`
- `commands.base_velocity.ranges.lin_vel_x = [-1.2, 1.2]`

这意味着：

- MuJoCo 若要对齐本次训练产物，应以 `0.34 + 零关节位姿 + 20/30/1.5 + 6.283` 为第一参考。
- 不能再默认参考当前源码里的另一套资产默认值。

## 4. 已发现的分叉

### 4.1 训练源码资产与训练产物参数分叉

文件：

- [wheeled.py](../../wheeled_train/wheeled_leg/source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/asset/wheeled.py)

当前源码里仍能看到另一套默认值：

- `init_state.pos = (0.0, 0.0, 0.38)`
- `HipL/HipR/KneeL/KneeR = -0.35, 0.35, 0.75, -0.75`
- `hip stiffness/damping = 30 / 2.5`
- `knee stiffness/damping = 45 / 2.5`
- `hub damping = 2.0`

但本次训练保存下来的 `env.yaml` 已经不是这一套。

结论：

- 本次 MuJoCo 对齐，必须优先参考训练产物 `env.yaml`，不能根据当前 `wheeled.py` 盲对。
- 如果后续要重训，必须先收敛“源码资产默认值”和“实际训练配置”之间的分叉。

### 4.2 MuJoCo 当前活动 MJCF 与原始导出 XML 分叉

文件：

- 活动 MJCF: [wheeled_leg.xml](../asset/mjcf/wheeled_leg.xml)
- 原始导出: [meshes/wheeled_leg.xml](../asset/wheeled_leg/meshes/wheeled_leg.xml)
- 原始 URDF: [wheeled_leg.urdf](../asset/wheeled_leg/urdf/wheeled_leg.urdf)

当前活动 MJCF 相比导出模型，已经有人为改动：

- `home qpos z` 已调整到 `0.34`
- base/leg 接触摩擦已从 `1.1` 降到 `0.8`
- wheel 接触摩擦已改到 `0.9`
- wheel actuator `ctrlrange` 已从 `±115` 改到 `±6.28319`
- wheel actuator `forcerange` 已从 `±17` 改到 `±150`
- wheel joint damping 当前是 `1.5`

结论：

- 活动 MJCF 已不是原始 CAD/URDF 导出值，而是“部署对齐版本”。
- 后续任何修改都必须说明究竟是在向“训练产物”对齐，还是在向“原始导出资产”回退。

## 5. 高优先级一致性核对项

以下项目优先级最高，且都只涉及 MuJoCo 侧。

### P0. 初始状态语义

检查项：

- `home` keyframe 是否固定为 `z=0.34`
- `_reset_to_home()` 是否使用 `z=0.34`
- `stand_base_height_target` 与 `tether_height_target` 是否都为 `0.34`
- `DEFAULT_STAND_JOINT_POS` 是否与本次训练产物一致

当前状态：

- 活动 runner 已对齐到 `0.34`
- source 层适配器之前没有暴露高度目标参数，容易造成入口默认值漂移

### P1. 执行器语义

检查项：

- hip/knee `kp` 是否分别为 `20/30`
- hip/knee `damping` 是否为 `1.5`
- hub velocity actuator `kv` 是否为 `1.5`
- hub `ctrlrange` 是否对齐训练侧 `velocity_limit = 6.283`
- `wheel_action_scale` 是否与当前部署 runner 保持一致

风险说明：

- MuJoCo velocity actuator 是显式速度伺服，Isaac 训练时是 implicit actuator 语义。
- 即使数值上看起来一致，执行效果也可能不同，这是当前最值得怀疑的根因之一。

### P2. 轮轴方向与左右轮符号

检查项：

- `HubL axis = 0 0 -1`
- `HubR axis = 0 0 1`
- 观测和动作映射是否默认接受这种左右轮反号结构
- 相同前进命令下，左右轮目标是否在期望方向产生正向推进

当前结论：

- 原始 URDF、原始导出 XML、活动 MJCF 三者在轴向上是一致的。
- 但“结构上一致”不等于“与训练时 USD/关节正方向一致”，所以这仍然是第一优先级嫌疑项。

### P3. 接触与摩擦

检查项：

- 平面 `condim` 是否合理
- 轮子碰撞 mesh 是否过于尖锐，导致 patch 行为与 Isaac 差异过大
- 基座和腿部碰撞 mesh 是否参与了不必要接触
- base/leg/wheel 摩擦是否接近训练时随机化可覆盖范围

当前结论：

- 训练环境默认 `1.0/1.0`，但随机化范围更接近 `0.6-1.0` 和 `0.4-0.8`
- 活动 MJCF 降低摩擦到 `0.8/0.9` 是合理方向
- 但如果轮子仍使用 mesh 直接接地，接触补丁差异仍可能主导高速失稳

### P4. 质心和惯量

检查项：

- 活动 MJCF 是否仍保留原始 URDF 的惯量主轴变换结果
- `base_link` COM 是否与训练资产一致
- wheel inertial 姿态是否与 MuJoCo 轮轴正方向组合出意外陀螺效应

当前结论：

- 活动 MJCF 当前的 inertial 数值与原始导出资产一致，没有发现后改痕迹
- 但尚未证明它与训练所用 USD 里的刚体惯量完全一致

## 6. 当前建议的 MuJoCo 排查顺序

只做 MuJoCo 侧，建议按下面顺序执行：

1. 保持当前启动基线不变，先固定 `0.34` 高度链路不再改。
2. 固定当前 handoff 逻辑，不再继续叠新控制补丁。
3. 用最小实验验证左右轮轴方向和前进指令的符号关系。
4. 核对 wheel velocity actuator 语义是否需要更接近 Isaac 的速度限幅，而不是继续提反馈。
5. 若高速问题仍主要表现为大前倾后轮速饱和，优先怀疑轮地接触几何和隐式执行器差异。

## 7. 当前建议保留的 MuJoCo 改动

这些改动目前属于“合理的训练产物对齐”，不建议轻易回退：

- 活动 MJCF `home z = 0.34`
- runner 的 reset 高度链路对齐到 `0.34`
- wheel actuator `ctrlrange = ±6.28319`
- `kp/kv/damping` 与本次训练产物一致
- 较低的地面与轮子摩擦设置

## 8. 何时判定 MuJoCo 侧已到极限

满足以下特征时，可以认定问题主要不在 MuJoCo 配置细调：

1. 启动稳定已恢复。
2. `0.0-0.2 m/s` 可重复稳定。
3. 一旦进入更高速度区，失稳模式始终集中在同一类前倾和轮速饱和。
4. 轮轴、接触、执行器限幅、初始状态都已核对过。
5. 再继续改 MuJoCo 只是在不同保守限幅之间来回切换，而不是恢复真实速度跟踪。

若满足以上条件，下一步应输出“训练资产配置调整报告”，而不是继续堆 MuJoCo 运行期补丁。
