# Wheeled-Leg Sim2Sim Dynamic Flow

本文档按“忽略 `modules/wheeled_mujoco` 中现有接口程序，由你自己重新完善 MuJoCo 侧 `sim2sim`”这一前提重写。它只维护流程，不修改任何训练代码、MuJoCo 代码或资产文件。

更新时间：`2026-07-09`

[文档索引](README.md) · [项目待办](TODO.md) · [MuJoCo 架构](../modules/wheeled_mujoco/docs/SIM2SIM_ARCHITECTURE.md)

本文保留 2026-07-09 的迁移方案背景；下文 `27D`、动作尺度和源码事实属于当时基线。现有 runner 也支持 `30 × history`，迁移时须以选定导出策略与该 run 的 `params/env.yaml` 确定真实语义。文件路径及命令均相对于工作区根目录。

## 1. 目标

你现在要做的不是“调用现有 MuJoCo 入口”，而是自己重新搭一条完整的 `Isaac Lab -> MuJoCo` `sim2sim` 迁移链路。

这条链路的目标分成三层：

1. 训练侧目标：拿到一个明确、可复现、可导出的 Isaac Lab 策略。
2. 迁移侧目标：在 MuJoCo 中从零重建和训练策略兼容的 observation、action、reset、control loop。
3. 验证侧目标：先跑通最小闭环，再做一致性对齐，再做性能逼近。

这里最重要的原则只有一句：

`sim2sim` 不是“把 policy.pt 扔进 MuJoCo 就结束”，而是“在另一个 simulator 中把训练时的输入输出语义完整复刻出来”。

## 2. 你现在真正要做的完整流程

如果完全忽略 `wheeled_mujoco` 现有接口程序，整条流程应该按下面 10 个阶段执行。

### 阶段 0：锁定唯一训练对象

先不要写 MuJoCo 代码，先锁定训练对象。

你必须明确四件事：

1. 这次迁移的是哪个 task。
2. 用的是哪个训练 run。
3. 用的是哪个 checkpoint。
4. 最终部署读取哪个导出策略文件。

如果这四件事不唯一，后面所有 MuJoCo 对齐工作都会漂。

当前工作空间里和双轮足直接相关的训练目录是：

- `modules/wheeled_train/wheeled_leg`
- `modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked`

你后续不管重建什么 MuJoCo 入口，第一步都应该先绑定到某一个具体训练产物，而不是绑定到“当前源码默认值”。

### 阶段 1：冻结 Isaac 训练基线

这一步的任务只有一个：

把“训练时策略到底看到了什么、输出了什么”彻底弄清楚。

你要从训练任务配置和目标 run 的 `params/env.yaml` 中整理出以下信息：

1. 仿真步长
2. decimation
3. episode/reset 语义
4. observation 顺序和维度
5. action 顺序和维度
6. 每个 action 对应的控制含义
7. 关节默认零位
8. actuator 参数
9. 命令量定义
10. 训练时的随机化项

对你这套双轮足 base-locked 任务，当前已经能从训练配置确认的关键事实有：

1. policy observation 是 `27D`
2. action 是 `6D`
3. 腿部动作是 `Hip/Knee` 的位置控制分支
4. 轮子动作是 `Hub` 的速度控制分支
5. 腿 action scale 是 `0.5`
6. 轮 action scale 是 `10.0`
7. 训练 `sim.dt = 1/120`
8. `decimation = 4`

这一步的最终产物应该是一张“训练语义基线表”。

如果这张表没有整理出来，MuJoCo 侧就不能开工。

### 阶段 2：验证 Isaac 侧策略本身是好的

不要把 MuJoCo 中的失败先归咎于迁移实现。先证明训练侧策略本身是好的。

你要做的事情：

1. 用 Isaac Play 跑目标 checkpoint。
2. 确认策略能稳定站立。
3. 确认策略能在目标命令下前进。
4. 确认策略没有明显的抖动、偏航、持续前倾失稳。
5. 确认导出出来的 `policy.pt` 或 `policy.onnx` 是这次要迁移的那个 checkpoint 产物。

这一步的结论要非常简单：

- 如果 Isaac Play 里都不好，先别做 MuJoCo。
- 如果 Isaac Play 里好，再进入 MuJoCo 重建。

### 阶段 3：决定你在 MuJoCo 侧要“复刻什么”

这一步是整个 `sim2sim` 里最容易偷懒但最不能跳过的一步。

你必须先决定 MuJoCo 里要复刻的是哪一层语义：

1. 只复刻 policy inference 闭环
2. 复刻训练时的 control semantics
3. 复刻训练时的 reset semantics
4. 复刻训练时的 command semantics
5. 复刻训练时的 observation corruption / noise
6. 复刻训练时的 domain randomization 范围

推荐顺序不是一次全做，而是分三层：

1. 第一层：最小闭环
   - 能加载策略
   - 能正确组织 observation
   - 能把 action 写进 MuJoCo
2. 第二层：训练语义对齐
   - reset 对齐
   - action 对齐
   - observation 对齐
   - actuator 对齐
3. 第三层：鲁棒性增强
   - randomization
   - noise
   - startup / handoff 保护

也就是说，你现在不是先写“完整版 sim2sim”，而是先写“可验证的最小版本”，再逐步补齐。

### 阶段 4：重建 MuJoCo 侧机器人资产语义

既然你要忽略现有接口程序，那 MuJoCo 侧第一件真正要做的工程工作不是写 runner，而是确认机器人资产。

你要明确以下事项：

1. MuJoCo 使用哪个 MJCF 或 XML
2. 关节名字是否和训练侧一致
3. 关节顺序是否和训练侧一致
4. 左右腿的正方向是否一致
5. 左右轮的正方向是否一致
6. 默认站立姿态是否与训练零位一致
7. 轮地接触几何是否合理
8. 碰撞体、惯量、摩擦、阻尼是否可用于部署

这一步非常关键，因为后续所有 observation/action 对齐都建立在“资产命名和轴方向没错”这个前提上。

如果这里错了，后面 observation 和 action 即使数值都对，也会表现成：

1. 左右轮同命令却一前一后
2. 腿部位置偏移方向相反
3. 前进命令变成原地抖动或倒地

### 阶段 5：自己定义 MuJoCo 侧最小 `sim2sim` 程序骨架

忽略现有接口后，你自己的 MuJoCo 程序至少要有下面这些模块。

#### 5.1 配置层

你需要一份配置对象或配置文件，至少包括：

1. model path
2. policy path
3. backend type
4. sim dt
5. control dt
6. action scale
7. observation normalization 相关设置
8. 初始姿态和 reset 设置
9. 测试命令 `cmd_vx/cmd_vy/cmd_wz`

#### 5.2 资产加载层

负责：

1. 加载 MuJoCo model/data
2. 建立 joint name -> qpos/qvel/actuator index 映射
3. 建立 sensor / body / geom 索引

这层的目标是“所有后面需要的 index 映射都一次性固定”。

#### 5.3 策略加载层

负责：

1. 加载 `policy.pt` 或 `policy.onnx`
2. 统一输入输出接口
3. 接收 `27D obs`
4. 输出 `6D action`

你后面最好让这个层完全独立，这样将来可以替换不同 backend，而不碰仿真主循环。

#### 5.4 observation builder

这是 `sim2sim` 最核心的模块之一。

它必须严格按训练时顺序输出 observation。

对当前双轮足 base-locked 任务，你至少要按顺序组织：

1. `base_ang_vel`
2. `projected_gravity`
3. `velocity_commands`
4. `joint_pos`
5. `joint_vel`
6. `last_action`

同时你必须明确：

1. 所有量是在世界系还是机体系
2. projected gravity 的计算坐标系
3. joint pos 是相对默认零位还是绝对值
4. last action 存的是 raw network action 还是 scale 后的 command

这一步如果有任何一点不严谨，策略在 MuJoCo 中的表现就会明显异常。

#### 5.5 action adapter

它负责把网络输出的 `6D action` 转成 MuJoCo 可执行控制量。

对当前任务，最少要处理：

1. `HipL/HipR/KneeL/KneeR` 的位置偏移控制
2. `HubL/HubR` 的轮速控制

这里要显式定义：

1. action 顺序
2. scale
3. offset
4. saturate / clip 规则
5. 对应 actuator 写入哪个 ctrl 通道

这层不能含糊。它必须是可打印、可检查、可单步验证的。

#### 5.6 reset / startup 模块

你需要自己决定 MuJoCo 中的启动方式，但推荐分开两个概念：

1. 训练语义 reset
2. 部署保护 startup

训练语义 reset 指：

1. base 初始位置
2. base 初始姿态
3. base 初始速度
4. joint 初值
5. joint 初速度

部署保护 startup 指：

1. 是否先挂起
2. 是否先 stand
3. 是否平滑 handoff 给 policy

这两个概念不要混在一起写，否则你后面排查时分不清问题来自训练语义还是来自保护逻辑。

#### 5.7 rollout 主循环

最小闭环主循环应该至少是：

1. reset
2. 读状态
3. build observation
4. policy inference
5. adapt action
6. 写 control
7. step simulator
8. 记录 last action
9. 记录日志
10. 重复

这条链必须尽量短、尽量透明，不要一开始就写成很多层抽象。

#### 5.8 日志与诊断层

没有日志，`sim2sim` 基本不可调。

最少要打的内容：

1. 仿真时间
2. base 位姿
3. base 线速度和角速度
4. 当前 command
5. 27 维 observation 分块
6. 6 维 raw action
7. 6 维 scaled action
8. 实际 actuator ctrl
9. 是否接触地面
10. 是否触发异常或终止

推荐第一版就支持：

1. 控制台摘要
2. JSONL 逐步日志

### 阶段 6：先做最小闭环，不做复杂保护

自己重写时最容易犯的错误，是一上来就加很多“稳定性补丁”。

你现在应该反过来：

先做一个最简单、最可解释的版本。

第一版目标只应该是：

1. 策略能加载
2. observation 维度正确
3. action 映射正确
4. MuJoCo 能持续 step
5. 不会在接入策略瞬间因为程序错误崩掉

第一版不要急着加：

1. wheel feedback
2. posture assist
3. heading hold
4. high-speed attenuation
5. emergency brake

这些都属于后验补丁。你还没证明基础语义对齐之前，不应该先引入它们。

### 阶段 7：做三轮验证

你自己重写 MuJoCo 侧后，验证要严格分三轮。

#### 第一轮：静态结构验证

不看是否跑得好，只看结构是否正确。

检查：

1. observation 维度是否为 `27`
2. action 维度是否为 `6`
3. joint 顺序是否正确
4. 左右轮方向是否正确
5. 腿部控制是不是写到了正确关节
6. 轮子控制是不是写到了正确 actuator

只要这一轮不过，后面都不要继续。

#### 第二轮：最小运动闭环验证

从最保守命令开始：

1. `cmd_vx = 0.0`
2. 再试 `cmd_vx = 0.1`
3. 再试 `cmd_vx = 0.2`

看三件事：

1. 启动是否稳定
2. 接入策略是否稳定
3. 低速前进是否方向正确

#### 第三轮：一致性与性能验证

只有第二轮过了，才进入这一步。

检查：

1. 速度跟踪误差
2. 前倾失稳模式
3. 轮速饱和
4. 接触异常
5. 和 Isaac Play 的行为差异

### 阶段 8：系统化做 Isaac/MuJoCo 对齐

当最小闭环能跑起来后，再开始系统化对齐。

你需要逐项建立下面四张表。

#### 8.1 observation 对齐表

每一项都要写清：

1. Isaac 中怎么定义
2. MuJoCo 中怎么计算
3. 单位是否一致
4. 坐标系是否一致
5. 是否加噪声

#### 8.2 action 对齐表

每一项都要写清：

1. 网络输出维度对应哪个关节
2. raw action 如何 scale
3. scale 后如何变成 position target 或 velocity target
4. MuJoCo actuator 具体如何接收

#### 8.3 reset 对齐表

每一项都要写清：

1. root pose
2. root velocity
3. joint pos
4. joint vel
5. 是否随机化

#### 8.4 执行器对齐表

每一项都要写清：

1. Isaac 中 actuator 类型
2. Isaac 中 stiffness / damping / velocity limit / effort limit
3. MuJoCo 中对应的实现方式
4. 这两者是“数值一致”还是“语义近似”

这一阶段的核心不是“调”，而是“证明你知道自己在对齐什么”。

### 阶段 9：只在必要时加入部署保护逻辑

当你完成基础语义对齐后，如果 MuJoCo 侧仍然存在接入瞬间摔倒、速度一高就失稳等问题，再考虑加保护逻辑。

保护逻辑的加入顺序应该是：

1. startup 软接入
2. command ramp
3. stand phase
4. 速度限幅
5. wheel balance feedback

但是这里有一个很重要的原则：

每加一个保护逻辑，都要能回答两件事：

1. 它是在修“程序错误”还是在补“动力学差异”？
2. 它会不会掩盖 observation/action/asset 本身没对齐的问题？

如果回答不清楚，就不要先加。

### 阶段 10：判断该继续调 MuJoCo，还是回头重训

这是最后一阶段。

当你发现：

1. 低速可以稳定
2. observation/action/reset/actuator 都已核对
3. 启动也已经稳定
4. 一到更高速度就稳定进入同一种失稳模式

那说明问题很可能已经不在“程序没搭好”，而在：

1. MuJoCo 与 Isaac 的接触动力学差异
2. 训练策略本身对新动力学的鲁棒性不足
3. 训练随机化范围不覆盖当前部署差异

到这个阶段，才讨论是否要重训、是否要改奖励、是否要改随机化。

## 3. 你真正应该按什么顺序开始

如果你现在就准备自己重写 MuJoCo `sim2sim`，建议严格按下面顺序启动，不要跳：

1. 先锁定唯一训练产物
2. 再整理训练基线表
3. 再确认 Isaac Play 中策略没问题
4. 再确认 MuJoCo 资产命名、关节顺序、轴方向
5. 再写最小配置层和资产加载层
6. 再写 policy loader
7. 再写 observation builder
8. 再写 action adapter
9. 再写最小 rollout loop
10. 再做三轮验证
11. 最后才做一致性对齐和保护逻辑

这就是最稳的顺序。

## 4. 当前与你这套仓库直接相关的已知训练事实

这些事实已经能从训练侧代码直接确认，后面你自己写 MuJoCo 时应优先围绕它们。

1. 训练模块位置：
   - `modules/wheeled_train/wheeled_leg`
2. base-locked 训练配置文件：
   - `modules/wheeled_train/wheeled_leg/source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/base_locked_locomotion_env_cfg.py`
3. 训练 observation 结构：
   - `base_ang_vel`
   - `projected_gravity`
   - `velocity_commands`
   - `joint_pos`
   - `joint_vel`
   - `last_action`
4. 训练动作结构：
   - `Hip.*` 和 `Knee.*` 为 position action
   - `Hub.*` 为 velocity action
5. 动作 scale：
   - leg `0.5`
   - wheel `10.0`
6. 训练步长：
   - `sim.dt = 1/120`
   - `decimation = 4`

这些是你自己重建 MuJoCo `sim2sim` 时最先要复刻的训练语义核心。

## 5. 当前动态状态

### 已完成

1. 已经把流程切换到“忽略现有 MuJoCo 接口程序，自行重建”的路线
2. 已经整理出你应该重建的完整 `sim2sim` 阶段顺序
3. 已经明确 MuJoCo 侧最小程序应该包含哪些模块

### 下一步

下一步不是改 MuJoCo 代码细节，而是先锁定：

1. 这次迁移的唯一 task
2. 唯一 run
3. 唯一 checkpoint
4. 唯一导出策略文件
5. 唯一 `env.yaml`

只有这五项固定，你后面自己写的 MuJoCo `sim2sim` 才有明确目标。
