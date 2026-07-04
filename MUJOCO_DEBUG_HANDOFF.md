# MuJoCo Wheeled-Leg Sim2Sim Debug State

本文档是 `wheeled_mujoco` 这部分工作的状态文件，同时承担交接文档作用。后续每进一步调试一次，都需要同步更新本文档中的“当前阶段状态”“最近验证结果”“失败样例”“下一步动作”，避免重新打开 Codex、切换模型或跨轮调试后丢失上下文。

当前维护原则：

```text
1. 本文档记录的是当前可运行基线，而不只是历史尝试。
2. 任何新改动都要先写明是否破坏启动基线。
3. 任何非零速度验证都要同时区分“没倒下”和“真实速度跟踪是否恢复”。
4. 如果某个结果只是依靠保守降额获得稳定，必须在文档里明确写出。
```

## 1. 当前总目标

目标是在 MuJoCo 中稳定运行 Isaac 训练得到的双轮足机器人策略，当前主脚本是：

```bash
modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py
```

当前策略和模型默认路径：

```bash
model:  modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml
policy: modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.pt
```

核心目标按优先级排序：

1. 启动阶段必须稳定，启动和 reset 后机器人不能立刻倒下。
2. 保持静止阶段必须稳定，`cmd_vx=0.0` 应该能长期站立。
3. 小速度运行必须稳定，`cmd_vx=0.1` 和 `cmd_vx=0.2` 至少要完成 20s 长跑验证。
4. 中高速运行逐步稳定，当前重点是避免 `cmd_vx=0.5` 长跑前倾倒下，同时逐步恢复真实速度跟踪能力。
5. 尽量对齐 Isaac 和 MuJoCo 的模型、初始状态、观测、动作、控制尺度、接触、地面摩擦和 reset 语义。

## 1.1 当前状态摘要

```text
当前结论：
- 启动和 reset 后的 startup baseline 已统一。
- cmd_vx/cmd_vy/cmd_wz 当前按 body-frame 语义送入策略观测。
- cmd_vx=0.1 和 0.2 可稳定通过 20s。
- cmd_vx=0.5 当前 20s 内可不倒，但依赖运行期命令降额，不代表真实 0.5 速度跟踪已恢复。

当前主矛盾：
- 不再是“启动就倒下”。
- 也不再是“0.5 一定会在 16-17s 倒地”。
- 当前真正的问题是：如何在不破坏现有稳定性的前提下，减少运行期降额，恢复更真实的高速跟踪。
```

## 2. 当前不可破坏的稳定基线

用户已经明确要求：当前“启动并且保持静止”的阶段效果已经很好，后续调试必须保持不变。任何后续修改都必须先验证启动阶段未退化。

当前启动流程：

```text
1s tether -> 1s release -> 2s stand -> policy
```

当前默认可视化行为：

```text
默认打开 MuJoCo viewer
默认 playback_speed=0.5，只降低可视化播放速度，不改变 sim_dt
默认 duration=None，不传 --duration 时持续运行
```

当前默认策略阶段：

```text
mode=isaac_play
stand_strategy=policy_zero
startup_tether_s=1.0
tether_release_s=1.0
startup_stand_s=2.0
tether_height_target=0.32
stand_base_height_target=0.32
cmd_vx 默认 0.5
cmd_vy 默认 0.0
cmd_wz 默认 0.0
leg_action_scale=0.5
wheel_action_scale=10.0
```

启动阶段验证命令：

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --no-viewer \
  --duration 4.5 \
  --cmd-vx 0.0
```

当前已知启动阶段稳定样例：

```text
t=4.00s:
base_pos=(+0.007, -0.002, +0.324)
rpy_deg=(-0.4, -2.3, -2.8)
状态：仍然直立，启动/释放/站立/进入 policy 的过渡没有明显失稳
```

后续任何调参前后，都必须先跑上面的启动回归命令。如果该命令失稳，优先回退运行阶段以外的改动，不能继续调高速。

## 3. 当前验证标准

用户要求后续验证分两类：

```text
短跑验证：6s
长跑验证：20s
只要是非零速度指令，都必须做 20s 长跑验证
后续短跑验证至少 6s
```

推荐验证命令：

```bash
# 启动阶段回归
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer --duration 4.5 --cmd-vx 0.0

# 短跑
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer --duration 6 --cmd-vx 0.1
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer --duration 6 --cmd-vx 0.2
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer --duration 6 --cmd-vx 0.5

# 长跑
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer --duration 20 --cmd-vx 0.1
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer --duration 20 --cmd-vx 0.2
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer --duration 20 --cmd-vx 0.5
```

判定标准：

```text
稳定：base_contact=False，leg_contact=False，base_z 约 0.32-0.33，pitch/roll 未持续发散，轮速未长期饱和
倒下：base_contact=True 或 leg_contact=True，pitch 接近几十度以上，base_z 明显下降，ctrl_sat 轮子长时间饱和
偏航问题：yaw 长时间单向累积，说明左右轮、yaw 反馈、初始 heading 或接触不对称仍需排查
```

## 4. 历史坏样例：启动后直接倒下

这是用户提供的旧日志，属于“旧 broken startup/handoff baseline”。这段数据必须保留，后续调试如果重新出现类似现象，说明启动或 handoff 回归。

运行命令：

```bash
python modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --cmd-vx 0.0
```

当时配置摘要：

```text
model: nq=13 nv=12 nu=6
policy backend=TorchPolicyRunner
command: vx=0.000 vy=0.000 wz=0.000
dt=0.008333 decimation=4
mode=isaac_play
stand_strategy=pose_hold
leg_scale=0.50
wheel_scale=10.00
playback_speed=0.50
tether_s=1.00
release_s=0.00
stand_s=0.00
```

旧失败关键过程：

```text
t=0.00s phase=tether
base_pos=(+0.000, +0.000, +0.320)
rpy_deg=(+0.0, +0.0, +0.0)
base_contact=False leg_contact=False
last_action=[0 0 0 0 0 0]

t=1.00s phase=policy
cmd=(+0.00,+0.00,+0.00)
base_pos=(-0.000, -0.000, +0.320)
base_linvel=(-0.006, -0.002, -0.032)
rpy_deg=(+0.0, +0.0, -0.0)
base_contact=False leg_contact=False
last_action=[ 0.105 -0.049 -0.430  0.434  0.005  0.003]

t=2.00s phase=policy
base_pos=(-0.118, +0.000, +0.315)
base_linvel=(-0.414, -0.004, +0.018)
rpy_deg=(+0.1, -14.4, -0.6)
hub_vel=(+1.316, +1.444)
base_contact=False leg_contact=False
last_action=[-0.271  0.290 -0.668  0.601  0.400  0.400]

t=3.00s phase=policy
base_pos=(-0.615, +0.019, +0.286)
base_linvel=(+0.029, -0.006, -0.013)
rpy_deg=(-179.5, -72.6, +171.4)
hub_vel=(+3.142, +1.198)
base_contact=False leg_contact=True
ctrl_sat=[0, 0, 0, 0, 1, 0]
last_action=[-0.536  0.556 -1.000  0.573  0.600  0.600]
```

坏样例判断：

```text
1. t=1s 刚进入 policy 后仍无接触，但 pitch 开始快速偏移。
2. t=2s pitch 已到 -14.4 deg，速度和轮速开始失控。
3. t=3s pitch 到 -72.6 deg，leg_contact=True，姿态已经倒下。
4. 后续 t=4s 到 t=9s 基本处于倒地状态，轮子控制逐步饱和。
```

这段坏样例的关键特征是：policy 接入太早或 handoff 太硬，且 `cmd_vx=0.0` 也会倒，说明当时不是“速度太大”的问题，而是启动/初始状态/控制接入不匹配。

## 5. 历史好样例：reset 后稳定站立

这是用户提供的稳定站立片段。用户明确说明“后面第二段是正常站立状态下的打印数据”。后续如果要判断稳定站立，应与这段对比。

稳定站立关键样例：

```text
t=12.00s phase=policy
cmd=(+0.00,+0.00,+0.00)
base_pos=(-0.318, +0.003, +0.330)
base_linvel=(-0.002, -0.000, -0.000)
rpy_deg=(-0.1, -2.3, -7.3)
hub_vel=(+0.040, -0.011)
base_contact=False leg_contact=False
ctrl_sat=[0, 0, 0, 0, 0, 0]
last_action=[ 0.027 -0.060 -0.963  0.990  0.012  0.011]

t=13.00s phase=policy
base_pos=(-0.318, +0.003, +0.330)
base_linvel=(+0.001, -0.001, -0.000)
rpy_deg=(-0.1, -2.3, -7.3)
hub_vel=(-0.044, +0.052)
base_contact=False leg_contact=False
last_action=[ 0.048 -0.077 -0.953  0.972  0.010  0.011]

t=14.00s phase=policy
base_pos=(-0.318, +0.003, +0.330)
base_linvel=(+0.002, +0.000, -0.000)
rpy_deg=(-0.1, -2.3, -7.3)
hub_vel=(+0.012, -0.061)
base_contact=False leg_contact=False
last_action=[ 0.034 -0.077 -0.962  0.973  0.012  0.013]

t=19.00s phase=policy
base_pos=(-0.319, +0.003, +0.330)
base_linvel=(+0.000, -0.000, +0.000)
rpy_deg=(-0.1, -2.3, -7.3)
hub_vel=(-0.020, -0.047)
base_contact=False leg_contact=False
last_action=[ 0.029 -0.070 -0.962  0.982  0.012  0.013]
```

稳定样例判断：

```text
1. base_z 稳定在约 0.330。
2. base_linvel 接近 0。
3. pitch 稳定在约 -2.3 deg。
4. roll 接近 0。
5. yaw 固定在约 -7.3 deg，没有继续明显漂移。
6. base_contact=False 且 leg_contact=False。
7. 轮速很小，没有饱和。
```

这段数据说明策略本身在某些状态下可以稳定站立，因此问题不应简单归因于“策略完全不可用”。更准确的判断是：MuJoCo 中启动/handoff/初始状态/运行阶段速度闭环与 Isaac 训练分布仍存在不一致。

## 6. 当前已经完成的重要改动和成果

当前已完成的方向：

```text
1. MuJoCo 环境中支持 PyTorch，默认可加载 .pt 策略。
2. README 中已经写入默认运行命令。
3. 默认打开 MuJoCo viewer。
4. 默认不传 --duration 时持续运行。
5. 默认 playback_speed=0.5，减慢视觉播放但不改变 sim_dt。
6. 地面材质已调整为避免金属光泽反光。
7. wheeled_leg.xml 和 wheeled_arm.xml 中已经围绕地形、接触、mesh collision 做过调整。
8. 用户要求实体 mesh 作为碰撞体，不想用等效碰撞箱，这一方向已执行过。
9. 初始/重置高度经历过 0.36、0.32、0.30 的尝试，当前有效基线回到 0.32 左右。
10. 当前启动阶段已经稳定，这是最重要成果，后续不要破坏。
```

当前脚本中的运行阶段稳定化手段：

```text
1. command soft start：策略阶段速度命令缓慢接入。
2. action soft start：策略动作在 handoff 初期做缩放。
3. early wheel action limit：策略初期限制轮子动作，避免 handoff 后轮速瞬间过大。
4. pitch command attenuation：pitch 变大时降低前向命令。
5. yaw command attenuation：heading 偏差变大时降低前向命令。
6. wheel speed command attenuation：平均轮速过大时降低前向命令。
7. high speed requested command downscale：请求速度超过阈值后整体降额。
8. wheel balance feedback：policy 阶段额外叠加薄层 pitch/yaw 轮速反馈。
```

需要注意：这些运行阶段补偿是为了让 MuJoCo sim2sim 先稳定起来，并不等价于最终部署方案。最终仍应回到 Isaac-MuJoCo 严格一致性排查。

## 7. 最近一次已知验证结果

最近一轮调紧运行阶段参数后的短跑验证：

```text
cmd_vx=0.1, duration=6s:
t=5.00s base_pos=(+0.002, -0.003, +0.328)
rpy_deg=(-0.2, -1.8, -3.3)
状态：稳定

cmd_vx=0.2, duration=6s:
t=5.00s base_pos=(+0.010, -0.004, +0.328)
rpy_deg=(-0.1, -1.5, -3.5)
状态：稳定

cmd_vx=0.5, duration=6s:
t=5.00s base_pos=(+0.019, -0.005, +0.328)
rpy_deg=(-0.2, -1.3, -3.5)
状态：稳定，但早期高速度命令已被保守处理
```

最近一轮调紧前的 20s 长跑验证：

```text
cmd_vx=0.1:
稳定通过 20s，但 t=19s yaw 漂移约 +22.8 deg。

cmd_vx=0.2:
稳定通过 20s，但 t=19s yaw 漂移约 +27.5 deg。

cmd_vx=0.5:
约 t=11s 到 t=12s 之间倒下。
t=12.00s base_pos=(+1.600, +0.038, +0.204)
pitch=+81.1 deg
base_contact=True
状态：失败，前倾/接触倒地。
```

最近一轮调紧后的 20s 长跑验证：

```text
cmd_vx=0.1:
t=10.00s base_pos=(+0.332, -0.005, +0.328), yaw=+1.8 deg
t=15.00s base_pos=(+0.767, +0.042, +0.328), yaw=+9.3 deg
t=19.00s base_pos=(+1.073, +0.122, +0.328), yaw=+18.9 deg
状态：稳定通过 20s，yaw 漂移比之前 +22.8 deg 有改善，但仍存在。

cmd_vx=0.2:
t=10.00s base_pos=(+0.408, -0.008, +0.328), yaw=+1.2 deg
t=15.00s base_pos=(+0.935, +0.018, +0.327), yaw=+4.3 deg
t=19.00s base_pos=(+1.376, +0.072, +0.327), yaw=+9.2 deg
状态：稳定通过 20s，yaw 漂移比之前 +27.5 deg 明显改善。

cmd_vx=0.5:
t=10.00s base_pos=(+0.524, -0.016, +0.327), yaw=-0.3 deg，仍直立
t=15.00s base_pos=(+1.416, +0.016, +0.326), yaw=+4.5 deg，仍直立
t=16.00s base_pos=(+1.652, +0.037, +0.314), pitch=+6.3 deg，出现前倾警告
t=17.00s base_contact=True，灾难性倒地
状态：仍失败，但比之前约 t=11-12s 倒下有改善。
```

当前结论：

```text
1. 启动阶段目前是可用基线。
2. cmd_vx=0.0 静止站立可以稳定。
3. cmd_vx=0.1 和 0.2 当前可通过 20s，但 yaw 漂移仍需继续压低。
4. cmd_vx=0.5 当前可以撑到约 16-17s，但仍会前倾倒下。
5. 速度越低越稳定，速度越高越容易在一段距离后前倾，说明运行阶段轮速/姿态能量累积和 pitch 抑制仍不足。
```

## 8. 当前问题分解

当前现象不是“开局立刻失败”，而是更具体的运行阶段问题：

```text
1. 启动和静止已经基本解决。
2. 有速度时能跑一段距离，但速度越高越容易后期前倾。
3. 低速能跑更远，高速更早倒。
4. 运行过程中存在 yaw 漂移，不完全直线向前。
5. cmd_vx=0.5 的失败前兆是 pitch 先上升、base_z 略下降、轮速/控制开始积累，然后 base_contact=True。
```

这说明主要矛盾已经从“初始状态/handoff”转移到：

```text
1. MuJoCo 中轮子速度控制与 Isaac 策略期望不一致。
2. 机器人运行时 pitch 能量积累，前向命令和轮速反馈抑制不够及时。
3. 左右轮或接触存在微小不对称，导致 yaw 漂移。
4. 策略在 Isaac 中训练的动力学、接触、摩擦、drive 参数、default state 与 MuJoCo 仍有差别。
```

## 9. Isaac 与 MuJoCo 不一致风险清单

后续排查时应逐项核对以下内容。

模型结构：

```text
1. link 名称和层级是否与 Isaac 训练模型一致。
2. joint 顺序是否完全一致：HipL HipR KneeL KneeR HubL HubR。
3. actuator 顺序是否完全一致：HipL_pos HipR_pos KneeL_pos KneeR_pos HubL_vel HubR_vel。
4. 左右腿和左右轮的正方向是否一致。
5. base_link 坐标系、IMU 坐标系和 Isaac 中 body frame 是否一致。
```

初始状态：

```text
1. root pos z 当前主要使用 0.32，历史尝试过 0.30 和 0.36。
2. root quat/yaw 是否与 Isaac reset 后一致。
3. joint default 是否为 0。
4. root linear/angular velocity 是否为 0，或是否复刻 Isaac randomization。
5. reset 后是否在同一姿态进入策略。
```

观测：

```text
1. obs_dim=27。
2. obs 顺序：base_ang_vel(3), projected_gravity(3), command(3), joint_pos_rel(6), joint_vel_rel(6), last_action(6)。
3. base_ang_vel 是否与 Isaac 的 body frame 一致。
4. projected_gravity 的方向、符号和旋转矩阵是否一致。
5. command 是否使用策略实际看到的衰减后命令，而不是原始命令。
6. joint_pos_rel 是否减了与训练一致的 default_joint_pos。
7. joint_vel 单位和符号是否一致。
8. last_action 是否在 reset/handoff 时与 Isaac 行为一致。
```

动作和控制：

```text
1. 腿部 action scale 当前为 0.5。
2. 轮子 action scale 当前为 10.0。
3. 腿部是位置控制，轮子是速度控制。
4. MuJoCo actuator ctrlrange、kp/kv/damping/frictionloss 是否与 Isaac drive 近似一致。
5. wheel velocity actuator 的实际力矩限制是否会导致策略预期和 MuJoCo 行为不同。
6. action clip 当前为 [-1, 1]。
```

接触和地面：

```text
1. 用户要求实体 mesh 作为碰撞体，不使用简化碰撞箱。
2. mesh collision 更真实，但也可能引入接触点抖动和非凸碰撞差异。
3. 地面摩擦、condim、solref、solimp、contact stiffness/damping 需要与 Isaac 尽量对齐。
4. 轮子接触材质、摩擦方向、滚动阻力会强烈影响双轮足平衡。
5. base/leg contact 检测当前依赖 geom 名称，需要确保所有碰撞 geom 命名可被准确识别。
```

动力学：

```text
1. link mass、inertia 是否与 URDF/Isaac 导入后一致。
2. mesh collision 是否影响惯性，或者 inertia 是否来自原 URDF。
3. joint damping/armature/frictionloss 是否与 Isaac 一致。
4. MuJoCo timestep 当前 sim_dt=1/120，decimation=4；策略频率应与训练一致。
5. Isaac 和 MuJoCo 的积分器、接触求解器差异无法完全消除，需要通过参数对齐和策略鲁棒性处理。
```

## 10. 判断是模型问题还是策略问题的方法

优先用以下顺序排查。

第一步：固定策略输入，检查静态站立是否可重复。

```text
命令：cmd_vx=0.0，duration >= 20s。
如果稳定：说明策略在当前 MuJoCo 模型上至少能维持某些站立状态。
如果不稳定：优先查初始状态、观测符号、joint 顺序、动作符号、模型重心和接触。
```

第二步：低速递增，记录失败阈值。

```text
cmd_vx=0.1 -> 0.2 -> 0.3 -> 0.5
每个非零速度至少 20s。
如果低速稳定、高速失败：通常是速度/轮控/接触/能量积累问题，不是策略完全失效。
如果所有速度都失败：优先回到动作方向、轮子方向、观测符号和模型参数。
```

第三步：对比 MuJoCo 和 Isaac 同一时刻的观测。

```text
在策略接入前、接入瞬间、运行 1s、运行 5s 分别记录 27 维 obs。
重点看 projected_gravity、base_ang_vel、joint_pos_rel、joint_vel_rel、last_action。
如果 obs 差异明显，先修观测/初始状态。
如果 obs 接近但行为不同，查动力学、接触、actuator 和摩擦。
```

第四步：断开策略，测试模型和控制器。

```text
用固定零动作、固定站立姿态、固定小轮速测试。
如果固定控制下模型也异常漂移或倒下，偏模型/接触/actuator 问题。
如果固定控制稳定但策略不稳定，偏策略部署输入输出或训练分布问题。
```

第五步：左右对称性测试。

```text
检查左右轮同命令时 hub_vel 是否一致。
检查 cmd_wz=0 时 yaw 是否仍单向漂移。
检查左右 hip/knee action 正负号是否镜像正确。
如果 yaw 单向累积，优先查左右轮半径、摩擦、collision mesh、actuator gear/ctrlrange 和 joint axis。
```

## 11. 当前下一步调试方向

短期目标：

```text
1. 保持当前启动阶段完全不变。
2. 在运行阶段继续修正 cmd_vx=0.5 约 16-17s 后前倾倒下。
3. 同时继续压低 cmd_vx=0.1 和 0.2 的 yaw 漂移。
```

建议下一步动作：

```text
1. 先跑启动回归，确认 4.5s 仍稳定。
2. 跑 cmd_vx=0.5 的 20s 长跑，重点观察 t=14s 到倒地前的 pitch、base_z、hub_vel、ctrl_sat、last_action。
3. 对 cmd_vx=0.5 增加更早的 pitch 正向抑制或 wheel braking，但只作用于 policy running phase。
4. 对 yaw 漂移继续做左右轮差分反馈或检查左右轮模型/接触不对称。
5. 每做一次调参，都重新跑 0.1、0.2、0.5 的 6s 短跑；非零速度最终跑 20s。
```

不要优先做的事：

```text
1. 不要再破坏启动阶段。
2. 不要把问题重新归因到“轮子锁死”，用户已经明确不建议沿这个方向诊断。
3. 不要用等效碰撞箱替代实体 mesh collision，用户明确要求实体模型就是碰撞体。
4. 不要只看 viewer 主观效果，必须保留关键数值日志。
```

## 12. 后续文档更新规则

每次进一步调试后，都必须更新本文档。这里的“更新”是手工维护交接状态，不是在代码里自动追加。

每次更新至少填写：

```text
1. 修改了什么文件、什么参数或什么逻辑。
2. 是否影响启动阶段。
3. 启动回归命令和结果。
4. 6s 短跑结果。
5. 非零速度 20s 长跑结果。
6. 是否出现 base_contact 或 leg_contact。
7. 倒下前最后 2-3 秒的 base_z、pitch、yaw、hub_vel、ctrl_sat、last_action。
8. 当前结论和下一步。
```

推荐在下面“调试流水”里追加记录。

## 13. 调试流水

### 2026-07-03 当前交接状态

```text
阶段：启动/静止已稳定，运行阶段继续调。

稳定项：
- 启动流程 1s tether -> 1s release -> 2s stand -> policy 当前可用。
- cmd_vx=0.0 可稳定站立。
- cmd_vx=0.1 和 cmd_vx=0.2 最近可通过 20s。

仍存在的问题：
- cmd_vx=0.5 可跑到约 16-17s，但之后前倾倒下。
- cmd_vx=0.1 和 cmd_vx=0.2 仍有 yaw 漂移，只是比之前改善。
- 高速当前比较保守，6s 能稳定不代表真实高速能力已经完全恢复。

下一步：
- 保持启动阶段不动。
- 针对 cmd_vx=0.5 的 t=14-17s 前倾前兆继续调运行阶段 pitch/wheel 抑制。
- 同步检查左右轮/接触/yaw 不对称。
```

### 2026-07-03 reset/命令语义/运行期保护调整

```text
修改文件：
- modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py

修改内容：
1. 新增统一 episode reset 路径：run() 开始时重新走 reset_to_home -> startup anchor/reference 初始化，
   保证 reset 后和刚启动时使用同一套 tether/release/stand 初始条件与内部状态。
2. 将 startup reference 的 tether anchor / heading reference 提取为 reset 后统一刷新，
   避免只在 __init__ 时初始化一次。
3. 明确保留 command 观测通道的 Isaac 语义：obs[6:9] 直接是 body-frame 的
   [cmd_vx, cmd_vy, cmd_wz]，没有在 sim2sim 里先投影成世界系巡航向量。
4. 默认关闭额外 heading-hold command 注入，避免把 body-frame 线速度命令和
   “保持世界朝向”绑定在一起；当前 yaw 稳定仍主要依赖原有的运行期保护与轮反馈。
5. 保留较保守的运行期线速度衰减链路，使高速请求在 pitch / wheel speed / requested speed
   风险升高时被下调，从而先保证不倒。

是否影响启动阶段：
- 已回归验证，未破坏当前 1s tether -> 1s release -> 2s stand -> policy 基线。

启动回归：
- 命令：
  python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer --no-real-time --no-handoff-log-every-step --duration 4.5 --cmd-vx 0.0
- 结果：
  t=4.00s base_pos=(+0.007, -0.002, +0.324)
  rpy_deg=(-0.4, -2.3, -2.8)
  base_contact=False leg_contact=False

短跑结果：
- cmd_vx=0.1, duration=6s：
  t=5.00s base_pos=(-0.009, -0.003, +0.328)
  rpy_deg=(-0.2, -1.8, -4.2)
  状态：稳定
- cmd_vx=0.5, duration=6s：
  t=5.00s base_pos=(+0.009, -0.004, +0.328)
  rpy_deg=(-0.2, -1.3, -4.1)
  状态：稳定

20s 长跑结果：
- cmd_vx=0.1：
  稳定通过 20s。
  t=19.00s base_pos=(+1.044, +0.016, +0.327), yaw=+9.2 deg
  说明：yaw 漂移仍存在，但小于旧 baseline 的二十多度。
- cmd_vx=0.2：
  稳定通过 20s。
  t=19.00s base_pos=(+1.335, -0.012, +0.326), yaw=+3.2 deg
  说明：运行期会周期性把有效 cmd 压到 0 或约 0.16，以换取稳定性。
- cmd_vx=0.5：
  当前 20s 内未倒下，base_contact=False leg_contact=False。
  t=19.00s base_pos=(+2.057, -0.364, +0.316), pitch=+5.7 deg, yaw=-12.5 deg
  说明：这是“保守稳定”而不是“真实 0.5 跟踪恢复”。
  运行期日志可见有效 cmd 在 0.00 和约 0.26 之间频繁切换，说明保护层正在主动降额。

当前结论：
1. reset 后的运行流程现在和刚启动时一致，不再只有首次启动才使用完整 startup baseline。
2. 当前 sim2sim 里的速度指令观测保持 body-frame 语义，不是世界系巡航方向向量。
3. 高速前倾倒地目前被压住了，但代价是有效前向命令经常被显著下调。
4. 因此当前阶段更像“先保不倒”的过渡方案，高速速度跟踪精度仍需继续恢复。

下一步：
1. 保持 reset/startup 基线不动。
2. 对 cmd_vx=0.5 增加“有效 cmd / 实际 base_x 速度 / pitch / wheel speed”联合日志，
   区分是 pitch 保护、wheel 保护还是 requested speed downscale 在主导降额。
3. 在不让机器人前倾倒地的前提下，逐步恢复 cmd_vx=0.5 下的有效 cmd 占空比，
   而不是继续单纯依赖大幅降额保稳定。
```
