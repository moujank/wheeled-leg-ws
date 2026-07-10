    ---step1: isaac 配置---

先查 Isaac 训练配置里的动作定义：

sed -n '80,140p' modules/wheeled_train/wheeled_leg/source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/base_locked_locomotion_env_cfg.py

wheeled_leg:

Hip.* 和 Knee.* : position action
    scale: 0.5
Hub.* : velocity action
    scale: 10.0

wheeled_arm:

Hip.* 和 Knee.* : position action
    scale: 0.5
Hub.* : velocity action
    scale: 10.0
arm.* : position action
    scale: 0.5
claw : position action
    scale: 0.5

再查训练资产默认零位和 actuator 参数：

sed -n '1,220p' modules/wheeled_train/wheeled_leg/source/wheeled_leg/wheeled_leg/tasks/manager_based/wheeled_leg/asset/wheeled.py
wheeled_leg:
linear_damping: 0.0
argular_damping: 0.0
init_state.pos :{0.0, 0.0, 0.32}
joint_pos :{
    "HipL": 0.0,
    "HipR": 0.0,
    "KneeL": 0.0,
    "KneeR": 0.0,
    "Hub.*": 0.0
}

Hip:
    stiffness:20.0
    damping:1.5
    velocity_limit:math.radians(360.0)
    effort_limit:150.0
Knee:
    stiffness:30.0
    damping:1.5
    velocity_limit:math.radians(360.0)
    effort_limit:150.0
Hub:
    stiffness:0.0
    damping:1.5
    velocity_limit:math.radians(360.0)
    effort_limit:150.0

wheeled_arm:

init_state.pos :
joint_pos :

Hip.* :
    stiffness:
    damping:
    velocity_limit:
    effort_limit:
Knee.* :
    stiffness:
    damping:
    velocity_limit:
    effort_limit:
Hub.* : 
    stiffness:
    damping:
    velocity_limit:
    effort_limit:
    
arm.* : 
    stiffness:
    damping:
    velocity_limit:
    effort_limit:
claw : 
    stiffness:
    damping:
    velocity_limit:
    effort_limit:

    ---step2: mujoco xml配置---
先直接从 XML 里把关键结构扫出来：
rg -n '<joint|<actuator|<position|<velocity|axis=|<body|<site|<sensor|<key'  modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml

  这一条会直接给你：

  - HipL/HipR/KneeL/KneeR/HubL/HubR
  - 每个 joint 的 axis
  - 每个 actuator 绑定了哪个 joint
  - imu site
  - home keyframe

  如果你想看完整上下文，用：

  sed -n '35,140p' modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml

  这一步你要人工确认：

  base_link 是 base body
  HipL/HipR/KneeL/KneeR/HubL/HubR 都存在
  左右轮符号差异：
  HubL axis="0 0 -1"
  HubR axis="0 0 1" 
  actuator 名称如 HipL_pos、HubL_vel



    ---step3: 查询 MuJoCo 中 joint 的 qpos/qvel 索引---

  这个信息单看 XML 不够，最稳的是直接用 MuJoCo Python API 读模型。
  执行：idx_qpos_qvel.py
  这一步能直接得到：

  - joint id
  - qpos 起始索引
  - qvel/dof 起始索引

HipL: joint_id=1, qpos_adr=7, qvel_adr=6
HipR: joint_id=4, qpos_adr=10, qvel_adr=9
KneeL: joint_id=2, qpos_adr=8, qvel_adr=7
KneeR: joint_id=5, qpos_adr=11, qvel_adr=10
HubL: joint_id=3, qpos_adr=9, qvel_adr=8
HubR: joint_id=6, qpos_adr=12, qvel_adr=11
HipL: id=1, type=HINGE, qpos_adr=7, qvel_adr=6, axis=[ 0.  0. -1.]
HipR: id=4, type=HINGE, qpos_adr=10, qvel_adr=9, axis=[ 0.  0. -1.]
KneeL: id=2, type=HINGE, qpos_adr=8, qvel_adr=7, axis=[ 0.  0. -1.]
KneeR: id=5, type=HINGE, qpos_adr=11, qvel_adr=10, axis=[ 0.  0. -1.]
HubL: id=3, type=HINGE, qpos_adr=9, qvel_adr=8, axis=[ 0.  0. -1.]
HubR: id=6, type=HINGE, qpos_adr=12, qvel_adr=11, axis=[0. 0. 1.]


    ---step4: 查询 actuator 的 ctrl 索引和 joint 绑定关系---

  MuJoCo 里 policy 最终写的是 data.ctrl[...]，所以你必须知道 actuator 索引。
  执行：idx_actuator.py

  - data.ctrl[aid] 里每个位置对应哪个 actuator
  - 这个 actuator 最终绑到哪个 joint
  - ctrlrange/forcerange 是多少

HipL_pos: actuator_id=0, ctrl_index=0, joint=HipL, ctrlrange=[-3.14  3.14], forcerange=[-150.  150.]
HipR_pos: actuator_id=1, ctrl_index=1, joint=HipR, ctrlrange=[-3.14  3.14], forcerange=[-150.  150.]
KneeL_pos: actuator_id=2, ctrl_index=2, joint=KneeL, ctrlrange=[-3.14  3.14], forcerange=[-150.  150.]
KneeR_pos: actuator_id=3, ctrl_index=3, joint=KneeR, ctrlrange=[-3.14  3.14], forcerange=[-150.  150.]
HubL_vel: actuator_id=4, ctrl_index=4, joint=HubL, ctrlrange=[-6.28319  6.28319], forcerange=[-150.  150.]
HubR_vel: actuator_id=5, ctrl_index=5, joint=HubR, ctrlrange=[-6.28319  6.28319], forcerange=[-150.  150.]

    ---step5: 查询 base body、imu site、sensor 的 id---

  后面你做 observation builder 一定会用到 body/site/sensor 索引。
  执行：idx_body_site_sensor.py

  - sensordata 里每个 sensor 的起始地址
  - 每个 sensor 占多少维

base_link: id=1
left_wheel_link: id=4
right_wheel_link: id=7
imu: id=0
base_com: id=1
left_wheel_site: id=2
right_wheel_site: id=3

0: imu_gyro, adr=0, dim=3
1: imu_linear_velocity, adr=3, dim=3
2: imu_linear_acceleration, adr=6, dim=3
3: imu_orientation, adr=9, dim=4
4: imu_position, adr=13, dim=3
5: imu_upvector, adr=16, dim=3
6: imu_global_linvel, adr=19, dim=3
7: imu_global_angvel, adr=22, dim=3
8: HipL_pos, adr=25, dim=1
9: HipL_vel, adr=26, dim=1
10: HipR_pos, adr=27, dim=1
11: HipR_vel, adr=28, dim=1
12: KneeL_pos, adr=29, dim=1
13: KneeL_vel, adr=30, dim=1
14: KneeR_pos, adr=31, dim=1
15: KneeR_vel, adr=32, dim=1
16: HubL_pos, adr=33, dim=1
17: HubL_vel, adr=34, dim=1
18: HubR_pos, adr=35, dim=1
19: HubR_vel, adr=36, dim=1
20: HipL_torque, adr=37, dim=1
21: HipR_torque, adr=38, dim=1
22: KneeL_torque, adr=39, dim=1
23: KneeR_torque, adr=40, dim=1
24: HubL_torque, adr=41, dim=1
25: HubR_torque, adr=42, dim=1

  如果你后面打算直接用 XML 里已经定义好的 imu_gyro、imu_orientation、jointpos、
  jointvel，这一步非常关键。

    ---step6： 查询 home pose 和 freejoint 相关的 qpos 布局---

  你必须知道 qpos 前 7 维是 freejoint，后面才是各个 hinge joint。
  执行：

  再看 XML 里 home keyframe：

  rg -n '<key name="home"' modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml

  这一步能帮助你确认：

  - qpos[0:7] 是 floating base
  - 后面的 6 个位置是不是你那 6 个关节
  - home pose 是否真的是你打算作为 reset 参考的零位

nq = 13
nv = 12
nu = 6
0 floating_base_joint qpos_adr= 0 qvel_adr= 0 type= 0
1 HipL qpos_adr= 7 qvel_adr= 6 type= 3
2 KneeL qpos_adr= 8 qvel_adr= 7 type= 3
3 HubL qpos_adr= 9 qvel_adr= 8 type= 3
4 HipR qpos_adr= 10 qvel_adr= 9 type= 3
5 KneeR qpos_adr= 11 qvel_adr= 10 type= 3
6 HubR qpos_adr= 12 qvel_adr= 11 type= 3

130:    <key name="home" qpos="0 0 0.34 1 0 0 0 0 0 0 0 0"/>


    ---step7：查询 joint 正负方向前，先确认 base 朝前轴---

  这个索引不一定要靠 API，看 body 姿态和实际试验更可靠。先从 XML 看 base_link 和 IMU：
  sed -n '35,90p' modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml
  然后建议你后面做一个最小运行实验，用 MuJoCo 打印：

  - xmat 或 base quaternion
  - base frame 下的前向轴
  - 左右轮给同号速度时 base 的线速度符号

  如果你想先查 body id：idx_body_id.py


---review: 当前审阅结论（2026-07-10）---

一、这份资产语义梳理里，已经确认正确的部分

1. 训练侧 action 语义主干是对的：
   - Hip.* / Knee.* 是 position action，scale = 0.5
   - Hub.* 是 velocity action，scale = 10.0

2. Isaac 默认零位和 actuator 参数主干是对的：
   - base init pos = (0, 0, 0.32)
   - HipL/HipR/KneeL/KneeR/Hub.* 默认 joint_pos 都是 0
   - Hip stiffness/damping = 20.0 / 1.5
   - Knee stiffness/damping = 30.0 / 1.5
   - Hub stiffness/damping = 0.0 / 1.5
   - 三类 actuator 的 effort_limit 都是 150
   - velocity_limit 都是 math.radians(360.0)

3. MuJoCo 资产关节集合和 actuator 集合是完整的：
   - joints: HipL, KneeL, HubL, HipR, KneeR, HubR
   - actuators: HipL_pos, HipR_pos, KneeL_pos, KneeR_pos, HubL_vel, HubR_vel

4. MuJoCo qpos/qvel 地址与你记录的一致：
   - HipL qpos_adr=7 qvel_adr=6
   - KneeL qpos_adr=8 qvel_adr=7
   - HubL qpos_adr=9 qvel_adr=8
   - HipR qpos_adr=10 qvel_adr=9
   - KneeR qpos_adr=11 qvel_adr=10
   - HubR qpos_adr=12 qvel_adr=11

5. MuJoCo actuator ctrl 索引与你记录的一致：
   - ctrl[0:6] = [HipL_pos, HipR_pos, KneeL_pos, KneeR_pos, HubL_vel, HubR_vel]

6. MuJoCo sensor 地址表与你记录的一致：
   - imu_* / joint pos / joint vel / actuator torque 的 sensordata 布局可直接用于 observation/debug


二、这一步还不完整的地方

1. 当前文档把 wheeled_arm 混进来了。
   你现在做的是 base_locked 的 wheeled_leg sim2sim，资产语义基线应该只保留这次迁移直接相关的对象。
   `wheeled_arm` 会分散注意力，建议从这份基线表中移除。

2. 缺少“最终用于 sim2sim 的唯一关节顺序总表”。
   现在文档里有分散的 joint / qpos / actuator 信息，但还没有明确写成一行最终结论：
   - policy action raw order: [HipL, HipR, KneeL, KneeR, HubL, HubR]
   - actuator ctrl order: [HipL_pos, HipR_pos, KneeL_pos, KneeR_pos, HubL_vel, HubR_vel]
   - qpos joint slice order: [HipL, KneeL, HubL, HipR, KneeR, HubR]
   这三个顺序不能混。

3. 缺少“哪些顺序来自训练定义，哪些顺序来自 MuJoCo 内部布局”的显式区分。
   这一步非常关键，否则后面很容易误把 qpos 顺序拿去当 action 顺序。

4. `idx_body_site_sensor.py` 在仓库中不存在。
   当前文档里把它当成现成步骤引用了，但仓库实际只有：
   - modules/wheeled_mujoco/asset/idx/idx_qpos_qvel.py
   - modules/wheeled_mujoco/asset/idx/idx_actuator.py
   - modules/wheeled_mujoco/asset/idx/idx_body_id.py
   所以这一段要么改成可复现的 `python -c` 查询，要么新增脚本。

5. “左右方向一致”还没有被真正证明。
   目前只能确认：
   - HipL/HipR/KneeL/KneeR axis 都是 [0, 0, -1]
   - HubL axis = [0, 0, -1]
   - HubR axis = [0, 0, 1]
   这说明左右轮在 MuJoCo 里采用了相反轴定义，但这不自动等价于“同号前进命令一定向前”。
   还需要做单步实测验证。

6. “默认站立姿态是否与训练零位一致”还没有被真正证明。
   目前只看到：
   - Isaac init_state.pos = (0, 0, 0.32)
   - Isaac 默认 joint_pos 全 0
   - MuJoCo base body 初始 pos = (0, 0, 0.32)
   - MuJoCo home keyframe base z = 0.34
   所以至少 base 高度已经出现了 0.32 / 0.34 两套值，需要明确你后面 reset 采用哪一个作为基线。

7. MuJoCo `home` keyframe 目前不能直接当可靠基线使用。
   模型 `nq = 13`，但 XML 里的 `<key name="home" ...>` 只写了 12 个 qpos 值。
   在修正前，不建议把这个 keyframe 当成 reset 参考语义。


三、这一步完成后，你应该沉淀出的最终结论

建议把最终资产语义表压缩成下面这 6 行，后面所有 sim2sim 代码都只认这份表：

1. 训练 action 语义顺序：
   [HipL, HipR, KneeL, KneeR, HubL, HubR]

2. 训练 action 类型与 scale：
   Hip/Knee -> position offset, scale=0.5
   Hub -> velocity target, scale=10.0

3. Isaac 默认零位：
   base pos = (0, 0, 0.32)
   all six joints default = 0

4. MuJoCo actuator ctrl 顺序：
   [HipL_pos, HipR_pos, KneeL_pos, KneeR_pos, HubL_vel, HubR_vel]

5. MuJoCo joint state 读取顺序（按 qpos/qvel 地址）：
   [HipL, KneeL, HubL, HipR, KneeR, HubR]

6. 仍需实测确认的方向项：
   - 同号 HubL/HubR velocity ctrl 是否得到前进
   - Hip/Knee 正方向是否与训练侧关节正方向一致
   - reset 时应采用 0.32 还是 0.34 的 base 高度


四、建议的最小对照顺序

1. 先只认训练 action 顺序：
   [HipL, HipR, KneeL, KneeR, HubL, HubR]

2. 再只认 MuJoCo ctrl 顺序：
   [HipL_pos, HipR_pos, KneeL_pos, KneeR_pos, HubL_vel, HubR_vel]

3. 明确写出 adapter：
   action[0] -> HipL_pos
   action[1] -> HipR_pos
   action[2] -> KneeL_pos
   action[3] -> KneeR_pos
   action[4] -> HubL_vel
   action[5] -> HubR_vel

4. 状态读取时不要偷懒按 qpos 切片直接复用 action 顺序。
   qpos/qvel 的内部顺序是：
   [HipL, KneeL, HubL, HipR, KneeR, HubR]
   读取后必须重排成训练 observation 需要的顺序。

base_link body id = 1


    ---step8：做“最小正负号实验”前需要的索引查询---

  后面你不加载 policy，直接手工写 ctrl 做测试时，最常用的查询命令是这两个。

  查 actuator 顺序：idx_joint_array.py

ctrl[0] -> HipL_pos
ctrl[1] -> HipR_pos
ctrl[2] -> KneeL_pos
ctrl[3] -> KneeR_pos
ctrl[4] -> HubL_vel
ctrl[5] -> HubR_vel
joint[0] -> floating_base_joint
joint[1] -> HipL
joint[2] -> KneeL
joint[3] -> HubL
joint[4] -> HipR
joint[5] -> KneeR
joint[6] -> HubR
