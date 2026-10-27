# 在未知场景中边移动、边建图、边选目标

这条流程从空白地图开始。机器人先读取 Isaac Sim 的真实传感器回波，找出已知空间与未知空间的交界，再选择新的观察位置。新的扫描会改变地图，地图又会改变下一次目标和路径。代码不接受预定游览路线，也不把 Code-as-Room 的家具坐标交给探索器。

本机已经实际运行了两种版本：第一种用 Python 栅格建图和在线 A* 检查探索闭环；第二种运行独立的 ROS 2 Humble、SLAM Toolbox、Nav2，加入有误差的里程计，由 Nav2 输出速度。两种版本都使用简化运动执行器：根据线速度和角速度逐步积分底座位姿。它们还没有调用 RL checkpoint，也没有验证双轮足的平衡、关节力矩或真实接触动力学。

## 场景、机器人和传感器

- Isaac Sim 4.5.0：`D:/isaacsim/python.bat`。这条流程不依赖 Isaac Lab 的训练接口。
- Code-as-Room 场景：工作区 `research/scene_audit/code_as_room/scene.usda`，引用已经转换好的家具和碰撞体。三间隔室之间的两道内墙保留。
- 机器人：`modules/wheeled_description/wheeled_arm_v2` 对应的弯腿、收臂可视资产，工作区 `research/scene_audit/code_as_room/wheeled_arm_v2_bent_visual.usdc`。没有缩放模型。
- 传感器：Isaac 原生 PhysX 三维激光雷达，360° 水平视场、140° 垂直视场，默认 720 个方位角、141 个俯仰通道，量程 0.15～8 米。
- 雷达结果投影：选取离地 0.06～1.12 米的真实三维命中点，每个方位角取最近的水平距离，形成覆盖机器人身体高度的虚拟平面扫描。这样可以看到低矮凳子和薄床罩。它是三维回波的保守投影，不对应某一款商用二维雷达，也不等于完整三维重建。

投影会遮掉同一方位更远、更高的回波，因此当前二维地图的观察面积不能代表所有三维回波看见过的表面。后续可以把用于定位的水平扫描和用于避障的身体高度投影分开处理；当前实现仍共用上述投影。

## 为什么原来只在中间通道来回走

原先用半径 0.38 米的圆表示机器人，占据宽度为 0.76 米。弯腿、收臂模型的完整水平包络约为 0.50 × 0.53 米。床尾一处间隙约为 0.62 米；在这类间隙里，圆形近似会禁止原本可能通过的位置。

ROS 版本现在使用 `scan_nav/footprint.py` 中的完整矩形轮廓，另加 1 厘米安全余量。Nav2 使用 Smac Hybrid A*，同时考虑位置和朝向，控制器允许沿路径倒车。探索目标还会经过完整轮廓检查和实际 Nav2 路径预检查。远处的探索目标可以拆成根据当前地图生成的较近观察位置；这些位置仍随扫描更新，不是提前写好的路线。

矩形外形并不意味着任何狭窄入口都能通过。机器人进入通道前还要有足够空间调整朝向；不够宽的真实入口仍应拒绝。左侧入口附近的小桌子也是实际障碍。不能通过缩小模型或关闭碰撞来证明可达。

实际测试还出现过 SLAM 栅格边缘比最新雷达回波向通道内多占几厘米的情况。在宽阔区域，这点差别通常不会阻断路线，在窄通道中却可能让规划器认为当前姿态已经碰撞。全局代价地图现在允许最新雷达实际扫过的单元修正静态地图，未观测位置不会因此被直接清空；同时关闭了“仅凭机器人处于某位置就清空其轮廓内障碍”的选项。局部避障仍检查完整身体和传感器回波。

## 启动

从仓库根目录 `wheeled-leg-ws` 打开 PowerShell。每次使用新的输出目录，启动器会保存配置、日志、扫描和结果，避免覆盖已有实验。

运行实际 SLAM Toolbox / Nav2：

```powershell
D:/Miniconda/python.exe modules/wheeled_navigation/scripts/run_autonomy.py --stage ros2 --scene code-as-room --seconds 420 --output modules/wheeled_navigation/artifacts/my_ros_exploration
```

运行不依赖 ROS 的第一阶段验证，并在运行途中放入一个原来不存在的障碍物：

```powershell
D:/Miniconda/python.exe modules/wheeled_navigation/scripts/run_autonomy.py --stage native --scene validation --seconds 240 --dynamic-test --output modules/wheeled_navigation/artifacts/my_native_exploration
```

`validation` 是有隔墙、门洞和家具的独立测试场景，便于检查地图和路径是否会随新障碍改变。动态障碍的位置仅由仿真端创建，不会直接通知规划器。需要检验噪声时，可以加 `--range-noise 0.005`；这表示回波距离上加入标准差 5 毫米的高斯误差。

ROS 版本可以通过 `--controller rpp` 或 `--controller dwb` 比较路径跟随器。RPP 直接跟随全局路径；DWB 会评估一批候选速度轨迹，并通过 `ObstacleFootprint` 检查每条轨迹上的完整身体。控制器选择和本次参数会写入 `run_config.json`，不能把一种配置的实验结果直接当作另一种配置已经通过验证。

ROS 运行环境位于工作区 `research/ros2_runtime/.pixi/envs/default`。其依赖清单位于 `research/ros2_runtime/pixi.toml`，锁文件位于同目录。重新安装时使用：

```powershell
.\research\runtime\pixi\pixi.exe install --manifest-path research/ros2_runtime/pixi.toml
```

运行器会为本轮仿真分配独立 ROS domain，并隔离 Isaac Python 3.10 与 ROS Python 3.12 的环境。SLAM 和导航采用传统算法，不需要下载神经网络权重。不要同时启动两个占用 GPU 的 Isaac 实例。

## 两条闭环各自验证什么

第一阶段：PhysX 回波 → 空白占据栅格逐步更新 → 选择探索目标 → 在线 A* → 路径跟随输出速度 → 位姿积分执行 → 新回波。它使用理想定位，检验探索、地图更新、绕障和返回起点的闭环。其圆形避障较保守，不适合用来判断本房间所有窄通道的极限通过能力。

第二阶段：PhysX 回波、带误差里程计、TF 和仿真时钟 → SLAM Toolbox → `/map` → 探索目标 → Nav2 Smac Hybrid A* / DWB（可选 RPP）→ `/cmd_vel` → 执行器 → 新回波。里程计默认有 3% 距离比例误差，移动时额外有 0.0015 rad/s 的航向误差。SLAM 和 Nav2 不订阅仿真真值，也不读取源场景几何。

探索器按以下依据挑选目标：

1. 只从当前扫描得到的地图中找已知与未知的交界。
2. 排除当前无法到达、完整身体放不下或近期已经失败的位置。
3. 在测量地图中模拟候选位置的可见方向。已知墙体会挡住预计视线，已经看过的区域不再计入新信息。
4. 综合预计新增观察面积、路径长度和访问记录选择目标。Nav2 再检查完整身体与朝向是否存在可行路径。
5. 移动过程中持续更新地图和路径，遇到障碍时停止或重选目标。长时间没有新增观察或达到时间预算时，尝试返回起点。

`phase=complete` 表示本轮探索结束后成功返回起点，不表示整间屋子已达到 100% 覆盖。`return_failed`、`blocked_by_measured_map`、时间耗尽等情况会保留为未完成状态。

## 数据和复查

每轮结果目录包含：

| 文件 | 用途 |
| --- | --- |
| `run_config.json`、`status.json` | 参数、最终状态、运行指标和明确的真值使用边界 |
| `motion.json` | 每一步执行速度、真值位姿、误差里程计、停止记录；真值仅供仿真执行及事后评价 |
| `sensor_history.npz` | 实际原始三维距离、投影扫描、时间和评价用位姿 |
| `measured_baseline/` | 事后对齐比较用的扫描地图；ROS 模式绝不发布给导航栈 |
| `map_progress.npz` | 运行中的地图快照，用于检查新增观察，而不是从最终图反推过程 |
| `ros/events.json`、`ros/plans.json`、`ros/commands.json` | 实际导航目标、结果、Nav2 路径和速度消息 |
| `ros/slam_map/` | SLAM Toolbox 实际输出地图 |
| `ros/stack.log`、`ros_console.log`、`isaac.log` | 各进程真实日志 |
| `online_scene.usda` | 场景与机器人末态，可在 Isaac 中打开；它不是运动回放 |

生成实验图和评价报告：

```powershell
D:/isaacsim/python.bat modules/wheeled_navigation/scripts/analyze_autonomy.py modules/wheeled_navigation/artifacts/my_ros_exploration
```

输出 `evaluation.json` 与 `autonomy_evidence.png`。Code-as-Room 的覆盖率按 9 × 6 米房间范围内、同一张 5 厘米评价栅格中已被观测的 XY 单元计算。室外单元不计入分子。它不是“可通行地面覆盖率”，也不是“三维表面重建完整率”；家具投影也属于这 54 平方米分母。

评价器可以读取场景碰撞体 AABB 和仿真位姿，独立复查执行轨迹上的完整身体是否重叠。该检查只用于事后评价，不参与目标选择；矩形/AABB 检查也不能替代之后真实刚体与关节动力学验证。定位误差仅做一次初始坐标对齐，不通过拟合整条轨迹掩盖漂移。开启回环参数本身不作为已经发生回环的证据。

## 已完成的实际验证与当前限制

- `artifacts/autonomy_native_dynamic/`：第一阶段实际完成了跨隔墙的探索并返回，运行中加入了新障碍；保存了扫描、目标、在线重规划和实际运动。
- `artifacts/autonomy_native_dynamic_v2/`：更新后第一阶段的回归运行，107.5 秒仿真、19.42 米实际位移、105 次在线重规划，新增障碍后继续运行并返回。它因新增观察停滞而结束，没有充分进入另一侧，不能将“返回成功”解释为“全部区域探索完成”。
- `artifacts/autonomy_ros_code_room_v2/`：真实 SLAM Toolbox / Nav2 的圆形避障对照实验。轨迹在中部通道反复移动，这正是需要修正的现象。
- `artifacts/autonomy_ros_polygon_v5/`：完整多边形与 Hybrid A* 实际进入右侧区域后返回。独立检查未发现完整轮廓与场景 AABB 重叠；房间内已观测 XY 投影为 22.73 平方米，占 54 平方米的 42.09%。这个实验还不能证明已经充分探索房间。
- `artifacts/autonomy_ros_polygon_v12/`：当前 DWB、持续完整代价地图和真实回波覆盖静态边缘的配置，实际完成了 4 个目标（含返回），完整轮廓/AABB 重叠为 0。已观测房间内投影为 19.06 平方米，35.30%；它修复了部分停滞，但覆盖面积没有超过 v5。不能据此声称覆盖率优化已经完成。

后续带编号的实验属于调试记录，应结合各自 `status.json`、目标结果和评价报告判断，不能把最近目录自动当成成功结果。当前仍在优化窄通道的目标朝向和观察位置。

缺少当前机器人的真实 checkpoint，下一阶段需要把 `/cmd_vel` 接入相应训练策略，再重新检验姿态变化、足端与轮胎接触、传感器安装、速度限幅和导航稳定性。目前不会把简化位姿积分描述成双轮足动力学仿真成功。
