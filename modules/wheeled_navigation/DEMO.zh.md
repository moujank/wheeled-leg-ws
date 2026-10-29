# 本机 Isaac Sim 激光建图与目标点规划演示

这套新增代码不修改已有训练配置、策略权重和 MuJoCo runner。场景由 Isaac Sim 的 USD/PhysX API 创建，不需要 Blender。

## 当前演示的范围

- 加载仓库自己的 `wheeled_leg_v2.usd`，在两间相连的房间中挂载虚拟 360° PhysX 激光雷达。
- 使用真实运行的仿真扫描进行 log-odds 占据栅格建图。地图和规划代码不读取房间的墙体坐标。
- 在已观测的自由空间上进行 A* 规划，将机器人半径与网格误差计入障碍膨胀；未知区域不可通行，拒绝对角穿墙。
- 验证三组跨房间目标点、拒绝障碍内/场景外目标，并用独立场景几何检查规划和运动轨迹净空。
- 用简化 unicycle 驱动沿路线移动到目标，输出地图、路径、扫描记录、统计、动画、报告和可重新打开的 USD 场景。

**这是仿真真值里程计条件下的传感器建图与规划基线。它没有执行已有 RL 策略，没有运行 SLAM Toolbox/Nav2，也没有验证回环优化、传感器里程计、自主探索或真实动力学路线执行。**

预设采集路线只用于采集扫描，不能称为自主探索；模型在新增场景中禁用动力学和自身碰撞，不能据此评价机器人平衡性能。所有修改都是对新建场景的覆盖，源 USD 资产没有被改写。

## 环境

本机已找到并成功启动 `D:\isaacsim`，版本 4.5.0，内置 Python 3.10.15。GPU 为 RTX 4060 Laptop 8GB。运行时探测确认 NumPy、SciPy、Pillow、Matplotlib 和 PhysX 雷达可用。没有在已检查路径和 Python 环境中发现独立安装的 Isaac Lab；仓库中的任务源码不能替代 Isaac Lab 框架安装。

Isaac Sim 自带 ROS 2 Humble bridge 的运行库，但本机没有检测到可直接启动的 SLAM Toolbox/Nav2 节点和完整 ROS CLI 环境。原生演示无需新装依赖、无需模型权重。

## 运行

在仓库根目录的 PowerShell 中执行：

```powershell
# 无界面执行完整验收，结果默认写到 modules/wheeled_navigation/artifacts/isaac_demo
& .\modules\wheeled_sim_bridge\run_navigation_demo.ps1

# 显示 Isaac Sim 窗口，完成验收后保留场景
& .\modules\wheeled_sim_bridge\run_navigation_demo.ps1 -Gui -Hold

# 指定另一个目标点，坐标单位是米
& .\modules\wheeled_sim_bridge\run_navigation_demo.ps1 -Goal 5.5,-3.5
```

如果系统阻止运行本地脚本，可直接运行 Isaac Python，无需改变执行策略：

```powershell
& 'D:\isaacsim\python.bat' '.\modules\wheeled_sim_bridge\scripts\run_isaac_mapping.py' --gui --hold
```

重复执行会覆盖同一输出目录。需要保留一次结果时，用 `-Output` 或 `--output` 指定新的输出路径。脚本不安装或改写系统环境。

## 输出

- `report.html`：包含地图、扫描回放、场景截图和指标的单文件报告。
- `isaac_scene.png`：实际 Isaac Sim 视口截图。
- `mapping_and_paths.png`、`mapping_progress.gif`：地图、路线与采集过程。
- `map.pgm`、`map.yaml`：ROS map_server 可读取的地图。
- `grid.npz`：保留未知区域和 log-odds 的规划地图。
- `scans.npz`：每帧雷达距离、角度、理想里程计位姿和采样时间；可重放审查。
- `metrics.json`：条件声明、三组路径、独立净空检查、终点误差。
- `trajectory.json`：简化驱动的位姿和速度命令。
- `navigation_scene.usda`：带自己机器人、障碍、雷达和路线的场景；依赖仓库原机器人 USD，不是完全独立资产包。

报告中的轨迹净空、激光命中位置误差使用场景几何做独立验收；这些几何信息不会交给地图构建器或路径规划器。

## 不启动仿真，重新规划目标

```powershell
& 'D:\isaacsim\python.bat' '.\modules\wheeled_navigation\scripts\plan_goal.py' `
  --map '.\modules\wheeled_navigation\artifacts\isaac_demo' `
  --start -5.5 -3.5 --goal 5.5 3.5 --output '.\new_goal.json'
```

## ROS 2 接口与后续现成 SLAM/Nav2

`--ros2` 开启 `/clock`、`/scan`、理想 `/odom`、TF、原生地图和 `/native_demo/plan` 发布；订阅 `/cmd_vel` 并提供带超时的命令读取接口。需要使用 Isaac Sim 的启动环境才能加载其 bridge。

本机已实际通过 `--ros2` 完整运行，并由诊断订阅回读了 `/scan`、`/odom` 和 `/map`。运行时只在本进程设置 bundled Humble DLL 路径，不修改系统环境。实际话题与消息类型见输出目录的 `ros2_topics.json`。

`--ros2 --external-slam` 时原生桥不会发布 `/map` 或 `map → odom`，由外部 SLAM Toolbox 独占这两项。TF 为：

```text
map → odom → base_footprint → base_link → lidar_link
```

当前 ROS 桥只提供数据和命令接口；原生演示的路径执行不读取 Nav2 命令。下一步接管时必须明确接线和命令仲裁，不能声称已经验证 Nav2 控制机器人。

在另一个已经安装 ROS 2 Humble、`slam_toolbox` 和 `nav2_bringup` 的 Linux 环境中，可使用：

```bash
source /opt/ros/humble/setup.bash
ros2 launch /absolute/path/to/modules/wheeled_navigation/launch/slam_nav.launch.py
```

提供的 launch 从所安装 Nav2 的完整默认配置读取，再覆盖机器人的足迹、速度、加速度、扫描与坐标系。它是后续接入入口，尚不能替代在真实 ROS 环境中的联调验收。两端需相同 ROS_DOMAIN_ID，DDS 网络可达；Windows/WSL/远程 Linux 的网络要单独验证。

## 验证

```powershell
& 'D:\isaacsim\python.bat' -m unittest discover -s '.\modules\wheeled_navigation\tests' -v
& 'D:\isaacsim\python.bat' '.\modules\wheeled_sim_bridge\scripts\run_isaac_mapping.py' --smoke
```

需要重做真实 SLAM 验证时，应在已有传感器/时间/TF接口上运行 SLAM Toolbox，记录回环前后的地图和位姿误差；随后将理想里程计替换为传感器估计，最后再单独验证 RL 运动策略接管。
