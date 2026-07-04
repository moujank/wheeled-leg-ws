# wheeled_mujoco.ros

该目录预留 MuJoCo 仿真控制在 ROS 侧的接口。

当前状态：

- 这里还没有引入 ROS 运行时依赖。
- `interfaces.py` 定义了后续命令、状态、策略和任务控制节点需要使用的轻量协议与节点规格。
- 后续真实的 ROS2 package、launch 文件和消息绑定应在不改变 sim2sim 核心 API 的前提下继续补充。

后续预期的节点角色：

- 命令输入节点：接收机体系速度命令或任务命令。
- MuJoCo 状态节点：发布仿真中的机器人状态。
- 策略运行节点：将状态和命令转换为动作。
- 任务监管节点：在 locomotion、recovery、grasp 和 obstacle 任务之间进行切换或混合。
