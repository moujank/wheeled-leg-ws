# wheeled_mujoco.ros

该目录预留给 MuJoCo 集成相关的 ROS 接口。

## 当前文件

- `interfaces.py`：目前只有轻量级协议和节点规格定义

## 当前状态

- 这里还没有导入 ROS2 运行时依赖
- 还没有实现 publisher、subscriber、launch 文件或消息包

## 未来预期角色

- 命令输入节点
- MuJoCo 状态发布节点
- 策略运行桥接节点
- 用于切换或混合任务策略的任务监督节点

这一层应保持在进程边界位置；仿真内部控制逻辑应放在 `sim2sim/` 或更底层运行代码中。
