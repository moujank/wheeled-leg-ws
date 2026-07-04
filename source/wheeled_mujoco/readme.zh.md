# wheeled_mujoco source package

该包包含 MuJoCo 侧的任务配置、sim2sim 运行器、ROS 接口以及后续任务编排代码。

这个包刻意保持对 Isaac Lab 独立。它会镜像 `modules/wheeled_train` 中的任务命名和策略日志布局，同时使用 `modules/wheeled_mujoco/asset` 中的 MuJoCo 资产。

导入顺序：

```text
scripts
  -> wheeled_mujoco.sim2sim
  -> wheeled_mujoco.tasks
  -> wheeled_mujoco.paths
```

当前 ROS 和任务切换代码都只是接口骨架。在对应运行时节点真正实现并完成测试之前，不应该用它们去改变已经验证过的 `base_locked` 启动行为。
