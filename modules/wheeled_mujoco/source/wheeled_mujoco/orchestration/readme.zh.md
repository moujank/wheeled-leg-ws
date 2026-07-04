# wheeled_mujoco.orchestration

该目录预留任务切换和任务耦合逻辑。

当前状态：

- `task_switcher.py` 提供了一个轻量的进程内注册表包装器和任务切换描述。
- 它目前不会混合策略，也不会直接修改 MuJoCo 控制。
- 后续 locomotion、grasp、obstacle traversal 和 recovery 之间的耦合，应先在这里实现，再暴露给 ROS 节点。

这层边界刻意放在单个 sim2sim runner 之上。runner 负责执行某一个任务，而 orchestration 层负责决定当前应该激活哪个任务。
