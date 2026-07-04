# wheeled_mujoco.sim2sim

该目录包含 sim2sim 运行适配器。

当前实现：

- `base_locked.py` 提供 source 层 API 和 argparse 构建器，对接现有的 base-locked MuJoCo rollout。
- 它通过导入 `minimal_sim2sim/run_base_locked.py` 中已经验证过的实现来复用行为，而不是复制一份，因此默认参数与当前调试基线保持一致。

后续任务专用 runner 只有在对应训练任务的观测、动作、执行器顺序和策略导出都核对清楚之后，才应该继续往这里添加。
