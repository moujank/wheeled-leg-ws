# wheeled_mujoco.sim2sim

该目录包含 MuJoCo sim-to-sim 执行的 runner 适配层。

## 当前文件

- `base_locked.py`：base-locked rollout 的参数解析构建器和执行适配器
- `__init__.py`：供脚本侧导入的公共导出

## 当前设计

`base_locked.py` 不会复制完整 rollout 实现，而是导入并转发到已经验证过的基线：

```text
modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py
```

这样可以把行为基线保持在一个地方，同时又允许更高层脚本和任务分发器通过包级 API 使用它。

只有在观测定义、动作顺序、资产连接和导出策略都和训练侧核对完成之后，才应在这里增加新的任务 runner。
