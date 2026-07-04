# wheeled_mujoco.tasks

该目录用 MuJoCo 侧的资产和环境配置去镜像 Isaac 训练任务名称。

当前范围：

- 注册任务元数据、资产路径、默认导出策略路径和命令范围。
- 让 `base_locked_locomotion` 通过已经验证过的 base-locked sim2sim 适配器可运行。
- 为 arm/grasp/obstacle/recovery 任务预留明确的 MuJoCo 资产和 log 映射，但不假装它们的完整观测/动作部署已经实现。

命名跟随训练侧目录：

```text
wheeled_train/.../wheeled_leg/*_env_cfg.py
```

真正的 runner 分发发生在 `wheeled_mujoco.sim2sim`。
