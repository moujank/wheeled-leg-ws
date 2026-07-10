# Wheeled MuJoCo Sim2Sim 架构与功能分工

更新时间：`2026-07-10`

本文档用于约束 `modules/wheeled_mujoco` 后续的 `sim2sim` 代码组织方式。

目标不是描述“现在仓库里所有历史代码都长什么样”，而是给出一套你后续继续扩展时应遵守的结构原则，尤其是：

1. `scripts/` 只做接入口
2. `source/wheeled_mujoco/tasks/` 只做任务配置和任务元数据
3. `source/wheeled_mujoco/sim2sim/` 只做运行时实现
4. `asset/` 只做资产与索引基线
5. `minimal_sim2sim/` 作为最小闭环实验区，不直接承担长期产品化接口职责

---

## 1. 总体目标

后续你想要的不是“每个任务各写一个散乱脚本”，而是：

1. 用户从 `scripts/` 进入
2. 脚本根据任务名选择一个 task
3. task 提供该任务的：
   - 模型路径
   - 默认 policy 路径
   - command 范围
   - observation/action 语义
   - reset / startup / handoff 参数
4. `source/wheeled_mujoco/sim2sim/` 里对应的 runner 负责真正执行 MuJoCo 闭环

也就是说，后续结构应该做到：

- `scripts` 负责“入口和参数解释”
- `tasks` 负责“任务声明”
- `sim2sim` 负责“任务运行”
- `asset` 负责“机器人资产语义”

---

## 2. 推荐目录职责

建议以当前目录为基础，按下面职责使用。

### 2.1 `asset/`

路径：

- `modules/wheeled_mujoco/asset/mjcf/`
- `modules/wheeled_mujoco/asset/idx/`
- `modules/wheeled_mujoco/asset/wheeled_leg/`
- `modules/wheeled_mujoco/asset/wheeled_arm/`

职责：

1. 保存 MJCF/XML、mesh、urdf 等资产
2. 保存资产索引检查脚本
3. 保存资产语义梳理文档

这里不应该放：

1. 任务逻辑
2. policy loader
3. sim2sim 主循环

也就是说，`asset/` 只回答一个问题：

`机器人在 MuJoCo 里到底是什么。`

---

### 2.2 `minimal_sim2sim/`

路径：

- `modules/wheeled_mujoco/minimal_sim2sim/`

职责：

1. 作为最小闭环实验区
2. 做 observation / action / reset / handoff 的快速验证
3. 做原型验证和调试

这里不应该承担的长期职责：

1. 不应该成为最终对外统一入口
2. 不应该承载多个任务混杂的生产接口
3. 不应该变成长期唯一的配置来源

建议定位：

`minimal_sim2sim/` 是“实验台”，不是“最终对外接口层”。

也就是说：

- 当你验证一个新任务的新 observation / action 语义时，可以先在这里做最小闭环
- 当语义稳定后，再整理进 `source/wheeled_mujoco/sim2sim/` 和 `scripts/`

---

### 2.3 `scripts/`

路径：

- `modules/wheeled_mujoco/scripts/`

职责：

1. 作为用户启动入口
2. 解析命令行参数
3. 根据 task 名选择任务
4. 调用 `source/wheeled_mujoco/...` 中的运行实现

这里应该只放：

1. 很薄的一层 CLI 入口
2. 参数默认值覆盖
3. 调用 task registry 和 runner

这里不应该放：

1. 大量 observation 细节
2. 大量 MuJoCo 主循环代码
3. 资产索引逻辑
4. 任务特定硬编码

换句话说，`scripts/` 应该回答：

`用户怎么启动某个 sim2sim 任务。`

不应该回答：

`这个任务的 observation 怎么算。`

---

### 2.4 `source/wheeled_mujoco/tasks/`

路径：

- `modules/wheeled_mujoco/source/wheeled_mujoco/tasks/`

职责：

1. 保存任务元数据
2. 保存任务默认路径
3. 保存任务 command 范围
4. 保存任务使用哪套 sim2sim runner
5. 保存任务级 observation/action/reset/startup 配置

这层应该尽量像“任务注册表”，而不是执行器。

每个任务文件应该描述的内容包括：

1. `task_name`
2. `model_path`
3. `default_policy_path`
4. `backend` 默认值
5. `command` 默认值和范围
6. `joint order`
7. `action scale`
8. `default joint pos`
9. `reset pose`
10. `startup / handoff` 默认参数
11. `runner_type` 或 `sim2sim implementation key`

这层不应该直接承担：

1. 打开 viewer
2. 加载 MuJoCo model
3. 主循环 step
4. 推理执行

也就是说，`tasks/` 只回答：

`这个任务需要什么配置和默认值。`

---

### 2.5 `source/wheeled_mujoco/sim2sim/`

路径：

- `modules/wheeled_mujoco/source/wheeled_mujoco/sim2sim/`

职责：

1. 实现 MuJoCo sim2sim runner
2. 实现 policy loader
3. 实现 observation builder
4. 实现 action adapter
5. 实现 reset / startup / handoff
6. 实现 rollout 循环
7. 实现 viewer / logging / diagnostics

这层是“运行时层”。

它应该吃进去的是：

1. task config
2. CLI 参数覆盖
3. policy path / backend

它吐出来的是：

1. 运行中的 MuJoCo 闭环
2. viewer
3. 日志
4. 诊断结果

也就是说，`sim2sim/` 负责回答：

`给定一个任务配置，MuJoCo 里到底怎么跑起来。`

---

## 3. 推荐的调用链

后续推荐的调用链如下：

1. 用户执行 `scripts/run_sim2sim.py --task base_locked_locomotion ...`
2. `scripts/run_sim2sim.py` 解析 CLI
3. 从 `source/wheeled_mujoco/tasks/` 里查到任务配置
4. 从配置里得到：
   - model path
   - default policy path
   - runner type
   - 命令和 handoff 默认值
5. `source/wheeled_mujoco/sim2sim/` 中的 runner 接收配置并运行

这条链里每一层只干自己该干的事。

---

## 4. 推荐的代码分工

如果以后要把当前最小闭环整理成长期结构，推荐拆成下面几个部件。

### 4.1 入口脚本层

建议新增或统一成：

- `modules/wheeled_mujoco/scripts/run_sim2sim.py`

职责：

1. CLI 参数定义
2. task 选择
3. backend 选择
4. 将 CLI 覆盖写回 task config
5. 启动 runner

这里不做 observation/action 细节。

---

### 4.2 任务配置层

建议在 `source/wheeled_mujoco/tasks/` 中，每个任务文件只暴露一个统一配置对象。

例如：

- `base_locked_locomotion_env_cfg.py`
- `arm_free_locomotion_env_cfg.py`

建议每个任务配置最终都能提供这些信息：

1. 任务名
2. 模型路径
3. 默认 policy 路径
4. 默认 backend
5. command 默认值
6. observation 语义描述
7. action 语义描述
8. reset 语义
9. startup/handoff 默认值
10. runner 类名或 key

---

### 4.3 sim2sim 公共运行时层

建议在 `source/wheeled_mujoco/sim2sim/` 中逐步抽出公共模块：

1. `policy.py`
   - torch / onnx loader
2. `indices.py`
   - name -> qpos/qvel/ctrl/sensor 索引
3. `observation.py`
   - observation builder
4. `action.py`
   - action adapter
5. `reset.py`
   - reset / startup / handoff
6. `runner.py`
   - 主循环
7. `viewer.py`
   - viewer / realtime sync
8. `logging.py`
   - JSONL / console diagnostics

这样任务特定实现只需要：

1. 选择配置
2. 少量 override

而不是把整套主循环复制一遍。

---

## 5. 推荐的配置分层

后续建议把参数分成 4 层。

### 5.1 资产层参数

来源：

- `asset/mjcf/*.xml`
- `asset/idx/*.py`

包括：

1. joint names
2. actuator names
3. axis
4. qpos/qvel 地址
5. sensor 地址

这层尽量不在运行时频繁改。

---

### 5.2 任务层参数

来源：

- `source/wheeled_mujoco/tasks/*.py`

包括：

1. policy 路径
2. action scale
3. command 默认值
4. reset pose
5. startup 参数
6. 任务级 command range

这层是“默认语义层”。

---

### 5.3 运行时层参数

来源：

- `source/wheeled_mujoco/sim2sim/*.py`

包括：

1. sim dt
2. control dt
3. viewer 开关
4. realtime sync
5. logging 频率
6. handoff 逻辑

这层是“运行机制层”。

---

### 5.4 CLI 覆盖层

来源：

- `scripts/*.py`

包括：

1. `--task`
2. `--policy`
3. `--backend`
4. `--cmd-vx`
5. `--cmd-vy`
6. `--cmd-wz`
7. `--viewer`
8. `--handoff-seconds`
9. `--duration`

这层只做临时覆盖，不改默认配置文件。

---

## 6. 推荐的任务注册方式

建议 `source/wheeled_mujoco/tasks/common.py` 继续承担“任务注册表”职责。

建议注册表至少维护：

1. `task_name`
2. `task_cfg`
3. `sim2sim_runner_key`
4. `default_model_path`
5. `default_policy_path`
6. `default_backend`

这样 `scripts/run_sim2sim.py` 只需要：

1. 根据 task_name 查注册表
2. 获取配置
3. 调用 runner

而不需要知道某个任务文件的内部细节。

---

## 7. 当前最小闭环在未来结构中的定位

当前文件：

- `modules/wheeled_mujoco/minimal_sim2sim/minimal_sim2sim.py`

建议定位如下：

1. 它是 `base_locked` 的最小闭环实验原型
2. 它是 observation / action / reset / handoff 的验证台
3. 它不是最终统一入口

后续当你把这套最小闭环验证稳定后，建议：

1. 把公共逻辑迁移到 `source/wheeled_mujoco/sim2sim/`
2. 把 CLI 包装迁移到 `scripts/`
3. 把任务默认值收敛到 `source/wheeled_mujoco/tasks/`
4. 保留 `minimal_sim2sim/` 作为实验和 debug 工具

也就是说：

- `minimal_sim2sim.py` 负责“先跑通”
- `source + scripts` 负责“长期维护与扩展”

---

## 8. 新增一个 sim2sim 任务时应修改哪些层

以后你新增任务时，建议严格按下面顺序动代码。

1. 在 `tasks/` 新增任务配置文件
   - 定义默认 model/policy/command/reset/startup

2. 在 `tasks/common.py` 注册任务
   - 给这个 task 一个统一名字

3. 如果 observation/action 语义与现有 runner 兼容
   - 直接复用已有 `sim2sim runner`

4. 如果 observation/action 语义不兼容
   - 在 `sim2sim/` 新增新的 runner 实现

5. 在 `scripts/run_sim2sim.py` 中暴露该 task
   - 通过 `--task` 选择

6. 必要时在 `minimal_sim2sim/` 先做最小闭环实验

---

## 9. 不建议的做法

后续实现时，建议避免下面这些结构问题。

1. 不要把所有任务都写成独立大脚本，互相复制主循环
2. 不要把 observation/action 细节散落在 `scripts/` 里
3. 不要让 `tasks/` 文件直接执行 MuJoCo 主循环
4. 不要把资产索引、运行时逻辑、任务默认值混在一个文件里
5. 不要把 `minimal_sim2sim.py` 直接继续膨胀成最终总入口

---

## 10. 推荐的近期落地顺序

结合你当前状态，建议按下面顺序逐步落地。

1. 先继续把 `minimal_sim2sim.py` 调到最小稳定可用
   - startup stand
   - handoff
   - wheel sign

2. 再把其中公共逻辑抽到 `source/wheeled_mujoco/sim2sim/`
   - policy loader
   - observation builder
   - action adapter
   - reset/startup

3. 再补一个统一入口脚本到 `scripts/`
   - 推荐名字：`run_sim2sim.py`

4. 再把任务默认配置统一收敛到 `source/wheeled_mujoco/tasks/`

5. 最后再考虑 ROS 或更高层 orchestration 的接入

---

## 11. 最终希望达到的使用方式

最终理想使用方式应类似：

```bash
python modules/wheeled_mujoco/scripts/run_sim2sim.py \
  --task base_locked_locomotion \
  --backend torch \
  --policy modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.pt \
  --cmd-vx 0.2 \
  --viewer
```

也就是说：

1. 用户只关心 task 和运行参数
2. 任务路径、默认配置、runner 选择都由 `source/` 负责
3. 主循环实现藏在 `sim2sim/`

这样后续新增任务和维护都不会失控。

