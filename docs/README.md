# 工作区文档索引

[中文入口](../README.zh.md) · [English overview](../README.md) · [模块导航](../modules/README.md)

## 工作区地图

```text
wheeled-leg-ws/
├── README.md / README.zh.md        工作区介绍与常用工作流
├── docs/                          跨模块流程、待办、文档规范与提交历史
├── modules/
│   ├── README.md                   九个模块的导航
│   ├── wheeled_train/
│   │   ├── README.md / README.zh.md 训练模块概述
│   │   └── wheeled_leg/
│   │       ├── README*             工程安装、任务清单、运行与日志约定
│   │       ├── source/             Isaac Lab 扩展与任务代码
│   │       ├── scripts/            训练、回放与策略维护入口
│   │       ├── tools/              工程分析工具
│   │       ├── outputs/            Hydra / Isaac Lab 输出
│   │       └── logs/rsl_rl/
│   │           ├── POLICY_INDEX.md 策略说明导航与选择流程
│   │           └── <experiment>/best/notes.md
│   ├── wheeled_mujoco/
│   │   ├── README*                 模块概述
│   │   ├── docs/                   架构、一致性、交接、资产查询与历史
│   │   ├── minimal_sim2sim/README*  最小闭环说明
│   │   ├── scripts/README*          命令行入口说明
│   │   ├── source/README*           源码导航
│   │   ├── source/wheeled_mujoco/   Python 包及各子包 README
│   │   └── asset/                  MJCF、URDF、mesh 与索引工具
│   └── wheeled_*/README*           其余七个预留模块的职责说明
├── assets/                         Isaac 侧导出资产与机器人资源
├── sync_to_github.sh               工作区与模块 subtree 同步
└── update_git_history.sh           生成 docs/GIT_HISTORY.md
```

`assets/` 包含 `usd/`、`usd_V/`、`usd_arm/`、`usd_leg/`、`wheeled_arm/` 和 `wheeled_leg/`。训练分析工具已统一迁入 `modules/wheeled_train/wheeled_leg/tools/`，包含力矩与奖励曲线工具，详见 [工具说明](../modules/wheeled_train/wheeled_leg/tools/README.md)。根目录的 TXT 工作记录、模块临时 TXT、编辑器配置等位置可从工作区直接查看；它们不属于 Markdown 导航。

## 按目的阅读

| 目的 | 入口 |
| --- | --- |
| 了解整个工程 | [工作区 README](../README.zh.md) → [模块导航](../modules/README.md) |
| 安装、训练、回放与查任务 | [训练工程 README](../modules/wheeled_train/wheeled_leg/README.zh.md) |
| 选择策略与确认 checkpoint 来源 | [策略索引](../modules/wheeled_train/wheeled_leg/logs/rsl_rl/POLICY_INDEX.md) → 各实验 `best/notes.md` |
| 使用 MuJoCo、查运行入口 | [MuJoCo README](../modules/wheeled_mujoco/README.zh.md) → [模块文档索引](../modules/wheeled_mujoco/docs/README.md) |
| 规划 Isaac → MuJoCo 迁移 | [SIM2SIM_DYNAMIC_FLOW.md](SIM2SIM_DYNAMIC_FLOW.md) |
| 查当前待办 | [TODO.md](TODO.md) |
| 查整理规则、迁移对应表及全部 MD 路径 | [DOCUMENTATION_STRUCTURE.md](DOCUMENTATION_STRUCTURE.md) |
| 查自动生成的提交历史 | [GIT_HISTORY.md](GIT_HISTORY.md) |

## 维护方式

目录入口统一使用 `README.md` / `README.zh.md`，中英文保留语言配对。同一主题的详细说明在一个位置维护，上层 README 提供链接。跨模块文档放在根 `docs/`，模块专题放在模块 `docs/`，策略说明随实验目录保存。文档链接使用相对路径；运行命令注明工作目录。历史结论须标注日期和对应训练产物。
