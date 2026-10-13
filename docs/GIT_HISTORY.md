# Git History

Generated at: 2026-10-13 12:08:58 +0800

This file is generated from Git history. Run `./update_git_history.sh` to refresh it.

## fe42a45 - sync: 2026-07-10 14:15:34 +0800
Author: moujank <moujank588@gmail.com>
Date: 2026-07-10 14:15:34 +0800

 README.md                                          | 104 ++-
 README.zh.md                                       |  94 +++
 SIM2SIM_DYNAMIC_FLOW.md                            | 514 ++++++++++++
 modules/wheeled_deploy/README.md                   |  17 +-
 modules/wheeled_deploy/README.zh.md                |  17 +
 modules/wheeled_description/README.md              |  17 +-
 modules/wheeled_description/README.zh.md           |  17 +
 modules/wheeled_firmware/README.md                 |  16 +-
 modules/wheeled_firmware/README.zh.md              |  16 +
 modules/wheeled_hardware/README.md                 |  16 +-
 modules/wheeled_hardware/README.zh.md              |  16 +
 .../wheeled_mujoco/MUJOCO_CONSISTENCY_CHECKLIST.md | 195 +++++
 modules/wheeled_mujoco/MUJOCO_DEBUG_HANDOFF.md     | 380 ++++++++-
 modules/wheeled_mujoco/SIM2SIM_ARCHITECTURE.md     | 521 ++++++++++++
 modules/wheeled_mujoco/asset/idx/idx.sh            |   5 +
 modules/wheeled_mujoco/asset/idx/idx_actuator.py   |  12 +
 modules/wheeled_mujoco/asset/idx/idx_axis_check.py | 193 +++++
 .../asset/idx/idx_bode_site_sensor.py              |  16 +
 modules/wheeled_mujoco/asset/idx/idx_body_id.py    |   4 +
 modules/wheeled_mujoco/asset/idx/idx_homepose.py   |  18 +
 .../wheeled_mujoco/asset/idx/idx_joint_array.py    |   7 +
 modules/wheeled_mujoco/asset/idx/idx_qpos_qvel.py  |  20 +
 modules/wheeled_mujoco/asset/mjcf/index_list.md    | 392 +++++++++
 modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml  |  24 +-
 modules/wheeled_mujoco/minimal_sim2sim/README.md   | 156 +---
 .../wheeled_mujoco/minimal_sim2sim/README.zh.md    | 178 +---
 .../__pycache__/run_base_locked.cpython-310.pyc    | Bin 38061 -> 50511 bytes
 .../__pycache__/run_base_locked.cpython-313.pyc    | Bin 75995 -> 102247 bytes
 .../minimal_sim2sim/minimal_sim2sim.py             | 919 +++++++++++++++++++++
 .../minimal_sim2sim/run_base_locked.py             | 585 +++++++++++--
 modules/wheeled_mujoco/readme.md                   |  65 +-
 modules/wheeled_mujoco/readme.zh.md                |  65 +-
 .../__pycache__/run_task_sim2sim.cpython-313.pyc   | Bin 0 -> 5651 bytes
 modules/wheeled_mujoco/scripts/readme.md           |  84 +-
 modules/wheeled_mujoco/scripts/readme.zh.md        |  84 +-
 modules/wheeled_mujoco/source/readme.md            |  21 +-
 modules/wheeled_mujoco/source/readme.zh.md         |  21 +-
 .../__pycache__/__init__.cpython-313.pyc           | Bin 0 -> 367 bytes
 .../__pycache__/paths.cpython-313.pyc              | Bin 0 -> 1287 bytes
 .../source/wheeled_mujoco/orchestration/readme.md  |  23 +-
 .../wheeled_mujoco/orchestration/readme.zh.md      |  23 +-
 .../wheeled_mujoco/source/wheeled_mujoco/readme.md |  21 +-
 .../source/wheeled_mujoco/readme.zh.md             |  21 +-
 .../source/wheeled_mujoco/ros/readme.md            |  25 +-
 .../source/wheeled_mujoco/ros/readme.zh.md         |  25 +-
 .../sim2sim/__pycache__/__init__.cpython-313.pyc   | Bin 0 -> 394 bytes
 .../__pycache__/base_locked.cpython-310.pyc        | Bin 10223 -> 13418 bytes
 .../__pycache__/base_locked.cpython-313.pyc        | Bin 0 -> 20001 bytes
 .../source/wheeled_mujoco/sim2sim/base_locked.py   | 150 +++-
 .../source/wheeled_mujoco/sim2sim/readme.md        |  20 +-
 .../source/wheeled_mujoco/sim2sim/readme.zh.md     |  20 +-
 .../tasks/__pycache__/__init__.cpython-313.pyc     | Bin 0 -> 1558 bytes
 .../arm_free_locomotion_env_cfg.cpython-313.pyc    | Bin 0 -> 871 bytes
 .../arm_free_obstacle_env_cfg.cpython-313.pyc      | Bin 0 -> 845 bytes
 .../arm_free_recovery_env_cfg.cpython-313.pyc      | Bin 0 -> 841 bytes
 .../base_locked_locomotion_env_cfg.cpython-310.pyc | Bin 1186 -> 1167 bytes
 .../base_locked_locomotion_env_cfg.cpython-313.pyc | Bin 0 -> 1263 bytes
 .../cartesian_grasp_env_cfg.cpython-313.pyc        | Bin 0 -> 845 bytes
 .../tasks/__pycache__/common.cpython-313.pyc       | Bin 0 -> 3584 bytes
 .../grasp_pose_locomotion_env_cfg.cpython-313.pyc  | Bin 0 -> 864 bytes
 .../loaded_grasp_obstacle_env_cfg.cpython-313.pyc  | Bin 0 -> 878 bytes
 .../tasks/base_locked_locomotion_env_cfg.py        |   6 +-
 .../source/wheeled_mujoco/tasks/readme.md          |  26 +-
 .../source/wheeled_mujoco/tasks/readme.zh.md       |  26 +-
 modules/wheeled_navigation/README.md               |  16 +-
 modules/wheeled_navigation/README.zh.md            |  16 +
 modules/wheeled_sim_bridge/README.md               |  16 +-
 modules/wheeled_sim_bridge/README.zh.md            |  16 +
 modules/wheeled_teleop/README.md                   |  16 +-
 modules/wheeled_teleop/README.zh.md                |  16 +
 modules/wheeled_train/README.md                    | 410 +--------
 modules/wheeled_train/README.zh.md                 |  57 ++
 modules/wheeled_train/wheeled_leg/README.md        | 202 ++---
 modules/wheeled_train/wheeled_leg/README.zh.md     | 139 ++++
 .../wheeled_base_locked/best/exported/policy.onnx  | Bin 9509 -> 116670 bytes
 .../wheeled_base_locked/best/exported/policy.pt    |   4 +-
 .../logs/rsl_rl/wheeled_base_locked/best/model.pt  |   2 +-
 .../logs/rsl_rl/wheeled_base_locked/best/notes.md  |  16 +-
 .../best/exported/policy.onnx                      | Bin 9509 -> 118230 bytes
 .../best/exported/policy.pt                        |   4 +-
 .../rsl_rl/wheeled_base_locked_hard/best/model.pt  |   2 +-
 .../rsl_rl/wheeled_base_locked_hard/best/notes.md  |  16 +-
 .../wheeled_leg/scripts/rsl_rl/play.py             | 135 ++-
 .../wheeled_leg/scripts/rsl_rl/train.py            | 113 +++
 .../tasks/manager_based/wheeled_leg/__init__.py    |  13 +-
 .../wheeled_leg/agents/rsl_rl_ppo_cfg.py           |  55 +-
 .../manager_based/wheeled_leg/asset/wheeled.py     |   8 +-
 .../wheeled_leg/base_locked_locomotion_env_cfg.py  | 473 +++--------
 .../base_locked_locomotion_hard_env_cfg.py         | 403 +++++++++
 .../wheeled_leg/grasp_pose_locomotion_env_cfg.py   | 110 ++-
 .../tasks/manager_based/wheeled_leg/mdp/rewards.py |  11 +
 .../manager_based/wheeled_leg/terrains/__init__.py |  10 +
 .../wheeled_leg/terrains/base_locked.py            |  85 +-
 .../wheeled_leg/terrains/single_terrains.py        | 273 ++++++
 .../manager_based/wheeled_leg/terrains/terrains.py | 116 ++-
 .../wheeled_leg/wheeled_glab_env_cfg.py            | 451 ++++++++++
 .../wheeled_leg/tools/plot_joint_torque.py         | 113 +++
 tools/plot_joint_torque.py                         | 113 +++
 tools/plot_mean_reward.py                          | 297 +++++++
 99 files changed, 7373 insertions(+), 1549 deletions(-)

## 5ec035c - sync: validate sync_to_github.sh rerun
Author: moujank <moujank588@gmail.com>
Date: 2026-07-04 15:50:45 +0800

 .../minimal_sim2sim/run_base_locked.py             | 42 ++++++++++++++++------
 1 file changed, 31 insertions(+), 11 deletions(-)

## 619c032 - sync: validate sync_to_github.sh
Author: moujank <moujank588@gmail.com>
Date: 2026-07-04 15:43:11 +0800

 .vscode/browse.vc.db     | Bin 737280 -> 0 bytes
 .vscode/browse.vc.db-shm | Bin 32768 -> 0 bytes
 .vscode/browse.vc.db-wal |   0
 sync_to_github.sh        | 116 ++++++++++++++++++++++++++++++++++++++++++++---
 4 files changed, 109 insertions(+), 7 deletions(-)

## 3448b8d - sync: uploadable main workspace snapshot
Author: moujank <moujank588@gmail.com>
Date: 2026-07-04 15:03:40 +0800

 .gitignore                                         |    2 +
 GIT_HISTORY.md                                     |   39 +-
 MUJOCO_LOG.TXT                                     |    3 +
 README.md                                          |    7 +
 assets/usd/config.yaml                             |   23 +
 assets/usd/configuration/wheeled_arm_base.usd      |  Bin 0 -> 80832643 bytes
 assets/usd/configuration/wheeled_arm_physics.usd   |  Bin 0 -> 6317 bytes
 assets/usd/configuration/wheeled_arm_robot.usd     |  Bin 0 -> 1835 bytes
 assets/usd/configuration/wheeled_arm_sensor.usd    |  Bin 0 -> 651 bytes
 assets/usd/wheeled_arm.usd                         |  Bin 0 -> 1460 bytes
 assets/usd_V/config.yaml                           |   23 +
 .../configuration/wheeled_urdf_init_V_base.usd     |  Bin 0 -> 80827968 bytes
 .../configuration/wheeled_urdf_init_V_physics.usd  |  Bin 0 -> 4330 bytes
 .../configuration/wheeled_urdf_init_V_robot.usd    |  Bin 0 -> 1605 bytes
 .../configuration/wheeled_urdf_init_V_sensor.usd   |  Bin 0 -> 659 bytes
 assets/usd_V/wheeled_urdf_init_V.usd               |  Bin 0 -> 1469 bytes
 assets/usd_arm/config.yaml                         |   23 +
 assets/usd_arm/configuration/wheeled_arm_base.usd  |  Bin 0 -> 80832321 bytes
 .../usd_arm/configuration/wheeled_arm_physics.usd  |  Bin 0 -> 6319 bytes
 assets/usd_arm/configuration/wheeled_arm_robot.usd |  Bin 0 -> 1835 bytes
 .../usd_arm/configuration/wheeled_arm_sensor.usd   |  Bin 0 -> 651 bytes
 assets/usd_arm/wheeled_arm.usd                     |  Bin 0 -> 1460 bytes
 assets/usd_leg/config.yaml                         |   23 +
 assets/usd_leg/configuration/wheeled_leg_base.usd  |  Bin 0 -> 80829690 bytes
 .../usd_leg/configuration/wheeled_leg_physics.usd  |  Bin 0 -> 4299 bytes
 assets/usd_leg/configuration/wheeled_leg_robot.usd |  Bin 0 -> 1597 bytes
 .../usd_leg/configuration/wheeled_leg_sensor.usd   |  Bin 0 -> 651 bytes
 assets/usd_leg/wheeled_leg.usd                     |  Bin 0 -> 1460 bytes
 assets/wheeled_arm/CMakeLists.txt                  |   14 +
 .../config/joint_names_wheeled_arm.yaml            |    1 +
 assets/wheeled_arm/export.log                      | 1621 ++++++++++++++++++++
 assets/wheeled_arm/launch/display.launch           |   20 +
 assets/wheeled_arm/launch/gazebo.launch            |   20 +
 assets/wheeled_arm/meshes/arm1_link.STL            |  Bin 0 -> 1219284 bytes
 assets/wheeled_arm/meshes/arm2_Link.STL            |  Bin 0 -> 2989384 bytes
 assets/wheeled_arm/meshes/arm3_Link.STL            |  Bin 0 -> 1824784 bytes
 assets/wheeled_arm/meshes/arm4_Link.STL            |  Bin 0 -> 2683384 bytes
 assets/wheeled_arm/meshes/armr_link.STL            |  Bin 0 -> 2711484 bytes
 assets/wheeled_arm/meshes/base_link.STL            |  Bin 0 -> 30402834 bytes
 assets/wheeled_arm/meshes/claw_Link.STL            |  Bin 0 -> 529284 bytes
 assets/wheeled_arm/meshes/left_hip_link.STL        |  Bin 0 -> 4939734 bytes
 assets/wheeled_arm/meshes/left_knee_link.STL       |  Bin 0 -> 359284 bytes
 assets/wheeled_arm/meshes/left_wheel_link.STL      |  Bin 0 -> 1583484 bytes
 assets/wheeled_arm/meshes/right_hip_link.STL       |  Bin 0 -> 4939734 bytes
 assets/wheeled_arm/meshes/right_knee_link.STL      |  Bin 0 -> 359284 bytes
 assets/wheeled_arm/meshes/right_wheel_link.STL     |  Bin 0 -> 1583484 bytes
 assets/wheeled_arm/package.xml                     |   21 +
 assets/wheeled_arm/urdf/.asset_hash                |    1 +
 assets/wheeled_arm/urdf/config.yaml                |   23 +
 .../urdf/configuration/wheeled_arm_base.usd        |  Bin 0 -> 80832321 bytes
 .../urdf/configuration/wheeled_arm_physics.usd     |  Bin 0 -> 6319 bytes
 .../urdf/configuration/wheeled_arm_robot.usd       |  Bin 0 -> 1835 bytes
 .../urdf/configuration/wheeled_arm_sensor.usd      |  Bin 0 -> 651 bytes
 assets/wheeled_arm/urdf/wheeled_arm.csv            |   14 +
 assets/wheeled_arm/urdf/wheeled_arm.urdf           |  739 +++++++++
 assets/wheeled_arm/urdf/wheeled_arm.usd            |  Bin 0 -> 1460 bytes
 assets/wheeled_leg/CMakeLists.txt                  |   14 +
 .../config/joint_names_wheeled_leg.yaml            |    1 +
 assets/wheeled_leg/export.log                      |  587 +++++++
 assets/wheeled_leg/launch/display.launch           |   20 +
 assets/wheeled_leg/launch/gazebo.launch            |   20 +
 assets/wheeled_leg/meshes/base_link.STL            |  Bin 0 -> 42359934 bytes
 assets/wheeled_leg/meshes/left_hip_link.STL        |  Bin 0 -> 4939734 bytes
 assets/wheeled_leg/meshes/left_knee_link.STL       |  Bin 0 -> 359284 bytes
 assets/wheeled_leg/meshes/left_wheel_link.STL      |  Bin 0 -> 1583484 bytes
 assets/wheeled_leg/meshes/right_hip_link.STL       |  Bin 0 -> 4939734 bytes
 assets/wheeled_leg/meshes/right_knee_link.STL      |  Bin 0 -> 359284 bytes
 assets/wheeled_leg/meshes/right_wheel_link.STL     |  Bin 0 -> 1583484 bytes
 assets/wheeled_leg/package.xml                     |   21 +
 assets/wheeled_leg/urdf/.asset_hash                |    1 +
 assets/wheeled_leg/urdf/config.yaml                |   23 +
 .../urdf/configuration/wheeled_leg_base.usd        |  Bin 0 -> 80829690 bytes
 .../urdf/configuration/wheeled_leg_physics.usd     |  Bin 0 -> 4299 bytes
 .../urdf/configuration/wheeled_leg_robot.usd       |  Bin 0 -> 1597 bytes
 .../urdf/configuration/wheeled_leg_sensor.usd      |  Bin 0 -> 651 bytes
 assets/wheeled_leg/urdf/wheeled_leg.csv            |    8 +
 assets/wheeled_leg/urdf/wheeled_leg.urdf           |  391 +++++
 assets/wheeled_leg/urdf/wheeled_leg.usd            |  Bin 0 -> 1460 bytes
 modules/wheeled_mujoco/MUJOCO_DEBUG_HANDOFF.md     |  648 ++++++++
 modules/wheeled_mujoco/README.md                   |    4 -
 modules/wheeled_mujoco/asset/mjcf/wheeled_arm.xml  |  207 +++
 modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml  |  132 ++
 .../asset/wheeled_arm/CMakeLists.txt               |   14 +
 .../config/joint_names_wheeled_arm.yaml            |    1 +
 .../wheeled_mujoco/asset/wheeled_arm/export.log    | 1621 ++++++++++++++++++++
 .../asset/wheeled_arm/launch/display.launch        |   20 +
 .../asset/wheeled_arm/launch/gazebo.launch         |   20 +
 .../asset/wheeled_arm/meshes/arm1_link.STL         |  Bin 0 -> 1219284 bytes
 .../asset/wheeled_arm/meshes/arm2_Link.STL         |  Bin 0 -> 2989384 bytes
 .../asset/wheeled_arm/meshes/arm3_Link.STL         |  Bin 0 -> 1824784 bytes
 .../asset/wheeled_arm/meshes/arm4_Link.STL         |  Bin 0 -> 2683384 bytes
 .../asset/wheeled_arm/meshes/armr_link.STL         |  Bin 0 -> 2711484 bytes
 .../asset/wheeled_arm/meshes/base_link.STL         |  Bin 0 -> 9500084 bytes
 .../asset/wheeled_arm/meshes/claw_Link.STL         |  Bin 0 -> 529284 bytes
 .../asset/wheeled_arm/meshes/left_hip_link.STL     |  Bin 0 -> 4939734 bytes
 .../asset/wheeled_arm/meshes/left_knee_link.STL    |  Bin 0 -> 359284 bytes
 .../asset/wheeled_arm/meshes/left_wheel_link.STL   |  Bin 0 -> 1583484 bytes
 .../asset/wheeled_arm/meshes/right_hip_link.STL    |  Bin 0 -> 4939734 bytes
 .../asset/wheeled_arm/meshes/right_knee_link.STL   |  Bin 0 -> 179684 bytes
 .../asset/wheeled_arm/meshes/right_wheel_link.STL  |  Bin 0 -> 1583484 bytes
 .../asset/wheeled_arm/meshes/wheeled_arm.urdf      |  752 +++++++++
 .../asset/wheeled_arm/meshes/wheeled_arm.xml       |  207 +++
 .../wheeled_mujoco/asset/wheeled_arm/package.xml   |   21 +
 .../asset/wheeled_arm/urdf/wheeled_arm.csv         |   14 +
 .../asset/wheeled_arm/urdf/wheeled_arm.urdf        |  739 +++++++++
 .../asset/wheeled_leg/CMakeLists.txt               |   14 +
 .../config/joint_names_wheeled_leg.yaml            |    1 +
 .../wheeled_mujoco/asset/wheeled_leg/export.log    |  587 +++++++
 .../asset/wheeled_leg/launch/display.launch        |   20 +
 .../asset/wheeled_leg/launch/gazebo.launch         |   20 +
 .../asset/wheeled_leg/meshes/base_link.STL         |  Bin 0 -> 9500084 bytes
 .../asset/wheeled_leg/meshes/left_hip_link.STL     |  Bin 0 -> 500084 bytes
 .../asset/wheeled_leg/meshes/left_knee_link.STL    |  Bin 0 -> 359284 bytes
 .../asset/wheeled_leg/meshes/left_wheel_link.STL   |  Bin 0 -> 500084 bytes
 .../asset/wheeled_leg/meshes/right_hip_link.STL    |  Bin 0 -> 500084 bytes
 .../asset/wheeled_leg/meshes/right_knee_link.STL   |  Bin 0 -> 359284 bytes
 .../asset/wheeled_leg/meshes/right_wheel_link.STL  |  Bin 0 -> 500084 bytes
 .../asset/wheeled_leg/meshes/wheeled_leg.urdf      |  404 +++++
 .../asset/wheeled_leg/meshes/wheeled_leg.xml       |  132 ++
 .../wheeled_mujoco/asset/wheeled_leg/package.xml   |   21 +
 .../asset/wheeled_leg/urdf/wheeled_leg.csv         |    8 +
 .../asset/wheeled_leg/urdf/wheeled_leg.urdf        |  391 +++++
 modules/wheeled_mujoco/minimal_sim2sim/README.md   |  155 ++
 .../wheeled_mujoco/minimal_sim2sim/README.zh.md    |  179 +++
 .../__pycache__/run_base_locked.cpython-310.pyc    |  Bin 0 -> 38061 bytes
 .../__pycache__/run_base_locked.cpython-313.pyc    |  Bin 0 -> 75995 bytes
 .../minimal_sim2sim/run_base_locked.py             | 1160 ++++++++++++++
 modules/wheeled_mujoco/readme.md                   |   22 +
 modules/wheeled_mujoco/readme.zh.md                |   22 +
 modules/wheeled_mujoco/scripts/readme.md           |   95 ++
 modules/wheeled_mujoco/scripts/readme.zh.md        |   95 ++
 .../scripts/run_base_locked_sim2sim.py             |   25 +
 modules/wheeled_mujoco/scripts/run_task_sim2sim.py |  100 ++
 modules/wheeled_mujoco/sim2sim.py                  |  193 +++
 modules/wheeled_mujoco/source/readme.md            |   12 +
 modules/wheeled_mujoco/source/readme.zh.md         |   12 +
 .../source/wheeled_mujoco/__init__.py              |    5 +
 .../__pycache__/__init__.cpython-310.pyc           |  Bin 0 -> 351 bytes
 .../__pycache__/paths.cpython-310.pyc              |  Bin 0 -> 981 bytes
 .../wheeled_mujoco/orchestration/__init__.py       |    5 +
 .../source/wheeled_mujoco/orchestration/readme.md  |   11 +
 .../wheeled_mujoco/orchestration/readme.zh.md      |   11 +
 .../wheeled_mujoco/orchestration/task_switcher.py  |   48 +
 .../wheeled_mujoco/source/wheeled_mujoco/paths.py  |   20 +
 .../wheeled_mujoco/source/wheeled_mujoco/readme.md |   16 +
 .../source/wheeled_mujoco/readme.zh.md             |   16 +
 .../source/wheeled_mujoco/ros/__init__.py          |    5 +
 .../source/wheeled_mujoco/ros/interfaces.py        |   85 +
 .../source/wheeled_mujoco/ros/readme.md            |   16 +
 .../source/wheeled_mujoco/ros/readme.zh.md         |   16 +
 .../source/wheeled_mujoco/sim2sim/__init__.py      |    5 +
 .../sim2sim/__pycache__/__init__.cpython-310.pyc   |  Bin 0 -> 377 bytes
 .../__pycache__/base_locked.cpython-310.pyc        |  Bin 0 -> 10223 bytes
 .../source/wheeled_mujoco/sim2sim/base_locked.py   |  340 ++++
 .../source/wheeled_mujoco/sim2sim/readme.md        |   10 +
 .../source/wheeled_mujoco/sim2sim/readme.zh.md     |   10 +
 .../source/wheeled_mujoco/tasks/__init__.py        |   40 +
 .../tasks/__pycache__/__init__.cpython-310.pyc     |  Bin 0 -> 1355 bytes
 .../arm_free_locomotion_env_cfg.cpython-310.pyc    |  Bin 0 -> 799 bytes
 .../arm_free_obstacle_env_cfg.cpython-310.pyc      |  Bin 0 -> 773 bytes
 .../arm_free_recovery_env_cfg.cpython-310.pyc      |  Bin 0 -> 766 bytes
 .../base_locked_locomotion_env_cfg.cpython-310.pyc |  Bin 0 -> 1186 bytes
 .../cartesian_grasp_env_cfg.cpython-310.pyc        |  Bin 0 -> 772 bytes
 .../tasks/__pycache__/common.cpython-310.pyc       |  Bin 0 -> 2748 bytes
 .../grasp_pose_locomotion_env_cfg.cpython-310.pyc  |  Bin 0 -> 792 bytes
 .../loaded_grasp_obstacle_env_cfg.cpython-310.pyc  |  Bin 0 -> 806 bytes
 .../tasks/arm_free_locomotion_env_cfg.py           |   16 +
 .../tasks/arm_free_obstacle_env_cfg.py             |   16 +
 .../tasks/arm_free_recovery_env_cfg.py             |   16 +
 .../tasks/base_locked_locomotion_env_cfg.py        |   29 +
 .../tasks/cartesian_grasp_env_cfg.py               |   16 +
 .../source/wheeled_mujoco/tasks/common.py          |   93 ++
 .../tasks/grasp_pose_locomotion_env_cfg.py         |   16 +
 .../tasks/loaded_grasp_obstacle_env_cfg.py         |   16 +
 .../source/wheeled_mujoco/tasks/readme.md          |   17 +
 .../source/wheeled_mujoco/tasks/readme.zh.md       |   17 +
 modules/wheeled_train/README.md                    |  391 ++++-
 modules/wheeled_train/wheeled_leg                  |    1 -
 modules/wheeled_train/wheeled_leg/.dockerignore    |   27 +
 modules/wheeled_train/wheeled_leg/.flake8          |   23 +
 modules/wheeled_train/wheeled_leg/.gitattributes   |   12 +
 modules/wheeled_train/wheeled_leg/.gitignore       |   71 +
 .../wheeled_leg/.pre-commit-config.yaml            |   86 ++
 .../wheeled_train/wheeled_leg/.vscode/.gitignore   |   10 +
 .../wheeled_leg/.vscode/extensions.json            |   12 +
 .../wheeled_train/wheeled_leg/.vscode/tasks.json   |   23 +
 .../wheeled_leg/.vscode/tools/launch.template.json |  179 +++
 .../.vscode/tools/settings.template.json           |   86 ++
 .../wheeled_leg/.vscode/tools/setup_vscode.py      |  220 +++
 modules/wheeled_train/wheeled_leg/README.md        |  135 ++
 .../wheeled_leg/logs/rsl_rl/POLICY_INDEX.md        |   38 +
 ...events.out.tfevents.1782788142.moujank.747879.0 |  Bin 0 -> 12513 bytes
 .../2026-06-30_10-55-37/git/wheeled_leg.diff       |   54 +
 .../cartpole_direct/2026-06-30_10-55-37/model_0.pt |    3 +
 .../2026-06-30_10-55-37/params/agent.pkl           |  Bin 0 -> 1029 bytes
 .../2026-06-30_10-55-37/params/agent.yaml          |   43 +
 .../2026-06-30_10-55-37/params/env.pkl             |  Bin 0 -> 9134 bytes
 .../2026-06-30_10-55-37/params/env.yaml            |  515 +++++++
 ...events.out.tfevents.1782976877.moujank.624619.0 |  Bin 0 -> 40608622 bytes
 .../2026-07-02_15-21-02/exported/policy.onnx       |  Bin 0 -> 25282 bytes
 .../2026-07-02_15-21-02/exported/policy.pt         |    3 +
 .../2026-07-02_15-21-02/git/wheeled_leg.diff       |  929 +++++++++++
 .../2026-07-02_15-21-02/model_0.pt                 |    3 +
 .../2026-07-02_15-21-02/model_100.pt               |    3 +
 .../2026-07-02_15-21-02/model_1000.pt              |    3 +
 .../2026-07-02_15-21-02/model_10000.pt             |    3 +
 .../2026-07-02_15-21-02/model_10050.pt             |    3 +
 .../2026-07-02_15-21-02/model_10100.pt             |    3 +
 .../2026-07-02_15-21-02/model_10150.pt             |    3 +
 .../2026-07-02_15-21-02/model_10200.pt             |    3 +
 .../2026-07-02_15-21-02/model_10250.pt             |    3 +
 .../2026-07-02_15-21-02/model_10300.pt             |    3 +
 .../2026-07-02_15-21-02/model_10350.pt             |    3 +
 .../2026-07-02_15-21-02/model_10400.pt             |    3 +
 .../2026-07-02_15-21-02/model_10450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1050.pt              |    3 +
 .../2026-07-02_15-21-02/model_10500.pt             |    3 +
 .../2026-07-02_15-21-02/model_10550.pt             |    3 +
 .../2026-07-02_15-21-02/model_10600.pt             |    3 +
 .../2026-07-02_15-21-02/model_10650.pt             |    3 +
 .../2026-07-02_15-21-02/model_10700.pt             |    3 +
 .../2026-07-02_15-21-02/model_10750.pt             |    3 +
 .../2026-07-02_15-21-02/model_10800.pt             |    3 +
 .../2026-07-02_15-21-02/model_10850.pt             |    3 +
 .../2026-07-02_15-21-02/model_10900.pt             |    3 +
 .../2026-07-02_15-21-02/model_10950.pt             |    3 +
 .../2026-07-02_15-21-02/model_1100.pt              |    3 +
 .../2026-07-02_15-21-02/model_11000.pt             |    3 +
 .../2026-07-02_15-21-02/model_11050.pt             |    3 +
 .../2026-07-02_15-21-02/model_11100.pt             |    3 +
 .../2026-07-02_15-21-02/model_11150.pt             |    3 +
 .../2026-07-02_15-21-02/model_11200.pt             |    3 +
 .../2026-07-02_15-21-02/model_11250.pt             |    3 +
 .../2026-07-02_15-21-02/model_11300.pt             |    3 +
 .../2026-07-02_15-21-02/model_11350.pt             |    3 +
 .../2026-07-02_15-21-02/model_11400.pt             |    3 +
 .../2026-07-02_15-21-02/model_11450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1150.pt              |    3 +
 .../2026-07-02_15-21-02/model_11500.pt             |    3 +
 .../2026-07-02_15-21-02/model_11550.pt             |    3 +
 .../2026-07-02_15-21-02/model_11600.pt             |    3 +
 .../2026-07-02_15-21-02/model_11650.pt             |    3 +
 .../2026-07-02_15-21-02/model_11700.pt             |    3 +
 .../2026-07-02_15-21-02/model_11750.pt             |    3 +
 .../2026-07-02_15-21-02/model_11800.pt             |    3 +
 .../2026-07-02_15-21-02/model_11850.pt             |    3 +
 .../2026-07-02_15-21-02/model_11900.pt             |    3 +
 .../2026-07-02_15-21-02/model_11950.pt             |    3 +
 .../2026-07-02_15-21-02/model_1200.pt              |    3 +
 .../2026-07-02_15-21-02/model_12000.pt             |    3 +
 .../2026-07-02_15-21-02/model_12050.pt             |    3 +
 .../2026-07-02_15-21-02/model_12100.pt             |    3 +
 .../2026-07-02_15-21-02/model_12150.pt             |    3 +
 .../2026-07-02_15-21-02/model_12200.pt             |    3 +
 .../2026-07-02_15-21-02/model_12250.pt             |    3 +
 .../2026-07-02_15-21-02/model_12300.pt             |    3 +
 .../2026-07-02_15-21-02/model_12350.pt             |    3 +
 .../2026-07-02_15-21-02/model_12400.pt             |    3 +
 .../2026-07-02_15-21-02/model_12450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1250.pt              |    3 +
 .../2026-07-02_15-21-02/model_12500.pt             |    3 +
 .../2026-07-02_15-21-02/model_12550.pt             |    3 +
 .../2026-07-02_15-21-02/model_12600.pt             |    3 +
 .../2026-07-02_15-21-02/model_12650.pt             |    3 +
 .../2026-07-02_15-21-02/model_12700.pt             |    3 +
 .../2026-07-02_15-21-02/model_12750.pt             |    3 +
 .../2026-07-02_15-21-02/model_12800.pt             |    3 +
 .../2026-07-02_15-21-02/model_12850.pt             |    3 +
 .../2026-07-02_15-21-02/model_12900.pt             |    3 +
 .../2026-07-02_15-21-02/model_12950.pt             |    3 +
 .../2026-07-02_15-21-02/model_1300.pt              |    3 +
 .../2026-07-02_15-21-02/model_13000.pt             |    3 +
 .../2026-07-02_15-21-02/model_13050.pt             |    3 +
 .../2026-07-02_15-21-02/model_13100.pt             |    3 +
 .../2026-07-02_15-21-02/model_13150.pt             |    3 +
 .../2026-07-02_15-21-02/model_13200.pt             |    3 +
 .../2026-07-02_15-21-02/model_13250.pt             |    3 +
 .../2026-07-02_15-21-02/model_13300.pt             |    3 +
 .../2026-07-02_15-21-02/model_13350.pt             |    3 +
 .../2026-07-02_15-21-02/model_13400.pt             |    3 +
 .../2026-07-02_15-21-02/model_13450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1350.pt              |    3 +
 .../2026-07-02_15-21-02/model_13500.pt             |    3 +
 .../2026-07-02_15-21-02/model_13550.pt             |    3 +
 .../2026-07-02_15-21-02/model_13600.pt             |    3 +
 .../2026-07-02_15-21-02/model_13650.pt             |    3 +
 .../2026-07-02_15-21-02/model_13700.pt             |    3 +
 .../2026-07-02_15-21-02/model_13750.pt             |    3 +
 .../2026-07-02_15-21-02/model_13800.pt             |    3 +
 .../2026-07-02_15-21-02/model_13850.pt             |    3 +
 .../2026-07-02_15-21-02/model_13900.pt             |    3 +
 .../2026-07-02_15-21-02/model_13950.pt             |    3 +
 .../2026-07-02_15-21-02/model_1400.pt              |    3 +
 .../2026-07-02_15-21-02/model_14000.pt             |    3 +
 .../2026-07-02_15-21-02/model_14050.pt             |    3 +
 .../2026-07-02_15-21-02/model_14100.pt             |    3 +
 .../2026-07-02_15-21-02/model_14150.pt             |    3 +
 .../2026-07-02_15-21-02/model_14200.pt             |    3 +
 .../2026-07-02_15-21-02/model_14250.pt             |    3 +
 .../2026-07-02_15-21-02/model_14300.pt             |    3 +
 .../2026-07-02_15-21-02/model_14350.pt             |    3 +
 .../2026-07-02_15-21-02/model_14400.pt             |    3 +
 .../2026-07-02_15-21-02/model_14450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1450.pt              |    3 +
 .../2026-07-02_15-21-02/model_14500.pt             |    3 +
 .../2026-07-02_15-21-02/model_14550.pt             |    3 +
 .../2026-07-02_15-21-02/model_14600.pt             |    3 +
 .../2026-07-02_15-21-02/model_14650.pt             |    3 +
 .../2026-07-02_15-21-02/model_14700.pt             |    3 +
 .../2026-07-02_15-21-02/model_14750.pt             |    3 +
 .../2026-07-02_15-21-02/model_14800.pt             |    3 +
 .../2026-07-02_15-21-02/model_14850.pt             |    3 +
 .../2026-07-02_15-21-02/model_14900.pt             |    3 +
 .../2026-07-02_15-21-02/model_14950.pt             |    3 +
 .../2026-07-02_15-21-02/model_150.pt               |    3 +
 .../2026-07-02_15-21-02/model_1500.pt              |    3 +
 .../2026-07-02_15-21-02/model_15000.pt             |    3 +
 .../2026-07-02_15-21-02/model_15050.pt             |    3 +
 .../2026-07-02_15-21-02/model_15100.pt             |    3 +
 .../2026-07-02_15-21-02/model_15150.pt             |    3 +
 .../2026-07-02_15-21-02/model_15200.pt             |    3 +
 .../2026-07-02_15-21-02/model_15250.pt             |    3 +
 .../2026-07-02_15-21-02/model_15300.pt             |    3 +
 .../2026-07-02_15-21-02/model_15350.pt             |    3 +
 .../2026-07-02_15-21-02/model_15400.pt             |    3 +
 .../2026-07-02_15-21-02/model_15450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1550.pt              |    3 +
 .../2026-07-02_15-21-02/model_15500.pt             |    3 +
 .../2026-07-02_15-21-02/model_15550.pt             |    3 +
 .../2026-07-02_15-21-02/model_15600.pt             |    3 +
 .../2026-07-02_15-21-02/model_15650.pt             |    3 +
 .../2026-07-02_15-21-02/model_15700.pt             |    3 +
 .../2026-07-02_15-21-02/model_15750.pt             |    3 +
 .../2026-07-02_15-21-02/model_15800.pt             |    3 +
 .../2026-07-02_15-21-02/model_15850.pt             |    3 +
 .../2026-07-02_15-21-02/model_15900.pt             |    3 +
 .../2026-07-02_15-21-02/model_15950.pt             |    3 +
 .../2026-07-02_15-21-02/model_1600.pt              |    3 +
 .../2026-07-02_15-21-02/model_16000.pt             |    3 +
 .../2026-07-02_15-21-02/model_16050.pt             |    3 +
 .../2026-07-02_15-21-02/model_16100.pt             |    3 +
 .../2026-07-02_15-21-02/model_16150.pt             |    3 +
 .../2026-07-02_15-21-02/model_16200.pt             |    3 +
 .../2026-07-02_15-21-02/model_16250.pt             |    3 +
 .../2026-07-02_15-21-02/model_16300.pt             |    3 +
 .../2026-07-02_15-21-02/model_16350.pt             |    3 +
 .../2026-07-02_15-21-02/model_16400.pt             |    3 +
 .../2026-07-02_15-21-02/model_16450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1650.pt              |    3 +
 .../2026-07-02_15-21-02/model_16500.pt             |    3 +
 .../2026-07-02_15-21-02/model_16550.pt             |    3 +
 .../2026-07-02_15-21-02/model_16600.pt             |    3 +
 .../2026-07-02_15-21-02/model_16650.pt             |    3 +
 .../2026-07-02_15-21-02/model_16700.pt             |    3 +
 .../2026-07-02_15-21-02/model_16750.pt             |    3 +
 .../2026-07-02_15-21-02/model_16800.pt             |    3 +
 .../2026-07-02_15-21-02/model_16850.pt             |    3 +
 .../2026-07-02_15-21-02/model_16900.pt             |    3 +
 .../2026-07-02_15-21-02/model_16950.pt             |    3 +
 .../2026-07-02_15-21-02/model_1700.pt              |    3 +
 .../2026-07-02_15-21-02/model_17000.pt             |    3 +
 .../2026-07-02_15-21-02/model_17050.pt             |    3 +
 .../2026-07-02_15-21-02/model_17100.pt             |    3 +
 .../2026-07-02_15-21-02/model_17150.pt             |    3 +
 .../2026-07-02_15-21-02/model_17200.pt             |    3 +
 .../2026-07-02_15-21-02/model_17250.pt             |    3 +
 .../2026-07-02_15-21-02/model_17300.pt             |    3 +
 .../2026-07-02_15-21-02/model_17350.pt             |    3 +
 .../2026-07-02_15-21-02/model_17400.pt             |    3 +
 .../2026-07-02_15-21-02/model_17450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1750.pt              |    3 +
 .../2026-07-02_15-21-02/model_17500.pt             |    3 +
 .../2026-07-02_15-21-02/model_17550.pt             |    3 +
 .../2026-07-02_15-21-02/model_17600.pt             |    3 +
 .../2026-07-02_15-21-02/model_17650.pt             |    3 +
 .../2026-07-02_15-21-02/model_17700.pt             |    3 +
 .../2026-07-02_15-21-02/model_17750.pt             |    3 +
 .../2026-07-02_15-21-02/model_17800.pt             |    3 +
 .../2026-07-02_15-21-02/model_17850.pt             |    3 +
 .../2026-07-02_15-21-02/model_17900.pt             |    3 +
 .../2026-07-02_15-21-02/model_17950.pt             |    3 +
 .../2026-07-02_15-21-02/model_1800.pt              |    3 +
 .../2026-07-02_15-21-02/model_18000.pt             |    3 +
 .../2026-07-02_15-21-02/model_18050.pt             |    3 +
 .../2026-07-02_15-21-02/model_18100.pt             |    3 +
 .../2026-07-02_15-21-02/model_18150.pt             |    3 +
 .../2026-07-02_15-21-02/model_18200.pt             |    3 +
 .../2026-07-02_15-21-02/model_18250.pt             |    3 +
 .../2026-07-02_15-21-02/model_18300.pt             |    3 +
 .../2026-07-02_15-21-02/model_18350.pt             |    3 +
 .../2026-07-02_15-21-02/model_18400.pt             |    3 +
 .../2026-07-02_15-21-02/model_18450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1850.pt              |    3 +
 .../2026-07-02_15-21-02/model_18500.pt             |    3 +
 .../2026-07-02_15-21-02/model_18550.pt             |    3 +
 .../2026-07-02_15-21-02/model_18600.pt             |    3 +
 .../2026-07-02_15-21-02/model_18650.pt             |    3 +
 .../2026-07-02_15-21-02/model_18700.pt             |    3 +
 .../2026-07-02_15-21-02/model_18750.pt             |    3 +
 .../2026-07-02_15-21-02/model_18800.pt             |    3 +
 .../2026-07-02_15-21-02/model_18850.pt             |    3 +
 .../2026-07-02_15-21-02/model_18900.pt             |    3 +
 .../2026-07-02_15-21-02/model_18950.pt             |    3 +
 .../2026-07-02_15-21-02/model_1900.pt              |    3 +
 .../2026-07-02_15-21-02/model_19000.pt             |    3 +
 .../2026-07-02_15-21-02/model_19050.pt             |    3 +
 .../2026-07-02_15-21-02/model_19100.pt             |    3 +
 .../2026-07-02_15-21-02/model_19150.pt             |    3 +
 .../2026-07-02_15-21-02/model_19200.pt             |    3 +
 .../2026-07-02_15-21-02/model_19250.pt             |    3 +
 .../2026-07-02_15-21-02/model_19300.pt             |    3 +
 .../2026-07-02_15-21-02/model_19350.pt             |    3 +
 .../2026-07-02_15-21-02/model_19400.pt             |    3 +
 .../2026-07-02_15-21-02/model_19450.pt             |    3 +
 .../2026-07-02_15-21-02/model_1950.pt              |    3 +
 .../2026-07-02_15-21-02/model_19500.pt             |    3 +
 .../2026-07-02_15-21-02/model_19550.pt             |    3 +
 .../2026-07-02_15-21-02/model_19600.pt             |    3 +
 .../2026-07-02_15-21-02/model_19650.pt             |    3 +
 .../2026-07-02_15-21-02/model_19700.pt             |    3 +
 .../2026-07-02_15-21-02/model_19750.pt             |    3 +
 .../2026-07-02_15-21-02/model_19800.pt             |    3 +
 .../2026-07-02_15-21-02/model_19850.pt             |    3 +
 .../2026-07-02_15-21-02/model_19900.pt             |    3 +
 .../2026-07-02_15-21-02/model_19950.pt             |    3 +
 .../2026-07-02_15-21-02/model_19999.pt             |    3 +
 .../2026-07-02_15-21-02/model_200.pt               |    3 +
 .../2026-07-02_15-21-02/model_2000.pt              |    3 +
 .../2026-07-02_15-21-02/model_2050.pt              |    3 +
 .../2026-07-02_15-21-02/model_2100.pt              |    3 +
 .../2026-07-02_15-21-02/model_2150.pt              |    3 +
 .../2026-07-02_15-21-02/model_2200.pt              |    3 +
 .../2026-07-02_15-21-02/model_2250.pt              |    3 +
 .../2026-07-02_15-21-02/model_2300.pt              |    3 +
 .../2026-07-02_15-21-02/model_2350.pt              |    3 +
 .../2026-07-02_15-21-02/model_2400.pt              |    3 +
 .../2026-07-02_15-21-02/model_2450.pt              |    3 +
 .../2026-07-02_15-21-02/model_250.pt               |    3 +
 .../2026-07-02_15-21-02/model_2500.pt              |    3 +
 .../2026-07-02_15-21-02/model_2550.pt              |    3 +
 .../2026-07-02_15-21-02/model_2600.pt              |    3 +
 .../2026-07-02_15-21-02/model_2650.pt              |    3 +
 .../2026-07-02_15-21-02/model_2700.pt              |    3 +
 .../2026-07-02_15-21-02/model_2750.pt              |    3 +
 .../2026-07-02_15-21-02/model_2800.pt              |    3 +
 .../2026-07-02_15-21-02/model_2850.pt              |    3 +
 .../2026-07-02_15-21-02/model_2900.pt              |    3 +
 .../2026-07-02_15-21-02/model_2950.pt              |    3 +
 .../2026-07-02_15-21-02/model_300.pt               |    3 +
 .../2026-07-02_15-21-02/model_3000.pt              |    3 +
 .../2026-07-02_15-21-02/model_3050.pt              |    3 +
 .../2026-07-02_15-21-02/model_3100.pt              |    3 +
 .../2026-07-02_15-21-02/model_3150.pt              |    3 +
 .../2026-07-02_15-21-02/model_3200.pt              |    3 +
 .../2026-07-02_15-21-02/model_3250.pt              |    3 +
 .../2026-07-02_15-21-02/model_3300.pt              |    3 +
 .../2026-07-02_15-21-02/model_3350.pt              |    3 +
 .../2026-07-02_15-21-02/model_3400.pt              |    3 +
 .../2026-07-02_15-21-02/model_3450.pt              |    3 +
 .../2026-07-02_15-21-02/model_350.pt               |    3 +
 .../2026-07-02_15-21-02/model_3500.pt              |    3 +
 .../2026-07-02_15-21-02/model_3550.pt              |    3 +
 .../2026-07-02_15-21-02/model_3600.pt              |    3 +
 .../2026-07-02_15-21-02/model_3650.pt              |    3 +
 .../2026-07-02_15-21-02/model_3700.pt              |    3 +
 .../2026-07-02_15-21-02/model_3750.pt              |    3 +
 .../2026-07-02_15-21-02/model_3800.pt              |    3 +
 .../2026-07-02_15-21-02/model_3850.pt              |    3 +
 .../2026-07-02_15-21-02/model_3900.pt              |    3 +
 .../2026-07-02_15-21-02/model_3950.pt              |    3 +
 .../2026-07-02_15-21-02/model_400.pt               |    3 +
 .../2026-07-02_15-21-02/model_4000.pt              |    3 +
 .../2026-07-02_15-21-02/model_4050.pt              |    3 +
 .../2026-07-02_15-21-02/model_4100.pt              |    3 +
 .../2026-07-02_15-21-02/model_4150.pt              |    3 +
 .../2026-07-02_15-21-02/model_4200.pt              |    3 +
 .../2026-07-02_15-21-02/model_4250.pt              |    3 +
 .../2026-07-02_15-21-02/model_4300.pt              |    3 +
 .../2026-07-02_15-21-02/model_4350.pt              |    3 +
 .../2026-07-02_15-21-02/model_4400.pt              |    3 +
 .../2026-07-02_15-21-02/model_4450.pt              |    3 +
 .../2026-07-02_15-21-02/model_450.pt               |    3 +
 .../2026-07-02_15-21-02/model_4500.pt              |    3 +
 .../2026-07-02_15-21-02/model_4550.pt              |    3 +
 .../2026-07-02_15-21-02/model_4600.pt              |    3 +
 .../2026-07-02_15-21-02/model_4650.pt              |    3 +
 .../2026-07-02_15-21-02/model_4700.pt              |    3 +
 .../2026-07-02_15-21-02/model_4750.pt              |    3 +
 .../2026-07-02_15-21-02/model_4800.pt              |    3 +
 .../2026-07-02_15-21-02/model_4850.pt              |    3 +
 .../2026-07-02_15-21-02/model_4900.pt              |    3 +
 .../2026-07-02_15-21-02/model_4950.pt              |    3 +
 .../2026-07-02_15-21-02/model_50.pt                |    3 +
 .../2026-07-02_15-21-02/model_500.pt               |    3 +
 .../2026-07-02_15-21-02/model_5000.pt              |    3 +
 .../2026-07-02_15-21-02/model_5050.pt              |    3 +
 .../2026-07-02_15-21-02/model_5100.pt              |    3 +
 .../2026-07-02_15-21-02/model_5150.pt              |    3 +
 .../2026-07-02_15-21-02/model_5200.pt              |    3 +
 .../2026-07-02_15-21-02/model_5250.pt              |    3 +
 .../2026-07-02_15-21-02/model_5300.pt              |    3 +
 .../2026-07-02_15-21-02/model_5350.pt              |    3 +
 .../2026-07-02_15-21-02/model_5400.pt              |    3 +
 .../2026-07-02_15-21-02/model_5450.pt              |    3 +
 .../2026-07-02_15-21-02/model_550.pt               |    3 +
 .../2026-07-02_15-21-02/model_5500.pt              |    3 +
 .../2026-07-02_15-21-02/model_5550.pt              |    3 +
 .../2026-07-02_15-21-02/model_5600.pt              |    3 +
 .../2026-07-02_15-21-02/model_5650.pt              |    3 +
 .../2026-07-02_15-21-02/model_5700.pt              |    3 +
 .../2026-07-02_15-21-02/model_5750.pt              |    3 +
 .../2026-07-02_15-21-02/model_5800.pt              |    3 +
 .../2026-07-02_15-21-02/model_5850.pt              |    3 +
 .../2026-07-02_15-21-02/model_5900.pt              |    3 +
 .../2026-07-02_15-21-02/model_5950.pt              |    3 +
 .../2026-07-02_15-21-02/model_600.pt               |    3 +
 .../2026-07-02_15-21-02/model_6000.pt              |    3 +
 .../2026-07-02_15-21-02/model_6050.pt              |    3 +
 .../2026-07-02_15-21-02/model_6100.pt              |    3 +
 .../2026-07-02_15-21-02/model_6150.pt              |    3 +
 .../2026-07-02_15-21-02/model_6200.pt              |    3 +
 .../2026-07-02_15-21-02/model_6250.pt              |    3 +
 .../2026-07-02_15-21-02/model_6300.pt              |    3 +
 .../2026-07-02_15-21-02/model_6350.pt              |    3 +
 .../2026-07-02_15-21-02/model_6400.pt              |    3 +
 .../2026-07-02_15-21-02/model_6450.pt              |    3 +
 .../2026-07-02_15-21-02/model_650.pt               |    3 +
 .../2026-07-02_15-21-02/model_6500.pt              |    3 +
 .../2026-07-02_15-21-02/model_6550.pt              |    3 +
 .../2026-07-02_15-21-02/model_6600.pt              |    3 +
 .../2026-07-02_15-21-02/model_6650.pt              |    3 +
 .../2026-07-02_15-21-02/model_6700.pt              |    3 +
 .../2026-07-02_15-21-02/model_6750.pt              |    3 +
 .../2026-07-02_15-21-02/model_6800.pt              |    3 +
 .../2026-07-02_15-21-02/model_6850.pt              |    3 +
 .../2026-07-02_15-21-02/model_6900.pt              |    3 +
 .../2026-07-02_15-21-02/model_6950.pt              |    3 +
 .../2026-07-02_15-21-02/model_700.pt               |    3 +
 .../2026-07-02_15-21-02/model_7000.pt              |    3 +
 .../2026-07-02_15-21-02/model_7050.pt              |    3 +
 .../2026-07-02_15-21-02/model_7100.pt              |    3 +
 .../2026-07-02_15-21-02/model_7150.pt              |    3 +
 .../2026-07-02_15-21-02/model_7200.pt              |    3 +
 .../2026-07-02_15-21-02/model_7250.pt              |    3 +
 .../2026-07-02_15-21-02/model_7300.pt              |    3 +
 .../2026-07-02_15-21-02/model_7350.pt              |    3 +
 .../2026-07-02_15-21-02/model_7400.pt              |    3 +
 .../2026-07-02_15-21-02/model_7450.pt              |    3 +
 .../2026-07-02_15-21-02/model_750.pt               |    3 +
 .../2026-07-02_15-21-02/model_7500.pt              |    3 +
 .../2026-07-02_15-21-02/model_7550.pt              |    3 +
 .../2026-07-02_15-21-02/model_7600.pt              |    3 +
 .../2026-07-02_15-21-02/model_7650.pt              |    3 +
 .../2026-07-02_15-21-02/model_7700.pt              |    3 +
 .../2026-07-02_15-21-02/model_7750.pt              |    3 +
 .../2026-07-02_15-21-02/model_7800.pt              |    3 +
 .../2026-07-02_15-21-02/model_7850.pt              |    3 +
 .../2026-07-02_15-21-02/model_7900.pt              |    3 +
 .../2026-07-02_15-21-02/model_7950.pt              |    3 +
 .../2026-07-02_15-21-02/model_800.pt               |    3 +
 .../2026-07-02_15-21-02/model_8000.pt              |    3 +
 .../2026-07-02_15-21-02/model_8050.pt              |    3 +
 .../2026-07-02_15-21-02/model_8100.pt              |    3 +
 .../2026-07-02_15-21-02/model_8150.pt              |    3 +
 .../2026-07-02_15-21-02/model_8200.pt              |    3 +
 .../2026-07-02_15-21-02/model_8250.pt              |    3 +
 .../2026-07-02_15-21-02/model_8300.pt              |    3 +
 .../2026-07-02_15-21-02/model_8350.pt              |    3 +
 .../2026-07-02_15-21-02/model_8400.pt              |    3 +
 .../2026-07-02_15-21-02/model_8450.pt              |    3 +
 .../2026-07-02_15-21-02/model_850.pt               |    3 +
 .../2026-07-02_15-21-02/model_8500.pt              |    3 +
 .../2026-07-02_15-21-02/model_8550.pt              |    3 +
 .../2026-07-02_15-21-02/model_8600.pt              |    3 +
 .../2026-07-02_15-21-02/model_8650.pt              |    3 +
 .../2026-07-02_15-21-02/model_8700.pt              |    3 +
 .../2026-07-02_15-21-02/model_8750.pt              |    3 +
 .../2026-07-02_15-21-02/model_8800.pt              |    3 +
 .../2026-07-02_15-21-02/model_8850.pt              |    3 +
 .../2026-07-02_15-21-02/model_8900.pt              |    3 +
 .../2026-07-02_15-21-02/model_8950.pt              |    3 +
 .../2026-07-02_15-21-02/model_900.pt               |    3 +
 .../2026-07-02_15-21-02/model_9000.pt              |    3 +
 .../2026-07-02_15-21-02/model_9050.pt              |    3 +
 .../2026-07-02_15-21-02/model_9100.pt              |    3 +
 .../2026-07-02_15-21-02/model_9150.pt              |    3 +
 .../2026-07-02_15-21-02/model_9200.pt              |    3 +
 .../2026-07-02_15-21-02/model_9250.pt              |    3 +
 .../2026-07-02_15-21-02/model_9300.pt              |    3 +
 .../2026-07-02_15-21-02/model_9350.pt              |    3 +
 .../2026-07-02_15-21-02/model_9400.pt              |    3 +
 .../2026-07-02_15-21-02/model_9450.pt              |    3 +
 .../2026-07-02_15-21-02/model_950.pt               |    3 +
 .../2026-07-02_15-21-02/model_9500.pt              |    3 +
 .../2026-07-02_15-21-02/model_9550.pt              |    3 +
 .../2026-07-02_15-21-02/model_9600.pt              |    3 +
 .../2026-07-02_15-21-02/model_9650.pt              |    3 +
 .../2026-07-02_15-21-02/model_9700.pt              |    3 +
 .../2026-07-02_15-21-02/model_9750.pt              |    3 +
 .../2026-07-02_15-21-02/model_9800.pt              |    3 +
 .../2026-07-02_15-21-02/model_9850.pt              |    3 +
 .../2026-07-02_15-21-02/model_9900.pt              |    3 +
 .../2026-07-02_15-21-02/model_9950.pt              |    3 +
 .../2026-07-02_15-21-02/params/agent.pkl           |  Bin 0 -> 1030 bytes
 .../2026-07-02_15-21-02/params/agent.yaml          |   43 +
 .../2026-07-02_15-21-02/params/env.pkl             |  Bin 0 -> 17953 bytes
 .../2026-07-02_15-21-02/params/env.yaml            | 1125 ++++++++++++++
 ...events.out.tfevents.1783063089.moujank.488937.0 |  Bin 0 -> 31300457 bytes
 .../2026-07-03_15-17-52/git/wheeled_leg.diff       |  699 +++++++++
 .../2026-07-03_15-17-52/model_0.pt                 |    3 +
 .../2026-07-03_15-17-52/model_100.pt               |    3 +
 .../2026-07-03_15-17-52/model_1000.pt              |    3 +
 .../2026-07-03_15-17-52/model_10000.pt             |    3 +
 .../2026-07-03_15-17-52/model_10050.pt             |    3 +
 .../2026-07-03_15-17-52/model_10100.pt             |    3 +
 .../2026-07-03_15-17-52/model_10150.pt             |    3 +
 .../2026-07-03_15-17-52/model_10200.pt             |    3 +
 .../2026-07-03_15-17-52/model_10250.pt             |    3 +
 .../2026-07-03_15-17-52/model_10300.pt             |    3 +
 .../2026-07-03_15-17-52/model_10350.pt             |    3 +
 .../2026-07-03_15-17-52/model_10400.pt             |    3 +
 .../2026-07-03_15-17-52/model_10450.pt             |    3 +
 .../2026-07-03_15-17-52/model_1050.pt              |    3 +
 .../2026-07-03_15-17-52/model_10500.pt             |    3 +
 .../2026-07-03_15-17-52/model_10550.pt             |    3 +
 .../2026-07-03_15-17-52/model_10600.pt             |    3 +
 .../2026-07-03_15-17-52/model_10650.pt             |    3 +
 .../2026-07-03_15-17-52/model_10700.pt             |    3 +
 .../2026-07-03_15-17-52/model_10750.pt             |    3 +
 .../2026-07-03_15-17-52/model_10800.pt             |    3 +
 .../2026-07-03_15-17-52/model_10850.pt             |    3 +
 .../2026-07-03_15-17-52/model_10900.pt             |    3 +
 .../2026-07-03_15-17-52/model_10950.pt             |    3 +
 .../2026-07-03_15-17-52/model_1100.pt              |    3 +
 .../2026-07-03_15-17-52/model_11000.pt             |    3 +
 .../2026-07-03_15-17-52/model_11050.pt             |    3 +
 .../2026-07-03_15-17-52/model_11100.pt             |    3 +
 .../2026-07-03_15-17-52/model_11150.pt             |    3 +
 .../2026-07-03_15-17-52/model_11200.pt             |    3 +
 .../2026-07-03_15-17-52/model_11250.pt             |    3 +
 .../2026-07-03_15-17-52/model_11300.pt             |    3 +
 .../2026-07-03_15-17-52/model_11350.pt             |    3 +
 .../2026-07-03_15-17-52/model_11400.pt             |    3 +
 .../2026-07-03_15-17-52/model_11450.pt             |    3 +
 .../2026-07-03_15-17-52/model_1150.pt              |    3 +
 .../2026-07-03_15-17-52/model_11500.pt             |    3 +
 .../2026-07-03_15-17-52/model_11550.pt             |    3 +
 .../2026-07-03_15-17-52/model_11600.pt             |    3 +
 .../2026-07-03_15-17-52/model_11650.pt             |    3 +
 .../2026-07-03_15-17-52/model_11700.pt             |    3 +
 .../2026-07-03_15-17-52/model_11750.pt             |    3 +
 .../2026-07-03_15-17-52/model_11800.pt             |    3 +
 .../2026-07-03_15-17-52/model_11850.pt             |    3 +
 .../2026-07-03_15-17-52/model_11900.pt             |    3 +
 .../2026-07-03_15-17-52/model_11950.pt             |    3 +
 .../2026-07-03_15-17-52/model_1200.pt              |    3 +
 .../2026-07-03_15-17-52/model_12000.pt             |    3 +
 .../2026-07-03_15-17-52/model_12050.pt             |    3 +
 .../2026-07-03_15-17-52/model_12100.pt             |    3 +
 .../2026-07-03_15-17-52/model_12150.pt             |    3 +
 .../2026-07-03_15-17-52/model_12200.pt             |    3 +
 .../2026-07-03_15-17-52/model_12250.pt             |    3 +
 .../2026-07-03_15-17-52/model_12300.pt             |    3 +
 .../2026-07-03_15-17-52/model_12350.pt             |    3 +
 .../2026-07-03_15-17-52/model_12400.pt             |    3 +
 .../2026-07-03_15-17-52/model_12450.pt             |    3 +
 .../2026-07-03_15-17-52/model_1250.pt              |    3 +
 .../2026-07-03_15-17-52/model_12500.pt             |    3 +
 .../2026-07-03_15-17-52/model_12550.pt             |    3 +
 .../2026-07-03_15-17-52/model_12600.pt             |    3 +
 .../2026-07-03_15-17-52/model_12650.pt             |    3 +
 .../2026-07-03_15-17-52/model_12700.pt             |    3 +
 .../2026-07-03_15-17-52/model_12750.pt             |    3 +
 .../2026-07-03_15-17-52/model_12800.pt             |    3 +
 .../2026-07-03_15-17-52/model_12850.pt             |    3 +
 .../2026-07-03_15-17-52/model_12900.pt             |    3 +
 .../2026-07-03_15-17-52/model_12950.pt             |    3 +
 .../2026-07-03_15-17-52/model_1300.pt              |    3 +
 .../2026-07-03_15-17-52/model_13000.pt             |    3 +
 .../2026-07-03_15-17-52/model_13050.pt             |    3 +
 .../2026-07-03_15-17-52/model_13100.pt             |    3 +
 .../2026-07-03_15-17-52/model_13150.pt             |    3 +
 .../2026-07-03_15-17-52/model_13200.pt             |    3 +
 .../2026-07-03_15-17-52/model_13250.pt             |    3 +
 .../2026-07-03_15-17-52/model_13300.pt             |    3 +
 .../2026-07-03_15-17-52/model_13350.pt             |    3 +
 .../2026-07-03_15-17-52/model_13400.pt             |    3 +
 .../2026-07-03_15-17-52/model_13450.pt             |    3 +
 .../2026-07-03_15-17-52/model_1350.pt              |    3 +
 .../2026-07-03_15-17-52/model_13500.pt             |    3 +
 .../2026-07-03_15-17-52/model_13550.pt             |    3 +
 .../2026-07-03_15-17-52/model_13600.pt             |    3 +
 .../2026-07-03_15-17-52/model_13650.pt             |    3 +
 .../2026-07-03_15-17-52/model_13700.pt             |    3 +
 .../2026-07-03_15-17-52/model_13750.pt             |    3 +
 .../2026-07-03_15-17-52/model_13800.pt             |    3 +
 .../2026-07-03_15-17-52/model_13850.pt             |    3 +
 .../2026-07-03_15-17-52/model_13900.pt             |    3 +
 .../2026-07-03_15-17-52/model_13950.pt             |    3 +
 .../2026-07-03_15-17-52/model_1400.pt              |    3 +
 .../2026-07-03_15-17-52/model_14000.pt             |    3 +
 .../2026-07-03_15-17-52/model_14050.pt             |    3 +
 .../2026-07-03_15-17-52/model_14100.pt             |    3 +
 .../2026-07-03_15-17-52/model_14150.pt             |    3 +
 .../2026-07-03_15-17-52/model_14200.pt             |    3 +
 .../2026-07-03_15-17-52/model_14250.pt             |    3 +
 .../2026-07-03_15-17-52/model_14300.pt             |    3 +
 .../2026-07-03_15-17-52/model_14350.pt             |    3 +
 .../2026-07-03_15-17-52/model_14400.pt             |    3 +
 .../2026-07-03_15-17-52/model_14450.pt             |    3 +
 .../2026-07-03_15-17-52/model_1450.pt              |    3 +
 .../2026-07-03_15-17-52/model_14500.pt             |    3 +
 .../2026-07-03_15-17-52/model_14550.pt             |    3 +
 .../2026-07-03_15-17-52/model_14600.pt             |    3 +
 .../2026-07-03_15-17-52/model_14650.pt             |    3 +
 .../2026-07-03_15-17-52/model_14700.pt             |    3 +
 .../2026-07-03_15-17-52/model_14750.pt             |    3 +
 .../2026-07-03_15-17-52/model_14800.pt             |    3 +
 .../2026-07-03_15-17-52/model_14850.pt             |    3 +
 .../2026-07-03_15-17-52/model_14900.pt             |    3 +
 .../2026-07-03_15-17-52/model_14950.pt             |    3 +
 .../2026-07-03_15-17-52/model_150.pt               |    3 +
 .../2026-07-03_15-17-52/model_1500.pt              |    3 +
 .../2026-07-03_15-17-52/model_15000.pt             |    3 +
 .../2026-07-03_15-17-52/model_15050.pt             |    3 +
 .../2026-07-03_15-17-52/model_15100.pt             |    3 +
 .../2026-07-03_15-17-52/model_15150.pt             |    3 +
 .../2026-07-03_15-17-52/model_15200.pt             |    3 +
 .../2026-07-03_15-17-52/model_15250.pt             |    3 +
 .../2026-07-03_15-17-52/model_15300.pt             |    3 +
 .../2026-07-03_15-17-52/model_15350.pt             |    3 +
 .../2026-07-03_15-17-52/model_15400.pt             |    3 +
 .../2026-07-03_15-17-52/model_15450.pt             |    3 +
 .../2026-07-03_15-17-52/model_1550.pt              |    3 +
 .../2026-07-03_15-17-52/model_1600.pt              |    3 +
 .../2026-07-03_15-17-52/model_1650.pt              |    3 +
 .../2026-07-03_15-17-52/model_1700.pt              |    3 +
 .../2026-07-03_15-17-52/model_1750.pt              |    3 +
 .../2026-07-03_15-17-52/model_1800.pt              |    3 +
 .../2026-07-03_15-17-52/model_1850.pt              |    3 +
 .../2026-07-03_15-17-52/model_1900.pt              |    3 +
 .../2026-07-03_15-17-52/model_1950.pt              |    3 +
 .../2026-07-03_15-17-52/model_200.pt               |    3 +
 .../2026-07-03_15-17-52/model_2000.pt              |    3 +
 .../2026-07-03_15-17-52/model_2050.pt              |    3 +
 .../2026-07-03_15-17-52/model_2100.pt              |    3 +
 .../2026-07-03_15-17-52/model_2150.pt              |    3 +
 .../2026-07-03_15-17-52/model_2200.pt              |    3 +
 .../2026-07-03_15-17-52/model_2250.pt              |    3 +
 .../2026-07-03_15-17-52/model_2300.pt              |    3 +
 .../2026-07-03_15-17-52/model_2350.pt              |    3 +
 .../2026-07-03_15-17-52/model_2400.pt              |    3 +
 .../2026-07-03_15-17-52/model_2450.pt              |    3 +
 .../2026-07-03_15-17-52/model_250.pt               |    3 +
 .../2026-07-03_15-17-52/model_2500.pt              |    3 +
 .../2026-07-03_15-17-52/model_2550.pt              |    3 +
 .../2026-07-03_15-17-52/model_2600.pt              |    3 +
 .../2026-07-03_15-17-52/model_2650.pt              |    3 +
 .../2026-07-03_15-17-52/model_2700.pt              |    3 +
 .../2026-07-03_15-17-52/model_2750.pt              |    3 +
 .../2026-07-03_15-17-52/model_2800.pt              |    3 +
 .../2026-07-03_15-17-52/model_2850.pt              |    3 +
 .../2026-07-03_15-17-52/model_2900.pt              |    3 +
 .../2026-07-03_15-17-52/model_2950.pt              |    3 +
 .../2026-07-03_15-17-52/model_300.pt               |    3 +
 .../2026-07-03_15-17-52/model_3000.pt              |    3 +
 .../2026-07-03_15-17-52/model_3050.pt              |    3 +
 .../2026-07-03_15-17-52/model_3100.pt              |    3 +
 .../2026-07-03_15-17-52/model_3150.pt              |    3 +
 .../2026-07-03_15-17-52/model_3200.pt              |    3 +
 .../2026-07-03_15-17-52/model_3250.pt              |    3 +
 .../2026-07-03_15-17-52/model_3300.pt              |    3 +
 .../2026-07-03_15-17-52/model_3350.pt              |    3 +
 .../2026-07-03_15-17-52/model_3400.pt              |    3 +
 .../2026-07-03_15-17-52/model_3450.pt              |    3 +
 .../2026-07-03_15-17-52/model_350.pt               |    3 +
 .../2026-07-03_15-17-52/model_3500.pt              |    3 +
 .../2026-07-03_15-17-52/model_3550.pt              |    3 +
 .../2026-07-03_15-17-52/model_3600.pt              |    3 +
 .../2026-07-03_15-17-52/model_3650.pt              |    3 +
 .../2026-07-03_15-17-52/model_3700.pt              |    3 +
 .../2026-07-03_15-17-52/model_3750.pt              |    3 +
 .../2026-07-03_15-17-52/model_3800.pt              |    3 +
 .../2026-07-03_15-17-52/model_3850.pt              |    3 +
 .../2026-07-03_15-17-52/model_3900.pt              |    3 +
 .../2026-07-03_15-17-52/model_3950.pt              |    3 +
 .../2026-07-03_15-17-52/model_400.pt               |    3 +
 .../2026-07-03_15-17-52/model_4000.pt              |    3 +
 .../2026-07-03_15-17-52/model_4050.pt              |    3 +
 .../2026-07-03_15-17-52/model_4100.pt              |    3 +
 .../2026-07-03_15-17-52/model_4150.pt              |    3 +
 .../2026-07-03_15-17-52/model_4200.pt              |    3 +
 .../2026-07-03_15-17-52/model_4250.pt              |    3 +
 .../2026-07-03_15-17-52/model_4300.pt              |    3 +
 .../2026-07-03_15-17-52/model_4350.pt              |    3 +
 .../2026-07-03_15-17-52/model_4400.pt              |    3 +
 .../2026-07-03_15-17-52/model_4450.pt              |    3 +
 .../2026-07-03_15-17-52/model_450.pt               |    3 +
 .../2026-07-03_15-17-52/model_4500.pt              |    3 +
 .../2026-07-03_15-17-52/model_4550.pt              |    3 +
 .../2026-07-03_15-17-52/model_4600.pt              |    3 +
 .../2026-07-03_15-17-52/model_4650.pt              |    3 +
 .../2026-07-03_15-17-52/model_4700.pt              |    3 +
 .../2026-07-03_15-17-52/model_4750.pt              |    3 +
 .../2026-07-03_15-17-52/model_4800.pt              |    3 +
 .../2026-07-03_15-17-52/model_4850.pt              |    3 +
 .../2026-07-03_15-17-52/model_4900.pt              |    3 +
 .../2026-07-03_15-17-52/model_4950.pt              |    3 +
 .../2026-07-03_15-17-52/model_50.pt                |    3 +
 .../2026-07-03_15-17-52/model_500.pt               |    3 +
 .../2026-07-03_15-17-52/model_5000.pt              |    3 +
 .../2026-07-03_15-17-52/model_5050.pt              |    3 +
 .../2026-07-03_15-17-52/model_5100.pt              |    3 +
 .../2026-07-03_15-17-52/model_5150.pt              |    3 +
 .../2026-07-03_15-17-52/model_5200.pt              |    3 +
 .../2026-07-03_15-17-52/model_5250.pt              |    3 +
 .../2026-07-03_15-17-52/model_5300.pt              |    3 +
 .../2026-07-03_15-17-52/model_5350.pt              |    3 +
 .../2026-07-03_15-17-52/model_5400.pt              |    3 +
 .../2026-07-03_15-17-52/model_5450.pt              |    3 +
 .../2026-07-03_15-17-52/model_550.pt               |    3 +
 .../2026-07-03_15-17-52/model_5500.pt              |    3 +
 .../2026-07-03_15-17-52/model_5550.pt              |    3 +
 .../2026-07-03_15-17-52/model_5600.pt              |    3 +
 .../2026-07-03_15-17-52/model_5650.pt              |    3 +
 .../2026-07-03_15-17-52/model_5700.pt              |    3 +
 .../2026-07-03_15-17-52/model_5750.pt              |    3 +
 .../2026-07-03_15-17-52/model_5800.pt              |    3 +
 .../2026-07-03_15-17-52/model_5850.pt              |    3 +
 .../2026-07-03_15-17-52/model_5900.pt              |    3 +
 .../2026-07-03_15-17-52/model_5950.pt              |    3 +
 .../2026-07-03_15-17-52/model_600.pt               |    3 +
 .../2026-07-03_15-17-52/model_6000.pt              |    3 +
 .../2026-07-03_15-17-52/model_6050.pt              |    3 +
 .../2026-07-03_15-17-52/model_6100.pt              |    3 +
 .../2026-07-03_15-17-52/model_6150.pt              |    3 +
 .../2026-07-03_15-17-52/model_6200.pt              |    3 +
 .../2026-07-03_15-17-52/model_6250.pt              |    3 +
 .../2026-07-03_15-17-52/model_6300.pt              |    3 +
 .../2026-07-03_15-17-52/model_6350.pt              |    3 +
 .../2026-07-03_15-17-52/model_6400.pt              |    3 +
 .../2026-07-03_15-17-52/model_6450.pt              |    3 +
 .../2026-07-03_15-17-52/model_650.pt               |    3 +
 .../2026-07-03_15-17-52/model_6500.pt              |    3 +
 .../2026-07-03_15-17-52/model_6550.pt              |    3 +
 .../2026-07-03_15-17-52/model_6600.pt              |    3 +
 .../2026-07-03_15-17-52/model_6650.pt              |    3 +
 .../2026-07-03_15-17-52/model_6700.pt              |    3 +
 .../2026-07-03_15-17-52/model_6750.pt              |    3 +
 .../2026-07-03_15-17-52/model_6800.pt              |    3 +
 .../2026-07-03_15-17-52/model_6850.pt              |    3 +
 .../2026-07-03_15-17-52/model_6900.pt              |    3 +
 .../2026-07-03_15-17-52/model_6950.pt              |    3 +
 .../2026-07-03_15-17-52/model_700.pt               |    3 +
 .../2026-07-03_15-17-52/model_7000.pt              |    3 +
 .../2026-07-03_15-17-52/model_7050.pt              |    3 +
 .../2026-07-03_15-17-52/model_7100.pt              |    3 +
 .../2026-07-03_15-17-52/model_7150.pt              |    3 +
 .../2026-07-03_15-17-52/model_7200.pt              |    3 +
 .../2026-07-03_15-17-52/model_7250.pt              |    3 +
 .../2026-07-03_15-17-52/model_7300.pt              |    3 +
 .../2026-07-03_15-17-52/model_7350.pt              |    3 +
 .../2026-07-03_15-17-52/model_7400.pt              |    3 +
 .../2026-07-03_15-17-52/model_7450.pt              |    3 +
 .../2026-07-03_15-17-52/model_750.pt               |    3 +
 .../2026-07-03_15-17-52/model_7500.pt              |    3 +
 .../2026-07-03_15-17-52/model_7550.pt              |    3 +
 .../2026-07-03_15-17-52/model_7600.pt              |    3 +
 .../2026-07-03_15-17-52/model_7650.pt              |    3 +
 .../2026-07-03_15-17-52/model_7700.pt              |    3 +
 .../2026-07-03_15-17-52/model_7750.pt              |    3 +
 .../2026-07-03_15-17-52/model_7800.pt              |    3 +
 .../2026-07-03_15-17-52/model_7850.pt              |    3 +
 .../2026-07-03_15-17-52/model_7900.pt              |    3 +
 .../2026-07-03_15-17-52/model_7950.pt              |    3 +
 .../2026-07-03_15-17-52/model_800.pt               |    3 +
 .../2026-07-03_15-17-52/model_8000.pt              |    3 +
 .../2026-07-03_15-17-52/model_8050.pt              |    3 +
 .../2026-07-03_15-17-52/model_8100.pt              |    3 +
 .../2026-07-03_15-17-52/model_8150.pt              |    3 +
 .../2026-07-03_15-17-52/model_8200.pt              |    3 +
 .../2026-07-03_15-17-52/model_8250.pt              |    3 +
 .../2026-07-03_15-17-52/model_8300.pt              |    3 +
 .../2026-07-03_15-17-52/model_8350.pt              |    3 +
 .../2026-07-03_15-17-52/model_8400.pt              |    3 +
 .../2026-07-03_15-17-52/model_8450.pt              |    3 +
 .../2026-07-03_15-17-52/model_850.pt               |    3 +
 .../2026-07-03_15-17-52/model_8500.pt              |    3 +
 .../2026-07-03_15-17-52/model_8550.pt              |    3 +
 .../2026-07-03_15-17-52/model_8600.pt              |    3 +
 .../2026-07-03_15-17-52/model_8650.pt              |    3 +
 .../2026-07-03_15-17-52/model_8700.pt              |    3 +
 .../2026-07-03_15-17-52/model_8750.pt              |    3 +
 .../2026-07-03_15-17-52/model_8800.pt              |    3 +
 .../2026-07-03_15-17-52/model_8850.pt              |    3 +
 .../2026-07-03_15-17-52/model_8900.pt              |    3 +
 .../2026-07-03_15-17-52/model_8950.pt              |    3 +
 .../2026-07-03_15-17-52/model_900.pt               |    3 +
 .../2026-07-03_15-17-52/model_9000.pt              |    3 +
 .../2026-07-03_15-17-52/model_9050.pt              |    3 +
 .../2026-07-03_15-17-52/model_9100.pt              |    3 +
 .../2026-07-03_15-17-52/model_9150.pt              |    3 +
 .../2026-07-03_15-17-52/model_9200.pt              |    3 +
 .../2026-07-03_15-17-52/model_9250.pt              |    3 +
 .../2026-07-03_15-17-52/model_9300.pt              |    3 +
 .../2026-07-03_15-17-52/model_9350.pt              |    3 +
 .../2026-07-03_15-17-52/model_9400.pt              |    3 +
 .../2026-07-03_15-17-52/model_9450.pt              |    3 +
 .../2026-07-03_15-17-52/model_950.pt               |    3 +
 .../2026-07-03_15-17-52/model_9500.pt              |    3 +
 .../2026-07-03_15-17-52/model_9550.pt              |    3 +
 .../2026-07-03_15-17-52/model_9600.pt              |    3 +
 .../2026-07-03_15-17-52/model_9650.pt              |    3 +
 .../2026-07-03_15-17-52/model_9700.pt              |    3 +
 .../2026-07-03_15-17-52/model_9750.pt              |    3 +
 .../2026-07-03_15-17-52/model_9800.pt              |    3 +
 .../2026-07-03_15-17-52/model_9850.pt              |    3 +
 .../2026-07-03_15-17-52/model_9900.pt              |    3 +
 .../2026-07-03_15-17-52/model_9950.pt              |    3 +
 .../2026-07-03_15-17-52/params/agent.pkl           |  Bin 0 -> 1061 bytes
 .../2026-07-03_15-17-52/params/agent.yaml          |   43 +
 .../2026-07-03_15-17-52/params/env.pkl             |  Bin 0 -> 17957 bytes
 .../2026-07-03_15-17-52/params/env.yaml            | 1125 ++++++++++++++
 .../best/exported/policy.onnx                      |  Bin 0 -> 25282 bytes
 .../best/exported/policy.pt                        |    3 +
 .../wheeled_arm_free_locomotion/best/model.pt      |    1 +
 .../wheeled_arm_free_locomotion/best/notes.md      |   25 +
 ...events.out.tfevents.1783088208.moujank.877617.0 |  Bin 0 -> 46334 bytes
 .../2026-07-03_22-16-08/git/wheeled_leg.diff       |  699 +++++++++
 .../2026-07-03_22-16-08/model_0.pt                 |    3 +
 .../2026-07-03_22-16-08/params/agent.pkl           |  Bin 0 -> 1057 bytes
 .../2026-07-03_22-16-08/params/agent.yaml          |   43 +
 .../2026-07-03_22-16-08/params/env.pkl             |  Bin 0 -> 18116 bytes
 .../2026-07-03_22-16-08/params/env.yaml            | 1095 +++++++++++++
 ...events.out.tfevents.1783088302.moujank.879377.0 |  Bin 0 -> 415847 bytes
 .../2026-07-03_22-18-04/git/wheeled_leg.diff       |  699 +++++++++
 .../2026-07-03_22-18-04/model_0.pt                 |    3 +
 .../2026-07-03_22-18-04/model_100.pt               |    3 +
 .../2026-07-03_22-18-04/model_150.pt               |    3 +
 .../2026-07-03_22-18-04/model_200.pt               |    3 +
 .../2026-07-03_22-18-04/model_50.pt                |    3 +
 .../2026-07-03_22-18-04/params/agent.pkl           |  Bin 0 -> 1057 bytes
 .../2026-07-03_22-18-04/params/agent.yaml          |   43 +
 .../2026-07-03_22-18-04/params/env.pkl             |  Bin 0 -> 18116 bytes
 .../2026-07-03_22-18-04/params/env.yaml            | 1095 +++++++++++++
 .../rsl_rl/wheeled_arm_free_obstacle/best/notes.md |   19 +
 ...events.out.tfevents.1783085941.moujank.804219.0 |  Bin 0 -> 494398 bytes
 .../2026-07-03_21-38-43/git/wheeled_leg.diff       |  699 +++++++++
 .../2026-07-03_21-38-43/model_0.pt                 |    3 +
 .../2026-07-03_21-38-43/model_100.pt               |    3 +
 .../2026-07-03_21-38-43/model_150.pt               |    3 +
 .../2026-07-03_21-38-43/model_200.pt               |    3 +
 .../2026-07-03_21-38-43/model_250.pt               |    3 +
 .../2026-07-03_21-38-43/model_300.pt               |    3 +
 .../2026-07-03_21-38-43/model_50.pt                |    3 +
 .../2026-07-03_21-38-43/params/agent.pkl           |  Bin 0 -> 1057 bytes
 .../2026-07-03_21-38-43/params/agent.yaml          |   43 +
 .../2026-07-03_21-38-43/params/env.pkl             |  Bin 0 -> 16395 bytes
 .../2026-07-03_21-38-43/params/env.yaml            |  943 ++++++++++++
 .../rsl_rl/wheeled_arm_free_recovery/best/notes.md |   19 +
 ...vents.out.tfevents.1782794419.moujank.1026075.0 |  Bin 0 -> 9753852 bytes
 .../2026-06-30_12-40-10/git/wheeled_leg.diff       |  840 ++++++++++
 .../2026-06-30_12-40-10/model_0.pt                 |    3 +
 .../2026-06-30_12-40-10/model_100.pt               |    3 +
 .../2026-06-30_12-40-10/model_1000.pt              |    3 +
 .../2026-06-30_12-40-10/model_1050.pt              |    3 +
 .../2026-06-30_12-40-10/model_1100.pt              |    3 +
 .../2026-06-30_12-40-10/model_1150.pt              |    3 +
 .../2026-06-30_12-40-10/model_1200.pt              |    3 +
 .../2026-06-30_12-40-10/model_1250.pt              |    3 +
 .../2026-06-30_12-40-10/model_1300.pt              |    3 +
 .../2026-06-30_12-40-10/model_1350.pt              |    3 +
 .../2026-06-30_12-40-10/model_1400.pt              |    3 +
 .../2026-06-30_12-40-10/model_1450.pt              |    3 +
 .../2026-06-30_12-40-10/model_150.pt               |    3 +
 .../2026-06-30_12-40-10/model_1500.pt              |    3 +
 .../2026-06-30_12-40-10/model_1550.pt              |    3 +
 .../2026-06-30_12-40-10/model_1600.pt              |    3 +
 .../2026-06-30_12-40-10/model_1650.pt              |    3 +
 .../2026-06-30_12-40-10/model_1700.pt              |    3 +
 .../2026-06-30_12-40-10/model_1750.pt              |    3 +
 .../2026-06-30_12-40-10/model_1800.pt              |    3 +
 .../2026-06-30_12-40-10/model_1850.pt              |    3 +
 .../2026-06-30_12-40-10/model_1900.pt              |    3 +
 .../2026-06-30_12-40-10/model_1950.pt              |    3 +
 .../2026-06-30_12-40-10/model_200.pt               |    3 +
 .../2026-06-30_12-40-10/model_2000.pt              |    3 +
 .../2026-06-30_12-40-10/model_2050.pt              |    3 +
 .../2026-06-30_12-40-10/model_2100.pt              |    3 +
 .../2026-06-30_12-40-10/model_2150.pt              |    3 +
 .../2026-06-30_12-40-10/model_2200.pt              |    3 +
 .../2026-06-30_12-40-10/model_2250.pt              |    3 +
 .../2026-06-30_12-40-10/model_2300.pt              |    3 +
 .../2026-06-30_12-40-10/model_2350.pt              |    3 +
 .../2026-06-30_12-40-10/model_2400.pt              |    3 +
 .../2026-06-30_12-40-10/model_2450.pt              |    3 +
 .../2026-06-30_12-40-10/model_250.pt               |    3 +
 .../2026-06-30_12-40-10/model_2500.pt              |    3 +
 .../2026-06-30_12-40-10/model_2550.pt              |    3 +
 .../2026-06-30_12-40-10/model_2600.pt              |    3 +
 .../2026-06-30_12-40-10/model_2650.pt              |    3 +
 .../2026-06-30_12-40-10/model_2700.pt              |    3 +
 .../2026-06-30_12-40-10/model_2750.pt              |    3 +
 .../2026-06-30_12-40-10/model_2800.pt              |    3 +
 .../2026-06-30_12-40-10/model_2850.pt              |    3 +
 .../2026-06-30_12-40-10/model_2900.pt              |    3 +
 .../2026-06-30_12-40-10/model_2950.pt              |    3 +
 .../2026-06-30_12-40-10/model_300.pt               |    3 +
 .../2026-06-30_12-40-10/model_3000.pt              |    3 +
 .../2026-06-30_12-40-10/model_3050.pt              |    3 +
 .../2026-06-30_12-40-10/model_3100.pt              |    3 +
 .../2026-06-30_12-40-10/model_3150.pt              |    3 +
 .../2026-06-30_12-40-10/model_3200.pt              |    3 +
 .../2026-06-30_12-40-10/model_3250.pt              |    3 +
 .../2026-06-30_12-40-10/model_3300.pt              |    3 +
 .../2026-06-30_12-40-10/model_3350.pt              |    3 +
 .../2026-06-30_12-40-10/model_3400.pt              |    3 +
 .../2026-06-30_12-40-10/model_3450.pt              |    3 +
 .../2026-06-30_12-40-10/model_350.pt               |    3 +
 .../2026-06-30_12-40-10/model_3500.pt              |    3 +
 .../2026-06-30_12-40-10/model_3550.pt              |    3 +
 .../2026-06-30_12-40-10/model_3600.pt              |    3 +
 .../2026-06-30_12-40-10/model_3650.pt              |    3 +
 .../2026-06-30_12-40-10/model_3700.pt              |    3 +
 .../2026-06-30_12-40-10/model_3750.pt              |    3 +
 .../2026-06-30_12-40-10/model_3800.pt              |    3 +
 .../2026-06-30_12-40-10/model_3850.pt              |    3 +
 .../2026-06-30_12-40-10/model_3900.pt              |    3 +
 .../2026-06-30_12-40-10/model_3950.pt              |    3 +
 .../2026-06-30_12-40-10/model_400.pt               |    3 +
 .../2026-06-30_12-40-10/model_4000.pt              |    3 +
 .../2026-06-30_12-40-10/model_4050.pt              |    3 +
 .../2026-06-30_12-40-10/model_4100.pt              |    3 +
 .../2026-06-30_12-40-10/model_4150.pt              |    3 +
 .../2026-06-30_12-40-10/model_4200.pt              |    3 +
 .../2026-06-30_12-40-10/model_4250.pt              |    3 +
 .../2026-06-30_12-40-10/model_4300.pt              |    3 +
 .../2026-06-30_12-40-10/model_4350.pt              |    3 +
 .../2026-06-30_12-40-10/model_4400.pt              |    3 +
 .../2026-06-30_12-40-10/model_4450.pt              |    3 +
 .../2026-06-30_12-40-10/model_450.pt               |    3 +
 .../2026-06-30_12-40-10/model_4500.pt              |    3 +
 .../2026-06-30_12-40-10/model_4550.pt              |    3 +
 .../2026-06-30_12-40-10/model_4600.pt              |    3 +
 .../2026-06-30_12-40-10/model_4650.pt              |    3 +
 .../2026-06-30_12-40-10/model_4700.pt              |    3 +
 .../2026-06-30_12-40-10/model_4750.pt              |    3 +
 .../2026-06-30_12-40-10/model_4800.pt              |    3 +
 .../2026-06-30_12-40-10/model_4850.pt              |    3 +
 .../2026-06-30_12-40-10/model_4900.pt              |    3 +
 .../2026-06-30_12-40-10/model_4950.pt              |    3 +
 .../2026-06-30_12-40-10/model_50.pt                |    3 +
 .../2026-06-30_12-40-10/model_500.pt               |    3 +
 .../2026-06-30_12-40-10/model_5000.pt              |    3 +
 .../2026-06-30_12-40-10/model_5050.pt              |    3 +
 .../2026-06-30_12-40-10/model_5100.pt              |    3 +
 .../2026-06-30_12-40-10/model_5150.pt              |    3 +
 .../2026-06-30_12-40-10/model_5200.pt              |    3 +
 .../2026-06-30_12-40-10/model_5250.pt              |    3 +
 .../2026-06-30_12-40-10/model_5300.pt              |    3 +
 .../2026-06-30_12-40-10/model_5350.pt              |    3 +
 .../2026-06-30_12-40-10/model_550.pt               |    3 +
 .../2026-06-30_12-40-10/model_600.pt               |    3 +
 .../2026-06-30_12-40-10/model_650.pt               |    3 +
 .../2026-06-30_12-40-10/model_700.pt               |    3 +
 .../2026-06-30_12-40-10/model_750.pt               |    3 +
 .../2026-06-30_12-40-10/model_800.pt               |    3 +
 .../2026-06-30_12-40-10/model_850.pt               |    3 +
 .../2026-06-30_12-40-10/model_900.pt               |    3 +
 .../2026-06-30_12-40-10/model_950.pt               |    3 +
 .../2026-06-30_12-40-10/params/agent.pkl           |  Bin 0 -> 1032 bytes
 .../2026-06-30_12-40-10/params/agent.yaml          |   43 +
 .../2026-06-30_12-40-10/params/env.pkl             |  Bin 0 -> 16613 bytes
 .../2026-06-30_12-40-10/params/env.yaml            |  981 ++++++++++++
 .../events.out.tfevents.1783043311.moujank.25758.0 |  Bin 0 -> 44711 bytes
 .../2026-07-03_09-48-21/git/wheeled_leg.diff       |  699 +++++++++
 .../2026-07-03_09-48-21/model_0.pt                 |    3 +
 .../2026-07-03_09-48-21/params/agent.pkl           |  Bin 0 -> 1044 bytes
 .../2026-07-03_09-48-21/params/agent.yaml          |   43 +
 .../2026-07-03_09-48-21/params/env.pkl             |  Bin 0 -> 16601 bytes
 .../2026-07-03_09-48-21/params/env.yaml            |  981 ++++++++++++
 .../events.out.tfevents.1783043373.moujank.26168.0 |  Bin 0 -> 26841 bytes
 .../2026-07-03_09-49-24/git/wheeled_leg.diff       |  699 +++++++++
 .../2026-07-03_09-49-24/model_0.pt                 |    3 +
 .../2026-07-03_09-49-24/params/agent.pkl           |  Bin 0 -> 1044 bytes
 .../2026-07-03_09-49-24/params/agent.yaml          |   43 +
 .../2026-07-03_09-49-24/params/env.pkl             |  Bin 0 -> 16601 bytes
 .../2026-07-03_09-49-24/params/env.yaml            |  981 ++++++++++++
 .../events.out.tfevents.1783043476.moujank.26600.0 |  Bin 0 -> 30415 bytes
 .../2026-07-03_09-51-07/git/wheeled_leg.diff       |  699 +++++++++
 .../2026-07-03_09-51-07/model_0.pt                 |    3 +
 .../2026-07-03_09-51-07/params/agent.pkl           |  Bin 0 -> 1044 bytes
 .../2026-07-03_09-51-07/params/agent.yaml          |   43 +
 .../2026-07-03_09-51-07/params/env.pkl             |  Bin 0 -> 16601 bytes
 .../2026-07-03_09-51-07/params/env.yaml            |  981 ++++++++++++
 .../events.out.tfevents.1783043556.moujank.26984.0 |  Bin 0 -> 19264484 bytes
 .../2026-07-03_09-52-21/git/wheeled_leg.diff       |  699 +++++++++
 .../2026-07-03_09-52-21/model_0.pt                 |    3 +
 .../2026-07-03_09-52-21/model_100.pt               |    3 +
 .../2026-07-03_09-52-21/model_1000.pt              |    3 +
 .../2026-07-03_09-52-21/model_10000.pt             |    3 +
 .../2026-07-03_09-52-21/model_10050.pt             |    3 +
 .../2026-07-03_09-52-21/model_10100.pt             |    3 +
 .../2026-07-03_09-52-21/model_10150.pt             |    3 +
 .../2026-07-03_09-52-21/model_10200.pt             |    3 +
 .../2026-07-03_09-52-21/model_10250.pt             |    3 +
 .../2026-07-03_09-52-21/model_10300.pt             |    3 +
 .../2026-07-03_09-52-21/model_10350.pt             |    3 +
 .../2026-07-03_09-52-21/model_10400.pt             |    3 +
 .../2026-07-03_09-52-21/model_10450.pt             |    3 +
 .../2026-07-03_09-52-21/model_1050.pt              |    3 +
 .../2026-07-03_09-52-21/model_10500.pt             |    3 +
 .../2026-07-03_09-52-21/model_10550.pt             |    3 +
 .../2026-07-03_09-52-21/model_10600.pt             |    3 +
 .../2026-07-03_09-52-21/model_1100.pt              |    3 +
 .../2026-07-03_09-52-21/model_1150.pt              |    3 +
 .../2026-07-03_09-52-21/model_1200.pt              |    3 +
 .../2026-07-03_09-52-21/model_1250.pt              |    3 +
 .../2026-07-03_09-52-21/model_1300.pt              |    3 +
 .../2026-07-03_09-52-21/model_1350.pt              |    3 +
 .../2026-07-03_09-52-21/model_1400.pt              |    3 +
 .../2026-07-03_09-52-21/model_1450.pt              |    3 +
 .../2026-07-03_09-52-21/model_150.pt               |    3 +
 .../2026-07-03_09-52-21/model_1500.pt              |    3 +
 .../2026-07-03_09-52-21/model_1550.pt              |    3 +
 .../2026-07-03_09-52-21/model_1600.pt              |    3 +
 .../2026-07-03_09-52-21/model_1650.pt              |    3 +
 .../2026-07-03_09-52-21/model_1700.pt              |    3 +
 .../2026-07-03_09-52-21/model_1750.pt              |    3 +
 .../2026-07-03_09-52-21/model_1800.pt              |    3 +
 .../2026-07-03_09-52-21/model_1850.pt              |    3 +
 .../2026-07-03_09-52-21/model_1900.pt              |    3 +
 .../2026-07-03_09-52-21/model_1950.pt              |    3 +
 .../2026-07-03_09-52-21/model_200.pt               |    3 +
 .../2026-07-03_09-52-21/model_2000.pt              |    3 +
 .../2026-07-03_09-52-21/model_2050.pt              |    3 +
 .../2026-07-03_09-52-21/model_2100.pt              |    3 +
 .../2026-07-03_09-52-21/model_2150.pt              |    3 +
 .../2026-07-03_09-52-21/model_2200.pt              |    3 +
 .../2026-07-03_09-52-21/model_2250.pt              |    3 +
 .../2026-07-03_09-52-21/model_2300.pt              |    3 +
 .../2026-07-03_09-52-21/model_2350.pt              |    3 +
 .../2026-07-03_09-52-21/model_2400.pt              |    3 +
 .../2026-07-03_09-52-21/model_2450.pt              |    3 +
 .../2026-07-03_09-52-21/model_250.pt               |    3 +
 .../2026-07-03_09-52-21/model_2500.pt              |    3 +
 .../2026-07-03_09-52-21/model_2550.pt              |    3 +
 .../2026-07-03_09-52-21/model_2600.pt              |    3 +
 .../2026-07-03_09-52-21/model_2650.pt              |    3 +
 .../2026-07-03_09-52-21/model_2700.pt              |    3 +
 .../2026-07-03_09-52-21/model_2750.pt              |    3 +
 .../2026-07-03_09-52-21/model_2800.pt              |    3 +
 .../2026-07-03_09-52-21/model_2850.pt              |    3 +
 .../2026-07-03_09-52-21/model_2900.pt              |    3 +
 .../2026-07-03_09-52-21/model_2950.pt              |    3 +
 .../2026-07-03_09-52-21/model_300.pt               |    3 +
 .../2026-07-03_09-52-21/model_3000.pt              |    3 +
 .../2026-07-03_09-52-21/model_3050.pt              |    3 +
 .../2026-07-03_09-52-21/model_3100.pt              |    3 +
 .../2026-07-03_09-52-21/model_3150.pt              |    3 +
 .../2026-07-03_09-52-21/model_3200.pt              |    3 +
 .../2026-07-03_09-52-21/model_3250.pt              |    3 +
 .../2026-07-03_09-52-21/model_3300.pt              |    3 +
 .../2026-07-03_09-52-21/model_3350.pt              |    3 +
 .../2026-07-03_09-52-21/model_3400.pt              |    3 +
 .../2026-07-03_09-52-21/model_3450.pt              |    3 +
 .../2026-07-03_09-52-21/model_350.pt               |    3 +
 .../2026-07-03_09-52-21/model_3500.pt              |    3 +
 .../2026-07-03_09-52-21/model_3550.pt              |    3 +
 .../2026-07-03_09-52-21/model_3600.pt              |    3 +
 .../2026-07-03_09-52-21/model_3650.pt              |    3 +
 .../2026-07-03_09-52-21/model_3700.pt              |    3 +
 .../2026-07-03_09-52-21/model_3750.pt              |    3 +
 .../2026-07-03_09-52-21/model_3800.pt              |    3 +
 .../2026-07-03_09-52-21/model_3850.pt              |    3 +
 .../2026-07-03_09-52-21/model_3900.pt              |    3 +
 .../2026-07-03_09-52-21/model_3950.pt              |    3 +
 .../2026-07-03_09-52-21/model_400.pt               |    3 +
 .../2026-07-03_09-52-21/model_4000.pt              |    3 +
 .../2026-07-03_09-52-21/model_4050.pt              |    3 +
 .../2026-07-03_09-52-21/model_4100.pt              |    3 +
 .../2026-07-03_09-52-21/model_4150.pt              |    3 +
 .../2026-07-03_09-52-21/model_4200.pt              |    3 +
 .../2026-07-03_09-52-21/model_4250.pt              |    3 +
 .../2026-07-03_09-52-21/model_4300.pt              |    3 +
 .../2026-07-03_09-52-21/model_4350.pt              |    3 +
 .../2026-07-03_09-52-21/model_4400.pt              |    3 +
 .../2026-07-03_09-52-21/model_4450.pt              |    3 +
 .../2026-07-03_09-52-21/model_450.pt               |    3 +
 .../2026-07-03_09-52-21/model_4500.pt              |    3 +
 .../2026-07-03_09-52-21/model_4550.pt              |    3 +
 .../2026-07-03_09-52-21/model_4600.pt              |    3 +
 .../2026-07-03_09-52-21/model_4650.pt              |    3 +
 .../2026-07-03_09-52-21/model_4700.pt              |    3 +
 .../2026-07-03_09-52-21/model_4750.pt              |    3 +
 .../2026-07-03_09-52-21/model_4800.pt              |    3 +
 .../2026-07-03_09-52-21/model_4850.pt              |    3 +
 .../2026-07-03_09-52-21/model_4900.pt              |    3 +
 .../2026-07-03_09-52-21/model_4950.pt              |    3 +
 .../2026-07-03_09-52-21/model_50.pt                |    3 +
 .../2026-07-03_09-52-21/model_500.pt               |    3 +
 .../2026-07-03_09-52-21/model_5000.pt              |    3 +
 .../2026-07-03_09-52-21/model_5050.pt              |    3 +
 .../2026-07-03_09-52-21/model_5100.pt              |    3 +
 .../2026-07-03_09-52-21/model_5150.pt              |    3 +
 .../2026-07-03_09-52-21/model_5200.pt              |    3 +
 .../2026-07-03_09-52-21/model_5250.pt              |    3 +
 .../2026-07-03_09-52-21/model_5300.pt              |    3 +
 .../2026-07-03_09-52-21/model_5350.pt              |    3 +
 .../2026-07-03_09-52-21/model_5400.pt              |    3 +
 .../2026-07-03_09-52-21/model_5450.pt              |    3 +
 .../2026-07-03_09-52-21/model_550.pt               |    3 +
 .../2026-07-03_09-52-21/model_5500.pt              |    3 +
 .../2026-07-03_09-52-21/model_5550.pt              |    3 +
 .../2026-07-03_09-52-21/model_5600.pt              |    3 +
 .../2026-07-03_09-52-21/model_5650.pt              |    3 +
 .../2026-07-03_09-52-21/model_5700.pt              |    3 +
 .../2026-07-03_09-52-21/model_5750.pt              |    3 +
 .../2026-07-03_09-52-21/model_5800.pt              |    3 +
 .../2026-07-03_09-52-21/model_5850.pt              |    3 +
 .../2026-07-03_09-52-21/model_5900.pt              |    3 +
 .../2026-07-03_09-52-21/model_5950.pt              |    3 +
 .../2026-07-03_09-52-21/model_600.pt               |    3 +
 .../2026-07-03_09-52-21/model_6000.pt              |    3 +
 .../2026-07-03_09-52-21/model_6050.pt              |    3 +
 .../2026-07-03_09-52-21/model_6100.pt              |    3 +
 .../2026-07-03_09-52-21/model_6150.pt              |    3 +
 .../2026-07-03_09-52-21/model_6200.pt              |    3 +
 .../2026-07-03_09-52-21/model_6250.pt              |    3 +
 .../2026-07-03_09-52-21/model_6300.pt              |    3 +
 .../2026-07-03_09-52-21/model_6350.pt              |    3 +
 .../2026-07-03_09-52-21/model_6400.pt              |    3 +
 .../2026-07-03_09-52-21/model_6450.pt              |    3 +
 .../2026-07-03_09-52-21/model_650.pt               |    3 +
 .../2026-07-03_09-52-21/model_6500.pt              |    3 +
 .../2026-07-03_09-52-21/model_6550.pt              |    3 +
 .../2026-07-03_09-52-21/model_6600.pt              |    3 +
 .../2026-07-03_09-52-21/model_6650.pt              |    3 +
 .../2026-07-03_09-52-21/model_6700.pt              |    3 +
 .../2026-07-03_09-52-21/model_6750.pt              |    3 +
 .../2026-07-03_09-52-21/model_6800.pt              |    3 +
 .../2026-07-03_09-52-21/model_6850.pt              |    3 +
 .../2026-07-03_09-52-21/model_6900.pt              |    3 +
 .../2026-07-03_09-52-21/model_6950.pt              |    3 +
 .../2026-07-03_09-52-21/model_700.pt               |    3 +
 .../2026-07-03_09-52-21/model_7000.pt              |    3 +
 .../2026-07-03_09-52-21/model_7050.pt              |    3 +
 .../2026-07-03_09-52-21/model_7100.pt              |    3 +
 .../2026-07-03_09-52-21/model_7150.pt              |    3 +
 .../2026-07-03_09-52-21/model_7200.pt              |    3 +
 .../2026-07-03_09-52-21/model_7250.pt              |    3 +
 .../2026-07-03_09-52-21/model_7300.pt              |    3 +
 .../2026-07-03_09-52-21/model_7350.pt              |    3 +
 .../2026-07-03_09-52-21/model_7400.pt              |    3 +
 .../2026-07-03_09-52-21/model_7450.pt              |    3 +
 .../2026-07-03_09-52-21/model_750.pt               |    3 +
 .../2026-07-03_09-52-21/model_7500.pt              |    3 +
 .../2026-07-03_09-52-21/model_7550.pt              |    3 +
 .../2026-07-03_09-52-21/model_7600.pt              |    3 +
 .../2026-07-03_09-52-21/model_7650.pt              |    3 +
 .../2026-07-03_09-52-21/model_7700.pt              |    3 +
 .../2026-07-03_09-52-21/model_7750.pt              |    3 +
 .../2026-07-03_09-52-21/model_7800.pt              |    3 +
 .../2026-07-03_09-52-21/model_7850.pt              |    3 +
 .../2026-07-03_09-52-21/model_7900.pt              |    3 +
 .../2026-07-03_09-52-21/model_7950.pt              |    3 +
 .../2026-07-03_09-52-21/model_800.pt               |    3 +
 .../2026-07-03_09-52-21/model_8000.pt              |    3 +
 .../2026-07-03_09-52-21/model_8050.pt              |    3 +
 .../2026-07-03_09-52-21/model_8100.pt              |    3 +
 .../2026-07-03_09-52-21/model_8150.pt              |    3 +
 .../2026-07-03_09-52-21/model_8200.pt              |    3 +
 .../2026-07-03_09-52-21/model_8250.pt              |    3 +
 .../2026-07-03_09-52-21/model_8300.pt              |    3 +
 .../2026-07-03_09-52-21/model_8350.pt              |    3 +
 .../2026-07-03_09-52-21/model_8400.pt              |    3 +
 .../2026-07-03_09-52-21/model_8450.pt              |    3 +
 .../2026-07-03_09-52-21/model_850.pt               |    3 +
 .../2026-07-03_09-52-21/model_8500.pt              |    3 +
 .../2026-07-03_09-52-21/model_8550.pt              |    3 +
 .../2026-07-03_09-52-21/model_8600.pt              |    3 +
 .../2026-07-03_09-52-21/model_8650.pt              |    3 +
 .../2026-07-03_09-52-21/model_8700.pt              |    3 +
 .../2026-07-03_09-52-21/model_8750.pt              |    3 +
 .../2026-07-03_09-52-21/model_8800.pt              |    3 +
 .../2026-07-03_09-52-21/model_8850.pt              |    3 +
 .../2026-07-03_09-52-21/model_8900.pt              |    3 +
 .../2026-07-03_09-52-21/model_8950.pt              |    3 +
 .../2026-07-03_09-52-21/model_900.pt               |    3 +
 .../2026-07-03_09-52-21/model_9000.pt              |    3 +
 .../2026-07-03_09-52-21/model_9050.pt              |    3 +
 .../2026-07-03_09-52-21/model_9100.pt              |    3 +
 .../2026-07-03_09-52-21/model_9150.pt              |    3 +
 .../2026-07-03_09-52-21/model_9200.pt              |    3 +
 .../2026-07-03_09-52-21/model_9250.pt              |    3 +
 .../2026-07-03_09-52-21/model_9300.pt              |    3 +
 .../2026-07-03_09-52-21/model_9350.pt              |    3 +
 .../2026-07-03_09-52-21/model_9400.pt              |    3 +
 .../2026-07-03_09-52-21/model_9450.pt              |    3 +
 .../2026-07-03_09-52-21/model_950.pt               |    3 +
 .../2026-07-03_09-52-21/model_9500.pt              |    3 +
 .../2026-07-03_09-52-21/model_9550.pt              |    3 +
 .../2026-07-03_09-52-21/model_9600.pt              |    3 +
 .../2026-07-03_09-52-21/model_9650.pt              |    3 +
 .../2026-07-03_09-52-21/model_9700.pt              |    3 +
 .../2026-07-03_09-52-21/model_9750.pt              |    3 +
 .../2026-07-03_09-52-21/model_9800.pt              |    3 +
 .../2026-07-03_09-52-21/model_9850.pt              |    3 +
 .../2026-07-03_09-52-21/model_9900.pt              |    3 +
 .../2026-07-03_09-52-21/model_9950.pt              |    3 +
 .../2026-07-03_09-52-21/params/agent.pkl           |  Bin 0 -> 1046 bytes
 .../2026-07-03_09-52-21/params/agent.yaml          |   43 +
 .../2026-07-03_09-52-21/params/env.pkl             |  Bin 0 -> 16601 bytes
 .../2026-07-03_09-52-21/params/env.yaml            |  981 ++++++++++++
 .../wheeled_base_locked/best/exported/policy.onnx  |  Bin 0 -> 9509 bytes
 .../wheeled_base_locked/best/exported/policy.pt    |    3 +
 .../logs/rsl_rl/wheeled_base_locked/best/model.pt  |    1 +
 .../logs/rsl_rl/wheeled_base_locked/best/notes.md  |   25 +
 ...vents.out.tfevents.1782802912.moujank.1252892.0 |  Bin 0 -> 4332467 bytes
 .../2026-06-30_15-01-41/git/wheeled_leg.diff       |  896 +++++++++++
 .../2026-06-30_15-01-41/model_0.pt                 |    3 +
 .../2026-06-30_15-01-41/model_100.pt               |    3 +
 .../2026-06-30_15-01-41/model_1000.pt              |    3 +
 .../2026-06-30_15-01-41/model_1050.pt              |    3 +
 .../2026-06-30_15-01-41/model_1100.pt              |    3 +
 .../2026-06-30_15-01-41/model_1150.pt              |    3 +
 .../2026-06-30_15-01-41/model_1200.pt              |    3 +
 .../2026-06-30_15-01-41/model_1250.pt              |    3 +
 .../2026-06-30_15-01-41/model_1300.pt              |    3 +
 .../2026-06-30_15-01-41/model_1350.pt              |    3 +
 .../2026-06-30_15-01-41/model_1400.pt              |    3 +
 .../2026-06-30_15-01-41/model_1450.pt              |    3 +
 .../2026-06-30_15-01-41/model_150.pt               |    3 +
 .../2026-06-30_15-01-41/model_1500.pt              |    3 +
 .../2026-06-30_15-01-41/model_1550.pt              |    3 +
 .../2026-06-30_15-01-41/model_1600.pt              |    3 +
 .../2026-06-30_15-01-41/model_1650.pt              |    3 +
 .../2026-06-30_15-01-41/model_1700.pt              |    3 +
 .../2026-06-30_15-01-41/model_1750.pt              |    3 +
 .../2026-06-30_15-01-41/model_1800.pt              |    3 +
 .../2026-06-30_15-01-41/model_1850.pt              |    3 +
 .../2026-06-30_15-01-41/model_1900.pt              |    3 +
 .../2026-06-30_15-01-41/model_1950.pt              |    3 +
 .../2026-06-30_15-01-41/model_200.pt               |    3 +
 .../2026-06-30_15-01-41/model_2000.pt              |    3 +
 .../2026-06-30_15-01-41/model_2050.pt              |    3 +
 .../2026-06-30_15-01-41/model_2100.pt              |    3 +
 .../2026-06-30_15-01-41/model_2150.pt              |    3 +
 .../2026-06-30_15-01-41/model_2200.pt              |    3 +
 .../2026-06-30_15-01-41/model_2250.pt              |    3 +
 .../2026-06-30_15-01-41/model_2300.pt              |    3 +
 .../2026-06-30_15-01-41/model_2350.pt              |    3 +
 .../2026-06-30_15-01-41/model_250.pt               |    3 +
 .../2026-06-30_15-01-41/model_300.pt               |    3 +
 .../2026-06-30_15-01-41/model_350.pt               |    3 +
 .../2026-06-30_15-01-41/model_400.pt               |    3 +
 .../2026-06-30_15-01-41/model_450.pt               |    3 +
 .../2026-06-30_15-01-41/model_50.pt                |    3 +
 .../2026-06-30_15-01-41/model_500.pt               |    3 +
 .../2026-06-30_15-01-41/model_550.pt               |    3 +
 .../2026-06-30_15-01-41/model_600.pt               |    3 +
 .../2026-06-30_15-01-41/model_650.pt               |    3 +
 .../2026-06-30_15-01-41/model_700.pt               |    3 +
 .../2026-06-30_15-01-41/model_750.pt               |    3 +
 .../2026-06-30_15-01-41/model_800.pt               |    3 +
 .../2026-06-30_15-01-41/model_850.pt               |    3 +
 .../2026-06-30_15-01-41/model_900.pt               |    3 +
 .../2026-06-30_15-01-41/model_950.pt               |    3 +
 .../2026-06-30_15-01-41/params/agent.pkl           |  Bin 0 -> 1032 bytes
 .../2026-06-30_15-01-41/params/agent.yaml          |   43 +
 .../2026-06-30_15-01-41/params/env.pkl             |  Bin 0 -> 17436 bytes
 .../2026-06-30_15-01-41/params/env.yaml            | 1029 +++++++++++++
 ...vents.out.tfevents.1782807039.moujank.1341838.0 |  Bin 0 -> 5581183 bytes
 .../2026-06-30_16-10-31/git/wheeled_leg.diff       |  929 +++++++++++
 .../2026-06-30_16-10-31/model_0.pt                 |    3 +
 .../2026-06-30_16-10-31/model_100.pt               |    3 +
 .../2026-06-30_16-10-31/model_1000.pt              |    3 +
 .../2026-06-30_16-10-31/model_1050.pt              |    3 +
 .../2026-06-30_16-10-31/model_1100.pt              |    3 +
 .../2026-06-30_16-10-31/model_1150.pt              |    3 +
 .../2026-06-30_16-10-31/model_1200.pt              |    3 +
 .../2026-06-30_16-10-31/model_1250.pt              |    3 +
 .../2026-06-30_16-10-31/model_1300.pt              |    3 +
 .../2026-06-30_16-10-31/model_1350.pt              |    3 +
 .../2026-06-30_16-10-31/model_1400.pt              |    3 +
 .../2026-06-30_16-10-31/model_1450.pt              |    3 +
 .../2026-06-30_16-10-31/model_150.pt               |    3 +
 .../2026-06-30_16-10-31/model_1500.pt              |    3 +
 .../2026-06-30_16-10-31/model_1550.pt              |    3 +
 .../2026-06-30_16-10-31/model_1600.pt              |    3 +
 .../2026-06-30_16-10-31/model_1650.pt              |    3 +
 .../2026-06-30_16-10-31/model_1700.pt              |    3 +
 .../2026-06-30_16-10-31/model_1750.pt              |    3 +
 .../2026-06-30_16-10-31/model_1800.pt              |    3 +
 .../2026-06-30_16-10-31/model_1850.pt              |    3 +
 .../2026-06-30_16-10-31/model_1900.pt              |    3 +
 .../2026-06-30_16-10-31/model_1950.pt              |    3 +
 .../2026-06-30_16-10-31/model_200.pt               |    3 +
 .../2026-06-30_16-10-31/model_2000.pt              |    3 +
 .../2026-06-30_16-10-31/model_2050.pt              |    3 +
 .../2026-06-30_16-10-31/model_2100.pt              |    3 +
 .../2026-06-30_16-10-31/model_2150.pt              |    3 +
 .../2026-06-30_16-10-31/model_2200.pt              |    3 +
 .../2026-06-30_16-10-31/model_2250.pt              |    3 +
 .../2026-06-30_16-10-31/model_2300.pt              |    3 +
 .../2026-06-30_16-10-31/model_2350.pt              |    3 +
 .../2026-06-30_16-10-31/model_2400.pt              |    3 +
 .../2026-06-30_16-10-31/model_2450.pt              |    3 +
 .../2026-06-30_16-10-31/model_250.pt               |    3 +
 .../2026-06-30_16-10-31/model_2500.pt              |    3 +
 .../2026-06-30_16-10-31/model_2550.pt              |    3 +
 .../2026-06-30_16-10-31/model_2600.pt              |    3 +
 .../2026-06-30_16-10-31/model_2650.pt              |    3 +
 .../2026-06-30_16-10-31/model_2700.pt              |    3 +
 .../2026-06-30_16-10-31/model_2750.pt              |    3 +
 .../2026-06-30_16-10-31/model_2800.pt              |    3 +
 .../2026-06-30_16-10-31/model_2850.pt              |    3 +
 .../2026-06-30_16-10-31/model_2900.pt              |    3 +
 .../2026-06-30_16-10-31/model_2950.pt              |    3 +
 .../2026-06-30_16-10-31/model_300.pt               |    3 +
 .../2026-06-30_16-10-31/model_3000.pt              |    3 +
 .../2026-06-30_16-10-31/model_3050.pt              |    3 +
 .../2026-06-30_16-10-31/model_350.pt               |    3 +
 .../2026-06-30_16-10-31/model_400.pt               |    3 +
 .../2026-06-30_16-10-31/model_450.pt               |    3 +
 .../2026-06-30_16-10-31/model_50.pt                |    3 +
 .../2026-06-30_16-10-31/model_500.pt               |    3 +
 .../2026-06-30_16-10-31/model_550.pt               |    3 +
 .../2026-06-30_16-10-31/model_600.pt               |    3 +
 .../2026-06-30_16-10-31/model_650.pt               |    3 +
 .../2026-06-30_16-10-31/model_700.pt               |    3 +
 .../2026-06-30_16-10-31/model_750.pt               |    3 +
 .../2026-06-30_16-10-31/model_800.pt               |    3 +
 .../2026-06-30_16-10-31/model_850.pt               |    3 +
 .../2026-06-30_16-10-31/model_900.pt               |    3 +
 .../2026-06-30_16-10-31/model_950.pt               |    3 +
 .../2026-06-30_16-10-31/params/agent.pkl           |  Bin 0 -> 1052 bytes
 .../2026-06-30_16-10-31/params/agent.yaml          |   43 +
 .../2026-06-30_16-10-31/params/env.pkl             |  Bin 0 -> 17436 bytes
 .../2026-06-30_16-10-31/params/env.yaml            | 1029 +++++++++++++
 ...events.out.tfevents.1783057521.moujank.377781.0 |  Bin 0 -> 7657571 bytes
 .../2026-07-03_13-45-06/git/wheeled_leg.diff       |  699 +++++++++
 .../2026-07-03_13-45-06/model_0.pt                 |    3 +
 .../2026-07-03_13-45-06/model_100.pt               |    3 +
 .../2026-07-03_13-45-06/model_1000.pt              |    3 +
 .../2026-07-03_13-45-06/model_1050.pt              |    3 +
 .../2026-07-03_13-45-06/model_1100.pt              |    3 +
 .../2026-07-03_13-45-06/model_1150.pt              |    3 +
 .../2026-07-03_13-45-06/model_1200.pt              |    3 +
 .../2026-07-03_13-45-06/model_1250.pt              |    3 +
 .../2026-07-03_13-45-06/model_1300.pt              |    3 +
 .../2026-07-03_13-45-06/model_1350.pt              |    3 +
 .../2026-07-03_13-45-06/model_1400.pt              |    3 +
 .../2026-07-03_13-45-06/model_1450.pt              |    3 +
 .../2026-07-03_13-45-06/model_150.pt               |    3 +
 .../2026-07-03_13-45-06/model_1500.pt              |    3 +
 .../2026-07-03_13-45-06/model_1550.pt              |    3 +
 .../2026-07-03_13-45-06/model_1600.pt              |    3 +
 .../2026-07-03_13-45-06/model_1650.pt              |    3 +
 .../2026-07-03_13-45-06/model_1700.pt              |    3 +
 .../2026-07-03_13-45-06/model_1750.pt              |    3 +
 .../2026-07-03_13-45-06/model_1800.pt              |    3 +
 .../2026-07-03_13-45-06/model_1850.pt              |    3 +
 .../2026-07-03_13-45-06/model_1900.pt              |    3 +
 .../2026-07-03_13-45-06/model_1950.pt              |    3 +
 .../2026-07-03_13-45-06/model_200.pt               |    3 +
 .../2026-07-03_13-45-06/model_2000.pt              |    3 +
 .../2026-07-03_13-45-06/model_2050.pt              |    3 +
 .../2026-07-03_13-45-06/model_2100.pt              |    3 +
 .../2026-07-03_13-45-06/model_2150.pt              |    3 +
 .../2026-07-03_13-45-06/model_2200.pt              |    3 +
 .../2026-07-03_13-45-06/model_2250.pt              |    3 +
 .../2026-07-03_13-45-06/model_2300.pt              |    3 +
 .../2026-07-03_13-45-06/model_2350.pt              |    3 +
 .../2026-07-03_13-45-06/model_2400.pt              |    3 +
 .../2026-07-03_13-45-06/model_2450.pt              |    3 +
 .../2026-07-03_13-45-06/model_250.pt               |    3 +
 .../2026-07-03_13-45-06/model_2500.pt              |    3 +
 .../2026-07-03_13-45-06/model_2550.pt              |    3 +
 .../2026-07-03_13-45-06/model_2600.pt              |    3 +
 .../2026-07-03_13-45-06/model_2650.pt              |    3 +
 .../2026-07-03_13-45-06/model_2700.pt              |    3 +
 .../2026-07-03_13-45-06/model_2750.pt              |    3 +
 .../2026-07-03_13-45-06/model_2800.pt              |    3 +
 .../2026-07-03_13-45-06/model_2850.pt              |    3 +
 .../2026-07-03_13-45-06/model_2900.pt              |    3 +
 .../2026-07-03_13-45-06/model_2950.pt              |    3 +
 .../2026-07-03_13-45-06/model_300.pt               |    3 +
 .../2026-07-03_13-45-06/model_3000.pt              |    3 +
 .../2026-07-03_13-45-06/model_3050.pt              |    3 +
 .../2026-07-03_13-45-06/model_3100.pt              |    3 +
 .../2026-07-03_13-45-06/model_3150.pt              |    3 +
 .../2026-07-03_13-45-06/model_3200.pt              |    3 +
 .../2026-07-03_13-45-06/model_3250.pt              |    3 +
 .../2026-07-03_13-45-06/model_3300.pt              |    3 +
 .../2026-07-03_13-45-06/model_3350.pt              |    3 +
 .../2026-07-03_13-45-06/model_3400.pt              |    3 +
 .../2026-07-03_13-45-06/model_3450.pt              |    3 +
 .../2026-07-03_13-45-06/model_350.pt               |    3 +
 .../2026-07-03_13-45-06/model_3500.pt              |    3 +
 .../2026-07-03_13-45-06/model_3550.pt              |    3 +
 .../2026-07-03_13-45-06/model_3600.pt              |    3 +
 .../2026-07-03_13-45-06/model_3650.pt              |    3 +
 .../2026-07-03_13-45-06/model_3700.pt              |    3 +
 .../2026-07-03_13-45-06/model_3750.pt              |    3 +
 .../2026-07-03_13-45-06/model_3800.pt              |    3 +
 .../2026-07-03_13-45-06/model_3850.pt              |    3 +
 .../2026-07-03_13-45-06/model_3900.pt              |    3 +
 .../2026-07-03_13-45-06/model_3950.pt              |    3 +
 .../2026-07-03_13-45-06/model_400.pt               |    3 +
 .../2026-07-03_13-45-06/model_4000.pt              |    3 +
 .../2026-07-03_13-45-06/model_4050.pt              |    3 +
 .../2026-07-03_13-45-06/model_4100.pt              |    3 +
 .../2026-07-03_13-45-06/model_4150.pt              |    3 +
 .../2026-07-03_13-45-06/model_4200.pt              |    3 +
 .../2026-07-03_13-45-06/model_450.pt               |    3 +
 .../2026-07-03_13-45-06/model_50.pt                |    3 +
 .../2026-07-03_13-45-06/model_500.pt               |    3 +
 .../2026-07-03_13-45-06/model_550.pt               |    3 +
 .../2026-07-03_13-45-06/model_600.pt               |    3 +
 .../2026-07-03_13-45-06/model_650.pt               |    3 +
 .../2026-07-03_13-45-06/model_700.pt               |    3 +
 .../2026-07-03_13-45-06/model_750.pt               |    3 +
 .../2026-07-03_13-45-06/model_800.pt               |    3 +
 .../2026-07-03_13-45-06/model_850.pt               |    3 +
 .../2026-07-03_13-45-06/model_900.pt               |    3 +
 .../2026-07-03_13-45-06/model_950.pt               |    3 +
 .../2026-07-03_13-45-06/params/agent.pkl           |  Bin 0 -> 1052 bytes
 .../2026-07-03_13-45-06/params/agent.yaml          |   43 +
 .../2026-07-03_13-45-06/params/env.pkl             |  Bin 0 -> 17430 bytes
 .../2026-07-03_13-45-06/params/env.yaml            | 1029 +++++++++++++
 .../best/exported/policy.onnx                      |  Bin 0 -> 9509 bytes
 .../best/exported/policy.pt                        |    3 +
 .../rsl_rl/wheeled_base_locked_hard/best/model.pt  |    1 +
 .../rsl_rl/wheeled_base_locked_hard/best/notes.md  |   25 +
 .../rsl_rl/wheeled_cartesian_grasp/best/notes.md   |   19 +
 .../wheeled_grasp_pose_locomotion/best/notes.md    |   19 +
 .../wheeled_loaded_grasp_obstacle/best/notes.md    |   19 +
 .../outputs/2026-06-30/10-55-37/.hydra/config.yaml |  488 ++++++
 .../outputs/2026-06-30/10-55-37/.hydra/hydra.yaml  |  154 ++
 .../2026-06-30/10-55-37/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-06-30/10-55-37/hydra.log          |   10 +
 .../outputs/2026-06-30/12-40-10/.hydra/config.yaml |  942 ++++++++++++
 .../outputs/2026-06-30/12-40-10/.hydra/hydra.yaml  |  154 ++
 .../2026-06-30/12-40-10/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-06-30/12-40-10/hydra.log          |   10 +
 .../outputs/2026-06-30/14-46-51/.hydra/config.yaml |  904 +++++++++++
 .../outputs/2026-06-30/14-46-51/.hydra/hydra.yaml  |  154 ++
 .../2026-06-30/14-46-51/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-06-30/14-46-51/hydra.log          |    0
 .../outputs/2026-06-30/14-49-52/.hydra/config.yaml |  904 +++++++++++
 .../outputs/2026-06-30/14-49-52/.hydra/hydra.yaml  |  154 ++
 .../2026-06-30/14-49-52/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-06-30/14-49-52/hydra.log          |   10 +
 .../outputs/2026-06-30/15-01-41/.hydra/config.yaml |  990 ++++++++++++
 .../outputs/2026-06-30/15-01-41/.hydra/hydra.yaml  |  154 ++
 .../2026-06-30/15-01-41/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-06-30/15-01-41/hydra.log          |   10 +
 .../outputs/2026-06-30/16-08-28/.hydra/config.yaml |  904 +++++++++++
 .../outputs/2026-06-30/16-08-28/.hydra/hydra.yaml  |  154 ++
 .../2026-06-30/16-08-28/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-06-30/16-08-28/hydra.log          |   10 +
 .../outputs/2026-06-30/16-10-31/.hydra/config.yaml |  990 ++++++++++++
 .../outputs/2026-06-30/16-10-31/.hydra/hydra.yaml  |  154 ++
 .../2026-06-30/16-10-31/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-06-30/16-10-31/hydra.log          |   10 +
 .../outputs/2026-06-30/17-05-43/.hydra/config.yaml |  949 ++++++++++++
 .../outputs/2026-06-30/17-05-43/.hydra/hydra.yaml  |  154 ++
 .../2026-06-30/17-05-43/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-06-30/17-05-43/hydra.log          |   10 +
 .../outputs/2026-07-01/20-26-38/.hydra/config.yaml | 1062 +++++++++++++
 .../outputs/2026-07-01/20-26-38/.hydra/hydra.yaml  |  154 ++
 .../2026-07-01/20-26-38/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-01/20-26-38/hydra.log          |   10 +
 .../outputs/2026-07-02/15-21-02/.hydra/config.yaml | 1062 +++++++++++++
 .../outputs/2026-07-02/15-21-02/.hydra/hydra.yaml  |  154 ++
 .../2026-07-02/15-21-02/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-02/15-21-02/hydra.log          |   10 +
 .../outputs/2026-07-02/21-17-27/.hydra/config.yaml | 1062 +++++++++++++
 .../outputs/2026-07-02/21-17-27/.hydra/hydra.yaml  |  154 ++
 .../2026-07-02/21-17-27/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-02/21-17-27/hydra.log          |   10 +
 .../outputs/2026-07-03/09-48-21/.hydra/config.yaml |  942 ++++++++++++
 .../outputs/2026-07-03/09-48-21/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/09-48-21/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/09-48-21/hydra.log          |   10 +
 .../outputs/2026-07-03/09-49-24/.hydra/config.yaml |  942 ++++++++++++
 .../outputs/2026-07-03/09-49-24/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/09-49-24/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/09-49-24/hydra.log          |   10 +
 .../outputs/2026-07-03/09-51-07/.hydra/config.yaml |  942 ++++++++++++
 .../outputs/2026-07-03/09-51-07/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/09-51-07/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/09-51-07/hydra.log          |   10 +
 .../outputs/2026-07-03/09-52-21/.hydra/config.yaml |  942 ++++++++++++
 .../outputs/2026-07-03/09-52-21/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/09-52-21/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/09-52-21/hydra.log          |   10 +
 .../outputs/2026-07-03/12-42-43/.hydra/config.yaml |  904 +++++++++++
 .../outputs/2026-07-03/12-42-43/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/12-42-43/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/12-42-43/hydra.log          |   10 +
 .../outputs/2026-07-03/13-45-06/.hydra/config.yaml |  990 ++++++++++++
 .../outputs/2026-07-03/13-45-06/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/13-45-06/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/13-45-06/hydra.log          |   10 +
 .../outputs/2026-07-03/15-15-53/.hydra/config.yaml |  949 ++++++++++++
 .../outputs/2026-07-03/15-15-53/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/15-15-53/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/15-15-53/hydra.log          |   10 +
 .../outputs/2026-07-03/15-17-51/.hydra/config.yaml | 1062 +++++++++++++
 .../outputs/2026-07-03/15-17-51/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/15-17-51/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/15-17-51/hydra.log          |   10 +
 .../outputs/2026-07-03/20-35-38/.hydra/config.yaml | 1024 +++++++++++++
 .../outputs/2026-07-03/20-35-38/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/20-35-38/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/20-35-38/hydra.log          |   10 +
 .../outputs/2026-07-03/21-38-43/.hydra/config.yaml |  940 ++++++++++++
 .../outputs/2026-07-03/21-38-43/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/21-38-43/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/21-38-43/hydra.log          |   10 +
 .../outputs/2026-07-03/22-16-08/.hydra/config.yaml | 1056 +++++++++++++
 .../outputs/2026-07-03/22-16-08/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/22-16-08/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/22-16-08/hydra.log          |   10 +
 .../outputs/2026-07-03/22-17-37/.hydra/config.yaml | 1056 +++++++++++++
 .../outputs/2026-07-03/22-17-37/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/22-17-37/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/22-17-37/hydra.log          |   10 +
 .../outputs/2026-07-03/22-18-04/.hydra/config.yaml | 1056 +++++++++++++
 .../outputs/2026-07-03/22-18-04/.hydra/hydra.yaml  |  154 ++
 .../2026-07-03/22-18-04/.hydra/overrides.yaml      |    1 +
 .../outputs/2026-07-03/22-18-04/hydra.log          |   10 +
 .../wheeled_train/wheeled_leg/scripts/list_envs.py |   64 +
 .../wheeled_leg/scripts/random_agent.py            |   72 +
 .../wheeled_leg/scripts/rl_games/play.py           |  234 +++
 .../wheeled_leg/scripts/rl_games/train.py          |  219 +++
 .../wheeled_leg/scripts/rsl_rl/cli_args.py         |   93 ++
 .../wheeled_leg/scripts/rsl_rl/play.py             |  213 +++
 .../wheeled_leg/scripts/rsl_rl/train.py            |  196 +++
 .../scripts/rsl_rl/update_best_policy.py           |  133 ++
 .../wheeled_train/wheeled_leg/scripts/skrl/play.py |  246 +++
 .../wheeled_leg/scripts/skrl/train.py              |  216 +++
 .../wheeled_leg/scripts/zero_agent.py              |   72 +
 .../source/wheeled_leg/config/extension.toml       |   35 +
 .../source/wheeled_leg/docs/CHANGELOG.rst          |   10 +
 .../wheeled_leg/source/wheeled_leg/pyproject.toml  |    3 +
 .../wheeled_leg/source/wheeled_leg/setup.py        |   49 +
 .../source/wheeled_leg/wheeled_leg/__init__.py     |   14 +
 .../wheeled_leg/wheeled_leg/tasks/__init__.py      |   17 +
 .../wheeled_leg/tasks/manager_based/__init__.py    |    6 +
 .../tasks/manager_based/wheeled_leg/__init__.py    |  123 ++
 .../manager_based/wheeled_leg/agents/__init__.py   |    4 +
 .../wheeled_leg/agents/rl_games_ppo_cfg.yaml       |   78 +
 .../wheeled_leg/agents/rsl_rl_ppo_cfg.py           |   90 ++
 .../wheeled_leg/agents/skrl_amp_cfg.yaml           |  111 ++
 .../wheeled_leg/agents/skrl_ppo_cfg.yaml           |   80 +
 .../wheeled_leg/arm_free_locomotion_env_cfg.py     |  326 ++++
 .../wheeled_leg/arm_free_obstacle_env_cfg.py       |  151 ++
 .../wheeled_leg/arm_free_recovery_env_cfg.py       |  200 +++
 .../manager_based/wheeled_leg/asset/wheeled.py     |   64 +
 .../manager_based/wheeled_leg/asset/wheeled_arm.py |   80 +
 .../wheeled_leg/asset/wheeled_step.py              |   64 +
 .../wheeled_leg/base_locked_locomotion_env_cfg.py  |  636 ++++++++
 .../wheeled_leg/cartesian_grasp_env_cfg.py         |  222 +++
 .../wheeled_leg/grasp_pose_locomotion_env_cfg.py   |  156 ++
 .../wheeled_leg/loaded_grasp_obstacle_env_cfg.py   |  174 +++
 .../manager_based/wheeled_leg/mdp/__init__.py      |   15 +
 .../tasks/manager_based/wheeled_leg/mdp/actions.py |   29 +
 .../manager_based/wheeled_leg/mdp/commands.py      |   31 +
 .../tasks/manager_based/wheeled_leg/mdp/events.py  |   30 +
 .../manager_based/wheeled_leg/mdp/observation.py   |    3 +
 .../manager_based/wheeled_leg/mdp/observations.py  |   67 +
 .../tasks/manager_based/wheeled_leg/mdp/rewards.py |  150 ++
 .../manager_based/wheeled_leg/mdp/terminations.py  |   31 +
 .../manager_based/wheeled_leg/terrains/__init__.py |    7 +
 .../wheeled_leg/terrains/arm_free_locomotion.py    |   31 +
 .../wheeled_leg/terrains/base_locked.py            |   69 +
 .../wheeled_leg/terrains/cartesian_grasp.py        |   28 +
 .../wheeled_leg/terrains/grasp_pose.py             |   28 +
 .../wheeled_leg/terrains/loaded_grasp_obstacle.py  |   46 +
 .../manager_based/wheeled_leg/terrains/obstacle.py |   46 +
 .../manager_based/wheeled_leg/terrains/recovery.py |   28 +
 .../manager_based/wheeled_leg/terrains/terrains.py |   11 +
 .../manager_based/wheeled_leg/usd/usd/config.yaml  |   23 +
 .../usd/usd/configuration/wheeled_arm_base.usd     |    3 +
 .../usd/usd/configuration/wheeled_arm_physics.usd  |    3 +
 .../usd/usd/configuration/wheeled_arm_robot.usd    |    3 +
 .../usd/usd/configuration/wheeled_arm_sensor.usd   |    3 +
 .../wheeled_leg/usd/usd/wheeled_arm.usd            |    3 +
 .../wheeled_leg/usd/usd_V/config.yaml              |   23 +
 .../configuration/wheeled_urdf_init_V_base.usd     |    3 +
 .../configuration/wheeled_urdf_init_V_physics.usd  |    3 +
 .../configuration/wheeled_urdf_init_V_robot.usd    |    3 +
 .../configuration/wheeled_urdf_init_V_sensor.usd   |    3 +
 .../wheeled_leg/usd/usd_V/wheeled_urdf_init_V.usd  |    3 +
 .../wheeled_leg/usd/usd_arm/config.yaml            |   23 +
 .../usd/usd_arm/configuration/wheeled_arm_base.usd |    3 +
 .../usd_arm/configuration/wheeled_arm_physics.usd  |    3 +
 .../usd_arm/configuration/wheeled_arm_robot.usd    |    3 +
 .../usd_arm/configuration/wheeled_arm_sensor.usd   |    3 +
 .../wheeled_leg/usd/usd_arm/wheeled_arm.usd        |    3 +
 .../wheeled_leg/usd/usd_leg/config.yaml            |   23 +
 .../usd/usd_leg/configuration/wheeled_leg_base.usd |    3 +
 .../usd_leg/configuration/wheeled_leg_physics.usd  |    3 +
 .../usd_leg/configuration/wheeled_leg_robot.usd    |    3 +
 .../usd_leg/configuration/wheeled_leg_sensor.usd   |    3 +
 .../wheeled_leg/usd/usd_leg/wheeled_leg.usd        |    3 +
 .../wheeled_leg/ui_extension_example.py            |   46 +
 1710 files changed, 74648 insertions(+), 7 deletions(-)

## 23387b5 - sync: 2026-06-30 17:08:54 +0800
Author: moujank <moujank588@gmail.com>
Date: 2026-06-30 17:08:54 +0800

 .vscode/browse.vc.db                  | Bin 0 -> 737280 bytes
 .vscode/browse.vc.db-shm              | Bin 0 -> 32768 bytes
 .vscode/browse.vc.db-wal              |   0
 .vscode/c_cpp_properties.json         |  20 +++
 .vscode/settings.json                 |   9 ++
 1.txt                                 | 288 ++++++++++++++++++++++++++++++++++
 modules/wheeled_deploy/README.md      |   4 +
 modules/wheeled_description/README.md |   4 +
 modules/wheeled_firmware/README.md    |   4 +
 modules/wheeled_hardware/README.md    |   4 +
 modules/wheeled_mujoco/README.md      |   4 +
 modules/wheeled_navigation/README.md  |   4 +
 modules/wheeled_sim_bridge/README.md  |   4 +
 modules/wheeled_teleop/README.md      |   4 +
 modules/wheeled_train/README.md       |   4 +
 modules/wheeled_train/wheeled_leg     |   1 +
 16 files changed, 354 insertions(+)

## 6db4296 - Add README for workspace scripts
Author: moujank <moujank588@gmail.com>
Date: 2026-06-29 16:16:09 +0800

 README.md | 59 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 1 file changed, 59 insertions(+)

## 68d6bba - Add Git history log generator
Author: moujank <moujank588@gmail.com>
Date: 2026-06-29 16:07:31 +0800

 GIT_HISTORY.md        | 20 ++++++++++++++++++++
 update_git_history.sh | 24 ++++++++++++++++++++++++
 2 files changed, 44 insertions(+)

## 5c7a1f0 - sync: 2026-06-29 15:57:35 +0800
Author: moujank <moujank588@gmail.com>
Date: 2026-06-29 15:57:35 +0800

 sync_to_github.sh | 27 +++++++++++++++++++++++++++
 1 file changed, 27 insertions(+)

## 047ca64 - Initial commit
Author: moujank <moujank588@gmail.com>
Date: 2026-06-29 15:42:33 +0800

 .gitignore   |   2 +
 thinking.txt | 624 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 2 files changed, 626 insertions(+)
