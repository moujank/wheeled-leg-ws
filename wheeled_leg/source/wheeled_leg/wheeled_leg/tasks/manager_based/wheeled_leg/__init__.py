# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents
from . import arm_free_locomotion_env_cfg
from . import arm_free_obstacle_env_cfg
from . import arm_free_recovery_env_cfg
from . import base_locked_locomotion_env_cfg
from . import base_locked_locomotion_hard_env_cfg
from . import cartesian_grasp_env_cfg
from . import grasp_pose_locomotion_env_cfg
from . import loaded_grasp_obstacle_env_cfg

##
# Register Gym environments.
##

_RSL_RL_CFG_ENTRY_POINT = f"{agents.__name__}.rsl_rl_ppo_cfg:PPORunnerCfg"
_RSL_RL_BASE_LOCKED_CFG_ENTRY_POINT = f"{agents.__name__}.rsl_rl_ppo_cfg:BaseLockedPPORunnerCfg"
_RSL_RL_BASE_LOCKED_HARD_CFG_ENTRY_POINT = f"{agents.__name__}.rsl_rl_ppo_cfg:BaseLockedHardPPORunnerCfg"
_RSL_RL_ARM_FREE_LOCOMOTION_CFG_ENTRY_POINT = f"{agents.__name__}.rsl_rl_ppo_cfg:ArmFreeLocomotionPPORunnerCfg"
_RSL_RL_ARM_FREE_RECOVERY_CFG_ENTRY_POINT = f"{agents.__name__}.rsl_rl_ppo_cfg:ArmFreeRecoveryPPORunnerCfg"
_RSL_RL_ARM_FREE_OBSTACLE_CFG_ENTRY_POINT = f"{agents.__name__}.rsl_rl_ppo_cfg:ArmFreeObstaclePPORunnerCfg"
_RSL_RL_GRASP_POSE_LOCOMOTION_CFG_ENTRY_POINT = f"{agents.__name__}.rsl_rl_ppo_cfg:GraspPoseLocomotionPPORunnerCfg"
_RSL_RL_CARTESIAN_GRASP_CFG_ENTRY_POINT = f"{agents.__name__}.rsl_rl_ppo_cfg:CartesianGraspPPORunnerCfg"
_RSL_RL_LOADED_GRASP_OBSTACLE_CFG_ENTRY_POINT = f"{agents.__name__}.rsl_rl_ppo_cfg:LoadedGraspObstaclePPORunnerCfg"

gym.register(
    id="Wheeled-Leg-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.v2_wheeled_leg_v0:WheeledLegEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WheeledLegPPORunnerCfg",
    },
)



def _register_rsl_rl_task(task_id: str, env_cfg_entry_point, rsl_rl_cfg_entry_point: str = _RSL_RL_CFG_ENTRY_POINT) -> None:
    gym.register(
        id=task_id,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": env_cfg_entry_point,
            "rsl_rl_cfg_entry_point": rsl_rl_cfg_entry_point,
        },
    )

_register_rsl_rl_task(
    "Wheeled-Leg-Play-v0",
    f"{__name__}.v2_wheeled_leg_v0:WheeledLegEnvCfg_PLAY",
    f"{agents.__name__}.rsl_rl_ppo_cfg:WheeledLegPPORunnerCfg",
)

_register_rsl_rl_task(
    "Wheeled-Base-Locked-simple",
    f"{__name__}.base_locked_locomotion_env_cfg:BaseLockedLocomotionEnvCfg",
    _RSL_RL_BASE_LOCKED_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Base-Locked-simple-Play",
    base_locked_locomotion_env_cfg.BaseLockedLocomotionEnvCfg_PLAY,
    _RSL_RL_BASE_LOCKED_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Base-Locked-Curriculum",
    f"{__name__}.base_locked_locomotion_hard_env_cfg:BaseLockedLocomotionHardEnvCfg",
    _RSL_RL_BASE_LOCKED_HARD_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Base-Locked-Curriculum-Play",
    base_locked_locomotion_hard_env_cfg.BaseLockedLocomotionHardEnvCfg_PLAY,
    _RSL_RL_BASE_LOCKED_HARD_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Locomotion-v0",
    f"{__name__}.arm_free_locomotion_env_cfg:ArmFreeLocomotionEnvCfg",
    _RSL_RL_ARM_FREE_LOCOMOTION_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Locomotion-Play-v0",
    arm_free_locomotion_env_cfg.ArmFreeLocomotionEnvCfg_PLAY,
    _RSL_RL_ARM_FREE_LOCOMOTION_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Recovery-v0",
    f"{__name__}.arm_free_recovery_env_cfg:ArmFreeRecoveryEnvCfg",
    _RSL_RL_ARM_FREE_RECOVERY_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Recovery-Play-v0",
    arm_free_recovery_env_cfg.ArmFreeRecoveryEnvCfg_PLAY,
    _RSL_RL_ARM_FREE_RECOVERY_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Obstacle-v0",
    f"{__name__}.arm_free_obstacle_env_cfg:ArmFreeObstacleEnvCfg",
    _RSL_RL_ARM_FREE_OBSTACLE_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Obstacle-Play-v0",
    arm_free_obstacle_env_cfg.ArmFreeObstacleEnvCfg_PLAY,
    _RSL_RL_ARM_FREE_OBSTACLE_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Grasp-Pose-Locomotion-v0",
    f"{__name__}.grasp_pose_locomotion_env_cfg:GraspPoseLocomotionEnvCfg",
    _RSL_RL_GRASP_POSE_LOCOMOTION_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Grasp-Pose-Locomotion-Play-v0",
    grasp_pose_locomotion_env_cfg.GraspPoseLocomotionEnvCfg_PLAY,
    _RSL_RL_GRASP_POSE_LOCOMOTION_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Cartesian-Grasp-v0",
    f"{__name__}.cartesian_grasp_env_cfg:CartesianGraspEnvCfg",
    _RSL_RL_CARTESIAN_GRASP_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Cartesian-Grasp-Play-v0",
    cartesian_grasp_env_cfg.CartesianGraspEnvCfg_PLAY,
    _RSL_RL_CARTESIAN_GRASP_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Loaded-Grasp-Obstacle-v0",
    f"{__name__}.loaded_grasp_obstacle_env_cfg:LoadedGraspObstacleEnvCfg",
    _RSL_RL_LOADED_GRASP_OBSTACLE_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Loaded-Grasp-Obstacle-Play-v0",
    loaded_grasp_obstacle_env_cfg.LoadedGraspObstacleEnvCfg_PLAY,
    _RSL_RL_LOADED_GRASP_OBSTACLE_CFG_ENTRY_POINT,
)
