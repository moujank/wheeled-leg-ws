# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

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
    "Wheeled-Leg-v1",
    f"{__name__}.v2_wheeled_leg_v1:WheeledLegV1EnvCfg",
    f"{agents.__name__}.rsl_rl_ppo_cfg:WheeledLegV1PPORunnerCfg",
)
_register_rsl_rl_task(
    "Wheeled-Leg-Play-v1",
    f"{__name__}.v2_wheeled_leg_v1:WheeledLegV1EnvCfg_PLAY",
    f"{agents.__name__}.rsl_rl_ppo_cfg:WheeledLegV1PPORunnerCfg",
)

_register_rsl_rl_task(
    "Wheeled-Arm-v1",
    f"{__name__}.v2_wheeled_arm_v1:WheeledArmV1EnvCfg",
    f"{agents.__name__}.rsl_rl_ppo_cfg:WheeledArmV1PPORunnerCfg",
)
_register_rsl_rl_task(
    "Wheeled-Arm-Play-v1",
    f"{__name__}.v2_wheeled_arm_v1:WheeledArmV1EnvCfg_PLAY",
    f"{agents.__name__}.rsl_rl_ppo_cfg:WheeledArmV1PPORunnerCfg",
)

_register_rsl_rl_task(
    "Wheeled-Leg-Distillation-v1",
    f"{__name__}.v2_wheeled_leg_v1:WheeledLegV1DistillationEnvCfg",
    f"{agents.__name__}.rsl_rl_ppo_cfg:DistillationRunnerCfg",
)
_register_rsl_rl_task(
    "Wheeled-Leg-Distillation-Play-v1",
    f"{__name__}.v2_wheeled_leg_v1:WheeledLegV1DistillationEnvCfg_PLAY",
    f"{agents.__name__}.rsl_rl_ppo_cfg:DistillationRunnerCfg",
)

_register_rsl_rl_task(
    "Wheeled-Leg-Terrain-Balance",
    f"{__name__}.terrain_balance_env_cfg:WheeledLegTerrainBalanceEnvCfg",
    f"{agents.__name__}.rsl_rl_ppo_cfg:TerrainBalancePPORunnerCfg",
)
_register_rsl_rl_task(
    "Wheeled-Leg-Terrain-Balance-Play",
    f"{__name__}.terrain_balance_env_cfg:WheeledLegTerrainBalanceEnvCfg_PLAY",
    f"{agents.__name__}.rsl_rl_ppo_cfg:TerrainBalancePPORunnerCfg",
)

_register_rsl_rl_task(
    "Wheeled-Leg-Terrains",
    f"{__name__}.v2_wheeled_leg_terrains:WheeledLegTerrainsEnvCfg",
    f"{agents.__name__}.rsl_rl_ppo_cfg:WheeledLegTerrainsPPORunnerCfg",
)
_register_rsl_rl_task(
    "Wheeled-Leg-Terrains-Play",
    f"{__name__}.v2_wheeled_leg_terrains:WheeledLegTerrainsEnvCfg_PLAY",
    f"{agents.__name__}.rsl_rl_ppo_cfg:WheeledLegTerrainsPPORunnerCfg",
)

_register_rsl_rl_task(
    "Wheeled-Leg-Obstacle-Jump",
    f"{__name__}.v2_wheeled_leg_obstacle_jump:WheeledLegObstacleJumpEnvCfg",
    f"{agents.__name__}.rsl_rl_ppo_cfg:ObstacleJumpPPORunnerCfg",
)
_register_rsl_rl_task(
    "Wheeled-Leg-Obstacle-Jump-Play",
    f"{__name__}.v2_wheeled_leg_obstacle_jump:WheeledLegObstacleJumpEnvCfg_PLAY",
    f"{agents.__name__}.rsl_rl_ppo_cfg:ObstacleJumpPPORunnerCfg",
)

_register_rsl_rl_task(
    "Wheeled-Leg-Jump",
    f"{__name__}.v2_wheeled_leg_jump:WheeledLegJumpEnvCfg",
    f"{agents.__name__}.rsl_rl_ppo_cfg:JumpPPORunnerCfg",
)
_register_rsl_rl_task(
    "Wheeled-Leg-Jump-Play",
    f"{__name__}.v2_wheeled_leg_jump:WheeledLegJumpEnvCfg_PLAY",
    f"{agents.__name__}.rsl_rl_ppo_cfg:JumpPPORunnerCfg",
)

_register_rsl_rl_task(
    "Wheeled-Base-Locked-simple",
    f"{__name__}.base_locked_locomotion_env_cfg:BaseLockedLocomotionEnvCfg",
    _RSL_RL_BASE_LOCKED_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Base-Locked-simple-Play",
    f"{__name__}.base_locked_locomotion_env_cfg:BaseLockedLocomotionEnvCfg_PLAY",
    _RSL_RL_BASE_LOCKED_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Base-Locked-Curriculum",
    f"{__name__}.base_locked_locomotion_hard_env_cfg:BaseLockedLocomotionHardEnvCfg",
    _RSL_RL_BASE_LOCKED_HARD_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Base-Locked-Curriculum-Play",
    f"{__name__}.base_locked_locomotion_hard_env_cfg:BaseLockedLocomotionHardEnvCfg_PLAY",
    _RSL_RL_BASE_LOCKED_HARD_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Locomotion-v0",
    f"{__name__}.arm_free_locomotion_env_cfg:ArmFreeLocomotionEnvCfg",
    _RSL_RL_ARM_FREE_LOCOMOTION_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Locomotion-Play-v0",
    f"{__name__}.arm_free_locomotion_env_cfg:ArmFreeLocomotionEnvCfg_PLAY",
    _RSL_RL_ARM_FREE_LOCOMOTION_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Recovery-v0",
    f"{__name__}.arm_free_recovery_env_cfg:ArmFreeRecoveryEnvCfg",
    _RSL_RL_ARM_FREE_RECOVERY_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Recovery-Play-v0",
    f"{__name__}.arm_free_recovery_env_cfg:ArmFreeRecoveryEnvCfg_PLAY",
    _RSL_RL_ARM_FREE_RECOVERY_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Obstacle-v0",
    f"{__name__}.arm_free_obstacle_env_cfg:ArmFreeObstacleEnvCfg",
    _RSL_RL_ARM_FREE_OBSTACLE_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Arm-Free-Obstacle-Play-v0",
    f"{__name__}.arm_free_obstacle_env_cfg:ArmFreeObstacleEnvCfg_PLAY",
    _RSL_RL_ARM_FREE_OBSTACLE_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Grasp-Pose-Locomotion-v0",
    f"{__name__}.grasp_pose_locomotion_env_cfg:GraspPoseLocomotionEnvCfg",
    _RSL_RL_GRASP_POSE_LOCOMOTION_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Grasp-Pose-Locomotion-Play-v0",
    f"{__name__}.grasp_pose_locomotion_env_cfg:GraspPoseLocomotionEnvCfg_PLAY",
    _RSL_RL_GRASP_POSE_LOCOMOTION_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Cartesian-Grasp-v0",
    f"{__name__}.cartesian_grasp_env_cfg:CartesianGraspEnvCfg",
    _RSL_RL_CARTESIAN_GRASP_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Cartesian-Grasp-Play-v0",
    f"{__name__}.cartesian_grasp_env_cfg:CartesianGraspEnvCfg_PLAY",
    _RSL_RL_CARTESIAN_GRASP_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Loaded-Grasp-Obstacle-v0",
    f"{__name__}.loaded_grasp_obstacle_env_cfg:LoadedGraspObstacleEnvCfg",
    _RSL_RL_LOADED_GRASP_OBSTACLE_CFG_ENTRY_POINT,
)
_register_rsl_rl_task(
    "Wheeled-Loaded-Grasp-Obstacle-Play-v0",
    f"{__name__}.loaded_grasp_obstacle_env_cfg:LoadedGraspObstacleEnvCfg_PLAY",
    _RSL_RL_LOADED_GRASP_OBSTACLE_CFG_ENTRY_POINT,
)
