# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import (
    RslRlDistillationAlgorithmCfg,
    RslRlDistillationStudentTeacherRecurrentCfg,
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
    RslRlSymmetryCfg,
)


def _bilateral_symmetry(coefficient):
    return RslRlSymmetryCfg(
        use_data_augmentation=True,
        use_mirror_loss=True,
        data_augmentation_func=(
            "wheeled_leg.tasks.manager_based.wheeled_leg.mdp.symmetry:compute_symmetric_states"
        ),
        mirror_loss_coeff=coefficient,
    )


@configclass
class PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 16
    max_iterations = 150
    save_interval = 50
    experiment_name = "cartpole_direct"
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        # Learn log-std instead of std directly so action noise stays positive during PPO updates.
        noise_std_type="log",
        actor_hidden_dims=[32, 32],
        critic_hidden_dims=[32, 32],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class WheeledLegPPORunnerCfg(PPORunnerCfg):
    """双轮足任务沿用基础 PPO 参数，并使用独立实验目录。"""

    experiment_name = "wheeled_leg_v0"


@configclass
class WheeledLegV1PPORunnerCfg(WheeledLegPPORunnerCfg):
    """Independent experiment for the wheeled_leg_v2 locomotion task."""

    experiment_name = "wheeled_leg_v1"
    logger = "tensorboard"
    num_steps_per_env = 32
    max_iterations = 3000
    empirical_normalization = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.35,
        noise_std_type="log",
        actor_hidden_dims=[128, 128, 64],
        critic_hidden_dims=[128, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        symmetry_cfg=_bilateral_symmetry(0.05),
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.15,
        entropy_coef=1.0e-4,
        num_learning_epochs=3,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        # The previous adaptive schedule grew above 0.002 late in training.
        schedule="fixed",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.005,
        max_grad_norm=0.5,
    )


@configclass
class WheeledArmV1PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """Flat locomotion with six actively controlled, folded arm joints."""

    experiment_name = "wheeled_arm_v1"
    logger = "tensorboard"
    num_steps_per_env = 32
    max_iterations = 5000
    save_interval = 50
    empirical_normalization = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.35,
        noise_std_type="log",
        actor_hidden_dims=[128, 128, 64],
        critic_hidden_dims=[128, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        # The single serial arm is not covered by the bilateral leg-only mapping.
        symmetry_cfg=None,
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.15,
        entropy_coef=1.0e-4,
        num_learning_epochs=3,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="fixed",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.005,
        max_grad_norm=0.5,
    )


@configclass
class WheeledLegDistillationAlgorithmCfg(RslRlDistillationAlgorithmCfg):
    """Expose the loss selector supported by the installed RSL-RL algorithm."""

    loss_type: str = "huber"


@configclass
class DistillationRunnerCfg(RslRlOnPolicyRunnerCfg):
    """Distill the v1 feed-forward PPO actor into a GRU student with Huber loss."""

    experiment_name = "wheeled_leg_v1_distillation"
    logger = "tensorboard"
    num_steps_per_env = 32
    max_iterations = 3000
    save_interval = 50
    # RSL-RL 2.3 reads student inputs from policy and expert inputs from teacher.
    # Observation groups are defined by DistillationObservationsCfg, not mapped here.
    empirical_normalization = True
    policy = RslRlDistillationStudentTeacherRecurrentCfg(
        init_noise_std=0.1,
        noise_std_type="scalar",
        student_hidden_dims=[512, 256, 128],
        # Must match WheeledLegV1PPORunnerCfg.policy and the PPO checkpoint.
        teacher_hidden_dims=[128, 128, 64],
        activation="elu",
        rnn_type="gru",
        rnn_hidden_dim=247,
        rnn_num_layers=2,
        teacher_recurrent=False,
    )
    algorithm = WheeledLegDistillationAlgorithmCfg(
        num_learning_epochs=3,
        learning_rate=3.0e-4,
        # Divide the 32-step rollout exactly; RSL-RL drops partial accumulations.
        gradient_length=16,
        max_grad_norm=1.0,
        loss_type="huber",
    )


@configclass
class TerrainBalancePPORunnerCfg(PPORunnerCfg):
    """Conservative PPO baseline for multi-terrain wheeled-leg balance."""

    experiment_name = "wheeled_leg_terrain_balance"
    logger = "tensorboard"
    num_steps_per_env = 32
    max_iterations = 5000
    empirical_normalization = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.35,
        noise_std_type="log",
        actor_hidden_dims=[128, 128, 64],
        critic_hidden_dims=[128, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        symmetry_cfg=_bilateral_symmetry(0.03),
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.15,
        entropy_coef=1.0e-4,
        num_learning_epochs=3,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="fixed",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.005,
        max_grad_norm=0.5,
    )


@configclass
class WheeledLegTerrainsPPORunnerCfg(WheeledLegV1PPORunnerCfg):
    """Conservative fine tuning of a terrain policy with a fresh optimizer."""

    experiment_name = "wheeled_leg_terrains"
    max_iterations = 8000
    def __post_init__(self):
        # configclass stores inherited fields on instances, not the base class.
        # Each instance receives its own deep-copied algorithm configuration.
        self.algorithm.learning_rate = 1.0e-4


@configclass
class ObstacleJumpPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """Independent PPO settings for unrestricted 20 cm traversal."""

    logger = "tensorboard"
    num_steps_per_env = 32
    save_interval = 50
    empirical_normalization = True
    experiment_name = "wheeled_leg_obstacle_jump"
    max_iterations = 7000
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.45,
        noise_std_type="log",
        actor_hidden_dims=[128, 128, 64],
        critic_hidden_dims=[128, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        symmetry_cfg=_bilateral_symmetry(0.02),
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.12,
        entropy_coef=2.0e-4,
        num_learning_epochs=3,
        num_mini_batches=4,
        learning_rate=2.5e-4,
        schedule="fixed",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.004,
        max_grad_norm=0.5,
    )


@configclass
class JumpPPORunnerCfg(ObstacleJumpPPORunnerCfg):
    """Stationary -> moving -> obstacle course with 12 appended observations."""

    experiment_name = "wheeled_leg_jump"
    max_iterations = 5000

    def __post_init__(self):
        self.algorithm.symmetry_cfg = _bilateral_symmetry(0.015)


@configclass
class BaseLockedPPORunnerCfg(PPORunnerCfg):
    experiment_name = "wheeled_base_locked_simple"
    empirical_normalization = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.6,
        actor_hidden_dims=[128, 128, 64],
        critic_hidden_dims=[128, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.002,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=5.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class BaseLockedHardPPORunnerCfg(PPORunnerCfg):
    """Dedicated PPO runner for the hard base-locked task.

    Use log-parameterized action noise so the learned standard deviation remains positive
    even under the harder terrain curriculum and stronger disturbances.
    """

    experiment_name = "wheeled_base_locked_hard"
    empirical_normalization = True
    # The hard task already learns to survive reliably. Late-training regressions are dominated
    # by excess exploration and overly aggressive PPO updates, so keep the policy more conservative.
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.45,
        noise_std_type="log",
        actor_hidden_dims=[128, 128, 64],
        critic_hidden_dims=[128, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.12,
        entropy_coef=8.0e-4,
        num_learning_epochs=3,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="adaptive",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.004,
        max_grad_norm=0.5,
    )


@configclass
class ArmFreeLocomotionPPORunnerCfg(PPORunnerCfg):
    experiment_name = "wheeled_arm_free_locomotion"


@configclass
class ArmFreeRecoveryPPORunnerCfg(PPORunnerCfg):
    experiment_name = "wheeled_arm_free_recovery"


@configclass
class ArmFreeObstaclePPORunnerCfg(PPORunnerCfg):
    experiment_name = "wheeled_arm_free_obstacle"


@configclass
class GraspPoseLocomotionPPORunnerCfg(PPORunnerCfg):
    experiment_name = "wheeled_grasp_pose_locomotion"
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        noise_std_type="log",
        actor_hidden_dims=[32, 32],
        critic_hidden_dims=[32, 32],
        activation="elu",
    )


@configclass
class CartesianGraspPPORunnerCfg(PPORunnerCfg):
    experiment_name = "wheeled_cartesian_grasp"


@configclass
class LoadedGraspObstaclePPORunnerCfg(PPORunnerCfg):
    experiment_name = "wheeled_loaded_grasp_obstacle"
