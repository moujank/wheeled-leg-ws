# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


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
