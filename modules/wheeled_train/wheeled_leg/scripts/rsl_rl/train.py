# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to train RL agent with RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip


# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")
parser.add_argument("--init_checkpoint", type=str, default=None,
                    help="Initialize a fresh experiment from compatible PPO weights and observation statistics.")
parser.add_argument("--terrain_phase", choices=["foundation", "focus", "retention"], default=None,
                    help="Terrains-only low-speed sampling phase; recorded in the environment config.")
parser.add_argument("--teacher_checkpoint", type=str, default=None,
                    help="PPO checkpoint path used as the frozen teacher for a new distillation run.")
parser.add_argument(
    "--distributed", action="store_true", default=False, help="Run training with multiple GPUs or nodes."
)
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
if args_cli.init_checkpoint and args_cli.resume:
    parser.error("--init_checkpoint starts a new experiment; it cannot be combined with --resume.")
if args_cli.teacher_checkpoint and (args_cli.resume or args_cli.init_checkpoint):
    parser.error("--teacher_checkpoint starts a new student; use --resume alone to continue distillation.")

# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Check for minimum supported RSL-RL version."""

import importlib.metadata as metadata
import platform

from packaging import version

# for distributed training, check minimum supported rsl-rl version
RSL_RL_VERSION = "2.3.1"
installed_version = metadata.version("rsl-rl-lib")
if args_cli.distributed and version.parse(installed_version) < version.parse(RSL_RL_VERSION):
    if platform.system() == "Windows":
        cmd = [r".\isaaclab.bat", "-p", "-m", "pip", "install", f"rsl-rl-lib=={RSL_RL_VERSION}"]
    else:
        cmd = ["./isaaclab.sh", "-p", "-m", "pip", "install", f"rsl-rl-lib=={RSL_RL_VERSION}"]
    print(
        f"Please install the correct version of RSL-RL.\nExisting version is: '{installed_version}'"
        f" and required version is: '{RSL_RL_VERSION}'.\nTo install the correct version, run:"
        f"\n\n\t{' '.join(cmd)}\n"
    )
    exit(1)

"""Rest everything follows."""

import gymnasium as gym
import hashlib
import json
import os
import torch
from datetime import datetime
from pathlib import Path
from warm_start import warm_start_runner
from distillation_runner import DistillationRunner

from rsl_rl.algorithms.ppo import PPO
from rsl_rl.modules.actor_critic import ActorCritic
from rsl_rl.runners import OnPolicyRunner
from rsl_rl.storage import RolloutStorage

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.dict import print_dict
from isaaclab.utils.io import dump_pickle, dump_yaml

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import wheeled_leg.tasks  # noqa: F401

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False


def _install_rsl_rl_numerics_guard() -> None:
    """Patch RSL-RL runtime to avoid PPO corruption from non-finite rollout samples."""

    if getattr(_install_rsl_rl_numerics_guard, "_installed", False):
        return

    _install_rsl_rl_numerics_guard._installed = True
    warn_counts: dict[str, int] = {}

    def warn_once(key: str, message: str, limit: int = 5) -> None:
        count = warn_counts.get(key, 0)
        if count < limit:
            print(message)
        warn_counts[key] = count + 1

    def sanitize_tensor(
        name: str,
        tensor: torch.Tensor | None,
        *,
        clip_value: float | None = None,
        fill_value: float = 0.0,
    ) -> torch.Tensor | None:
        if tensor is None:
            return None
        if not torch.isfinite(tensor).all():
            bad = int((~torch.isfinite(tensor)).sum().item())
            warn_once(name, f"[WARN][numerics] Sanitizing {bad}/{tensor.numel()} non-finite entries in {name}.")
            tensor = torch.nan_to_num(tensor, nan=fill_value, posinf=fill_value, neginf=fill_value)
        if clip_value is not None:
            if torch.any(torch.abs(tensor) > clip_value):
                warn_once(
                    f"{name}_clip",
                    f"[WARN][numerics] Clipping {name} to +/-{clip_value:.1e} to bound PPO updates.",
                )
            tensor = torch.clamp(tensor, min=-clip_value, max=clip_value)
        return tensor

    original_update_distribution = ActorCritic.update_distribution

    def safe_update_distribution(self: ActorCritic, observations):
        observations = sanitize_tensor("policy_observations", observations, clip_value=1.0e6)
        mean = sanitize_tensor("policy_mean", self.actor(observations), clip_value=1.0e3)
        if self.noise_std_type == "scalar":
            safe_std = sanitize_tensor("policy_std", self.std, clip_value=1.0e2, fill_value=1.0)
            std = torch.clamp(safe_std, min=1.0e-6).expand_as(mean)
        elif self.noise_std_type == "log":
            safe_log_std = sanitize_tensor("policy_log_std", self.log_std, clip_value=20.0)
            safe_log_std = torch.clamp(safe_log_std, min=-20.0, max=2.0)
            std = torch.exp(safe_log_std).expand_as(mean)
        else:
            return original_update_distribution(self, observations)
        std = sanitize_tensor("policy_distribution_std", std, clip_value=1.0e2, fill_value=1.0)
        std = torch.clamp(std, min=1.0e-6)
        self.distribution = torch.distributions.Normal(mean, std)

    ActorCritic.update_distribution = safe_update_distribution

    original_add_transitions = RolloutStorage.add_transitions

    def safe_add_transitions(self: RolloutStorage, transition: RolloutStorage.Transition):
        transition.observations = sanitize_tensor("rollout_observations", transition.observations, clip_value=1.0e6)
        transition.privileged_observations = sanitize_tensor(
            "rollout_privileged_observations", transition.privileged_observations, clip_value=1.0e6
        )
        transition.actions = sanitize_tensor("rollout_actions", transition.actions, clip_value=1.0e3)
        transition.rewards = sanitize_tensor("rollout_rewards", transition.rewards, clip_value=1.0e4)
        transition.values = sanitize_tensor("rollout_values", transition.values, clip_value=1.0e6)
        transition.actions_log_prob = sanitize_tensor(
            "rollout_actions_log_prob", transition.actions_log_prob, clip_value=1.0e3
        )
        transition.action_mean = sanitize_tensor("rollout_action_mean", transition.action_mean, clip_value=1.0e3)
        transition.action_sigma = sanitize_tensor(
            "rollout_action_sigma", transition.action_sigma, clip_value=1.0e2, fill_value=1.0
        )
        transition.rnd_state = sanitize_tensor("rollout_rnd_state", transition.rnd_state, clip_value=1.0e6)
        return original_add_transitions(self, transition)

    RolloutStorage.add_transitions = safe_add_transitions

    original_compute_returns = RolloutStorage.compute_returns

    def safe_compute_returns(self: RolloutStorage, last_values, gamma, lam, normalize_advantage: bool = True):
        self.rewards = sanitize_tensor("storage_rewards", self.rewards, clip_value=1.0e4)
        self.values = sanitize_tensor("storage_values", self.values, clip_value=1.0e6)
        last_values = sanitize_tensor("storage_last_values", last_values, clip_value=1.0e6)
        original_compute_returns(self, last_values, gamma, lam, normalize_advantage=normalize_advantage)
        self.returns = sanitize_tensor("storage_returns", self.returns, clip_value=1.0e6)
        self.advantages = sanitize_tensor("storage_advantages", self.advantages, clip_value=1.0e4)

    RolloutStorage.compute_returns = safe_compute_returns

    original_ppo_update = PPO.update

    def safe_ppo_update(self: PPO):
        loss_dict = original_ppo_update(self)
        for name, param in self.policy.named_parameters():
            if not torch.isfinite(param).all():
                warn_once(
                    f"param_{name}",
                    f"[WARN][numerics] Non-finite policy parameter detected after PPO update: {name}. Resetting bad entries.",
                )
                with torch.no_grad():
                    param.copy_(torch.nan_to_num(param, nan=0.0, posinf=0.0, neginf=0.0))
        return loss_dict

    PPO.update = safe_ppo_update


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):
    """Train with RSL-RL agent."""
    # override configurations with non-hydra CLI arguments
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    if args_cli.terrain_phase is not None:
        from wheeled_leg.tasks.manager_based.wheeled_leg.terrain_training import configure_traversal_phase
        configure_traversal_phase(env_cfg, args_cli.terrain_phase)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    agent_cfg.max_iterations = (
        args_cli.max_iterations if args_cli.max_iterations is not None else agent_cfg.max_iterations
    )
    is_distillation = agent_cfg.algorithm.class_name == "Distillation"
    if is_distillation and args_cli.init_checkpoint:
        raise ValueError("Distillation uses --teacher_checkpoint for a new student, or --resume for an existing student.")
    if args_cli.teacher_checkpoint:
        if not is_distillation:
            raise ValueError("--teacher_checkpoint requires a registered Distillation task.")
        if agent_cfg.resume:
            raise ValueError("--teacher_checkpoint cannot be combined with resume=True.")
        args_cli.teacher_checkpoint = os.path.abspath(os.path.expanduser(args_cli.teacher_checkpoint))
        if not os.path.isfile(args_cli.teacher_checkpoint):
            raise FileNotFoundError(f"--teacher_checkpoint does not exist: {args_cli.teacher_checkpoint}")
    elif is_distillation and not agent_cfg.resume:
        raise ValueError("A new distillation run requires --teacher_checkpoint /path/to/ppo/model.pt; "
                         "use --resume --load_run <student_run> to continue a student.")

    # ``--init_checkpoint`` is a warm start for a *new* experiment (for
    # example v1 -> Terrains).  It is already a complete path and must not be
    # passed through Isaac Lab's resume resolver, which searches inside the
    # destination experiment directory.  Some saved/configured runner cfgs
    # may carry ``resume=True``; override that state explicitly here.
    if args_cli.init_checkpoint:
        init_checkpoint = os.path.abspath(os.path.expanduser(args_cli.init_checkpoint))
        if not os.path.isfile(init_checkpoint):
            raise FileNotFoundError(f"--init_checkpoint does not exist: {init_checkpoint}")
        agent_cfg.resume = False
        args_cli.init_checkpoint = init_checkpoint

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # multi-gpu training configuration
    if args_cli.distributed:
        env_cfg.sim.device = f"cuda:{app_launcher.local_rank}"
        agent_cfg.device = f"cuda:{app_launcher.local_rank}"

        # set seed to have diversity in different threads
        seed = agent_cfg.seed + app_launcher.local_rank
        env_cfg.seed = seed
        agent_cfg.seed = seed

    # specify directory for logging experiments
    log_root_path = os.path.join(Path(__file__).resolve().parents[2], "logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Logging experiment in directory: {log_root_path}")
    # specify directory for logging runs: {time-stamp}_{run_name}
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # The Ray Tune workflow extracts experiment name using the logging line below, hence, do not change it (see PR #2346, comment-2819298849)
    print(f"Exact experiment name requested from command line: {log_dir}")
    if agent_cfg.run_name:
        log_dir += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root_path, log_dir)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # save resume path before creating a new log_dir
    resume_path = None
    if not args_cli.init_checkpoint and agent_cfg.resume:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "train"),
            "step_trigger": lambda step: step % args_cli.video_interval == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    if not is_distillation:
        _install_rsl_rl_numerics_guard()

    # create runner from rsl-rl
    runner_class = DistillationRunner if is_distillation else OnPolicyRunner
    runner = runner_class(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    # write git state to logs
    runner.add_git_repo_to_log(__file__)
    if args_cli.teacher_checkpoint:
        runner.load_teacher(args_cli.teacher_checkpoint)
        print(f"[INFO] Loaded frozen PPO teacher from: {args_cli.teacher_checkpoint}")
    if args_cli.init_checkpoint:
        source = warm_start_runner(runner, args_cli.init_checkpoint,
                                   appended_observations=getattr(env_cfg, "warm_start_appended_observations", 0),
                                   env=env,
                                   preserve_normalizer_count=getattr(env_cfg, "warm_start_preserve_normalizer_count", False))
        print(f"[INFO] Initialized fresh experiment from: {source}")
        # Keep checkpoint provenance alongside the run; env/agent YAML alone
        # cannot distinguish scratch training from a warm start.
        with open(source, "rb") as checkpoint_stream:
            source_hash = hashlib.file_digest(checkpoint_stream, "sha256").hexdigest()
        initialization = {
            "mode": "warm_start",
            "checkpoint": source,
            "sha256": source_hash,
            "preserve_normalizer_count": getattr(env_cfg, "warm_start_preserve_normalizer_count", False),
            "policy_normalizer_count_after_load": float(runner.obs_normalizer.count)
                if runner.empirical_normalization else None,
            "terrain_phase": getattr(env_cfg, "traversal_phase", None),
        }
        # The runner creates its log directory lazily at the first learning step.
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        Path(log_dir, "initialization.json").write_text(json.dumps(initialization, indent=2))
    # load the checkpoint
    if resume_path is not None:
        print(f"[INFO]: Loading model checkpoint from: {resume_path}")
        # load previously trained model
        runner.load(resume_path)

    # Resolve reflection maps and migrate normalization only after all checkpoint
    # loading. v0 has no symmetry configuration and keeps its original path.
    if getattr(agent_cfg.algorithm, "symmetry_cfg", None) is not None:
        from wheeled_leg.tasks.manager_based.wheeled_leg.mdp.symmetry import install_symmetry_normalization
        install_symmetry_normalization(runner, env)

    # dump the configuration into log-directory
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
    dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), agent_cfg)

    # run training
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
