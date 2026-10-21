# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from RSL-RL."""

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
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--terrain_level", type=int, default=None, help="Fix the Terrains task to one terrain row (0-9).")
parser.add_argument("--terrain_speed", type=float, default=None, help="Fix the forward target after ramp-up (.20 < m/s <= .40).")
parser.add_argument("--max_steps", type=int, default=None, help="Stop evaluation after this many control steps.")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
if args_cli.max_steps is not None and args_cli.max_steps <= 0:
    parser.error("--max_steps must be positive")
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import csv
import os
import time
import torch
from datetime import datetime
from pathlib import Path

from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import wheeled_leg.tasks  # noqa: F401
from terrain_evaluation import configure_terrain_evaluation, set_fixed_terrain_level


def _resolve_artifact_log_dir(resume_path: str, experiment_log_root: str) -> str:
    """Resolve the directory where play artifacts should be stored.

    If the selected checkpoint comes from ``logs/.../<experiment>/best/model.pt``, then the parent
    directory is only the lightweight best-policy index. In that case, try to recover the original
    source run from ``best/notes.md`` and place play artifacts under the real training run directory.
    """

    checkpoint_path = Path(resume_path).resolve()
    checkpoint_parent = checkpoint_path.parent
    experiment_root = Path(experiment_log_root).resolve()

    if checkpoint_parent.name != "best":
        return str(checkpoint_parent)

    notes_path = checkpoint_parent / "notes.md"
    if not notes_path.is_file():
        return str(checkpoint_parent)

    source_run = None
    for line in notes_path.read_text(encoding="utf-8").splitlines():
        if line.startswith("source_run:"):
            source_run = line.split(":", 1)[1].strip()
            break

    if not source_run or source_run == "TBD":
        return str(checkpoint_parent)

    candidate_dir = experiment_root / source_run
    if candidate_dir.is_dir():
        return str(candidate_dir)

    return str(checkpoint_parent)


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):
    """Play with RSL-RL agent."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device
    configure_terrain_evaluation(env_cfg, args_cli.terrain_level, args_cli.terrain_speed)
    if args_cli.terrain_level is not None or args_cli.terrain_speed is not None:
        from isaaclab.managers import CurriculumTermCfg
        from wheeled_leg.tasks.manager_based.wheeled_leg.mdp.curriculums import terrain_levels_forward_traversal

        # Record terminal states before automatic reset, without changing rows.
        env_cfg.curriculum.terrain_levels = CurriculumTermCfg(
            func=terrain_levels_forward_traversal, params={"update_levels": False}
        )

    # specify directory for logging experiments
    log_root_path = os.path.join(Path(__file__).resolve().parents[2], "logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        try:
            resume_path = retrieve_file_path(args_cli.checkpoint)
        except FileNotFoundError:
            if os.path.isabs(args_cli.checkpoint):
                raise
            candidate_paths = []
            if agent_cfg.load_run:
                candidate_paths.append(os.path.join(log_root_path, agent_cfg.load_run, args_cli.checkpoint))
            candidate_paths.append(os.path.join(log_root_path, args_cli.checkpoint))

            for candidate_path in candidate_paths:
                try:
                    resume_path = retrieve_file_path(candidate_path)
                    break
                except FileNotFoundError:
                    continue
            else:
                raise FileNotFoundError(
                    f"Unable to find checkpoint '{args_cli.checkpoint}'. Tried direct path and: {candidate_paths}"
                )
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = _resolve_artifact_log_dir(resume_path, log_root_path)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    base_env = env.unwrapped
    set_fixed_terrain_level(base_env, args_cli.terrain_level)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
        base_env = env.unwrapped

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    torque_log_file = None
    torque_log_writer = None
    torque_log_row_count = 0
    torque_log_path = None
    torque_log_env_index = 0
    torque_joint_names: list[str] = []

    if getattr(base_env.cfg, "torque_logging_enabled", False):
        torque_log_env_index = int(getattr(base_env.cfg, "torque_log_env_index", 0))
        torque_log_flush_interval = int(getattr(base_env.cfg, "torque_log_flush_interval", 50))
        torque_log_folder_name = str(getattr(base_env.cfg, "torque_log_folder_name", "play_joint_torque"))
        torque_log_root = Path(log_dir) / torque_log_folder_name
        torque_log_root.mkdir(parents=True, exist_ok=True)
        torque_run_dir = torque_log_root / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        torque_run_dir.mkdir(parents=True, exist_ok=True)

        robot = base_env.scene["robot"]
        torque_joint_names = list(robot.joint_names)
        torque_log_path = torque_run_dir / "joint_torque.csv"
        torque_log_file = torque_log_path.open("w", encoding="utf-8", newline="")
        torque_log_writer = csv.writer(torque_log_file)
        torque_log_writer.writerow(["step", "sim_time_s", "env_index", *torque_joint_names])

        metadata_path = torque_run_dir / "metadata.txt"
        metadata_path.write_text(
            "\n".join(
                [
                    f"task={args_cli.task}",
                    f"checkpoint={resume_path}",
                    f"log_dir={log_dir}",
                    f"env_index={torque_log_env_index}",
                    f"step_dt={base_env.step_dt}",
                    f"num_joints={len(torque_joint_names)}",
                    f"joint_names={','.join(torque_joint_names)}",
                ]
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"[INFO] Joint torque logging enabled: {torque_log_path}")

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    from distillation_runner import DistillationRunner

    runner_class = DistillationRunner if agent_cfg.algorithm.class_name == "Distillation" else OnPolicyRunner
    ppo_runner = runner_class(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    ppo_runner.load(resume_path)

    # obtain the trained policy for inference
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    # extract the neural network module
    # we do this in a try-except to maintain backwards compatibility.
    try:
        # version 2.3 onwards
        policy_nn = ppo_runner.alg.policy
    except AttributeError:
        # version 2.2 and below
        policy_nn = ppo_runner.alg.actor_critic

    # export policy to onnx/jit
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_jit(policy_nn, ppo_runner.obs_normalizer, path=export_model_dir, filename="policy.pt")
    export_policy_as_onnx(
        policy_nn, normalizer=ppo_runner.obs_normalizer, path=export_model_dir, filename="policy.onnx"
    )

    dt = env.unwrapped.step_dt

    # reset environment
    obs, _ = env.get_observations()
    timestep = 0
    # simulate environment
    try:
        while simulation_app.is_running():
            start_time = time.time()
            # run everything in inference mode
            with torch.inference_mode():
                # agent stepping
                actions = policy(obs)
                # env stepping
                obs, _, dones, _ = env.step(actions)
                if policy_nn.is_recurrent:
                    policy_nn.reset(dones)

            if torque_log_writer is not None:
                robot = base_env.scene["robot"]
                applied_torque = robot.data.applied_torque[torque_log_env_index].detach().cpu().tolist()
                torque_log_writer.writerow([timestep, timestep * dt, torque_log_env_index, *applied_torque])
                torque_log_row_count += 1
                if torque_log_row_count % torque_log_flush_interval == 0:
                    torque_log_file.flush()

            timestep += 1
            if args_cli.max_steps is not None and timestep >= args_cli.max_steps:
                break
            if args_cli.video:
                # Exit the play loop after recording one video
                if timestep == args_cli.video_length:
                    break

            # time delay for real-time evaluation
            sleep_time = dt - (time.time() - start_time)
            if args_cli.real_time and sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        if hasattr(base_env, "_forward_terrain_counts"):
            print("[INFO] Fixed-terrain traversal outcomes (completed attempts only):")
            names = base_env.scene.terrain.cfg.terrain_generator.sub_terrains
            for name, counts in zip(names, base_env._forward_terrain_counts.detach().cpu().tolist()):
                attempts, successes = (int(value) for value in counts[:2])
                rate = f"{successes / attempts:.1%}" if attempts else "N/A"
                print(f"  {name}: {successes}/{attempts}, success_rate={rate}")
        if torque_log_file is not None:
            torque_log_file.flush()
            torque_log_file.close()
            print(f"[INFO] Joint torque log saved to: {torque_log_path}")
        # close the simulator
        env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
