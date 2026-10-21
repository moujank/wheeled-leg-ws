"""RSL-RL 2.3 distillation with a fixed teacher and its observation statistics."""

from copy import deepcopy
from pathlib import Path

import torch
from rsl_rl.runners import OnPolicyRunner


class DistillationRunner(OnPolicyRunner):
    """Keep the loaded PPO actor's physical input/output mapping fixed.

    The installed OnPolicyRunner supports StudentTeacher, but trains both
    normalizers and passes raw observations to the first rollout action.
    Handle those two details locally without changing the installed package.
    """

    def __init__(self, env, train_cfg, log_dir=None, device="cpu"):
        train_cfg = deepcopy(train_cfg)
        if train_cfg["algorithm"]["class_name"] != "Distillation":
            raise ValueError("DistillationRunner requires the Distillation algorithm.")
        # RSL-RL 2.3 StudentTeacher only implements scalar std; Isaac Lab's
        # config also serializes noise_std_type, which that module ignores.
        if train_cfg["policy"].pop("noise_std_type", "scalar") != "scalar":
            raise ValueError("This StudentTeacher version requires scalar action noise.")
        steps = train_cfg["num_steps_per_env"]
        gradient_length = train_cfg["algorithm"]["gradient_length"]
        if gradient_length <= 0 or steps % gradient_length:
            raise ValueError("gradient_length must be positive and divide num_steps_per_env.")
        # StudentTeacher 2.3 assigns False to this class method instead of
        # calling it. Restore the descriptor so later PPO construction works.
        normal_class = torch.distributions.Normal
        validation_descriptor = normal_class.__dict__.get("set_default_validate_args")
        try:
            super().__init__(env, train_cfg, log_dir=log_dir, device=device)
        finally:
            if validation_descriptor is not None:
                normal_class.set_default_validate_args = validation_descriptor
            elif "set_default_validate_args" in normal_class.__dict__:
                delattr(normal_class, "set_default_validate_args")
        if self.privileged_obs_type != "teacher":
            raise ValueError("Distillation requires an explicit 'teacher' observation group.")
        self._freeze_teacher()

    def _freeze_teacher(self):
        self.alg.policy.teacher.requires_grad_(False)
        self.alg.policy.teacher.eval()
        self.privileged_obs_normalizer.eval()

    def train_mode(self):
        super().train_mode()
        self._freeze_teacher()

    def _validate_checkpoint(self, path, *, teacher):
        checkpoint = torch.load(Path(path).expanduser(), map_location="cpu", weights_only=False)
        state = checkpoint["model_state_dict"]
        prefix = "actor." if teacher else "student."
        if not any(key.startswith(prefix) for key in state):
            flag = "--teacher_checkpoint needs a PPO actor" if teacher else "--resume/play needs a distilled student"
            raise ValueError(f"{flag} checkpoint: {path}")
        target = self.alg.policy.teacher.state_dict() if teacher else self.alg.policy.state_dict()
        source = {key.removeprefix("actor."): value for key, value in state.items()
                  if key.startswith("actor.")} if teacher else state
        if source.keys() != target.keys() or any(source[key].shape != target[key].shape for key in target):
            raise ValueError(
                f"Incompatible {'teacher PPO actor' if teacher else 'distilled student'} network in {path}. "
                "Observation order/dimensions, action dimensions and hidden layers must match the task. "
                "Current v1 uses 34 actor inputs (including joint_effort); legacy 28-input checkpoints "
                "require their original observation configuration."
            )
        if self.empirical_normalization:
            required = ["obs_norm_state_dict"]
            if not teacher:
                required.append("privileged_obs_norm_state_dict")
            if any(key not in checkpoint for key in required):
                raise ValueError(f"Checkpoint lacks required observation normalization statistics: {path}")

    def load_teacher(self, path):
        """Start a fresh student from a PPO actor and its actor normalizer."""
        self._validate_checkpoint(path, teacher=True)
        infos = super().load(str(Path(path).expanduser()), load_optimizer=False)
        self._freeze_teacher()
        return infos

    def load(self, path, load_optimizer=True):
        """Resume/evaluate a student, restoring both networks and normalizers."""
        self._validate_checkpoint(path, teacher=False)
        infos = super().load(str(Path(path).expanduser()), load_optimizer=load_optimizer)
        self._freeze_teacher()
        return infos

    def learn(self, num_learning_iterations, init_at_random_ep_len=False):
        # All later rollout observations are normalized by OnPolicyRunner.
        # Normalize only its initial action inputs here, including after resume.
        original_act = self.alg.act
        first_action = True

        def act(obs, teacher_obs):
            nonlocal first_action
            if first_action:
                obs = self.obs_normalizer(obs.to(self.device))
                teacher_obs = self.privileged_obs_normalizer(teacher_obs.to(self.device))
                first_action = False
            return original_act(obs, teacher_obs)

        self.alg.act = act
        try:
            return super().learn(num_learning_iterations, init_at_random_ep_len)
        finally:
            self.alg.act = original_act
