"""ROS-ready data interfaces without importing ROS runtime packages."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol


class CommandFrame(str, Enum):
    BODY = "body"
    WORLD = "world"


@dataclass(frozen=True)
class RobotCommand:
    """Command object used by future ROS and non-ROS dispatchers."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    frame: CommandFrame = CommandFrame.BODY
    task_name: str = "wheeled_base_locked"


@dataclass(frozen=True)
class RobotState:
    """Minimal state packet for policy and supervisor nodes."""

    time_s: float
    base_position: tuple[float, float, float]
    base_rpy: tuple[float, float, float]
    base_linear_velocity: tuple[float, float, float]
    base_angular_velocity: tuple[float, float, float]
    joint_position: tuple[float, ...]
    joint_velocity: tuple[float, ...]


class CommandConsumer(Protocol):
    def set_command(self, command: RobotCommand) -> None:
        """Set the currently requested robot command."""


class StatePublisher(Protocol):
    def publish_state(self, state: RobotState) -> None:
        """Publish a robot state packet."""


@dataclass(frozen=True)
class RosNodeSpec:
    """Future ROS node metadata."""

    name: str
    role: str
    inputs: tuple[str, ...] = field(default_factory=tuple)
    outputs: tuple[str, ...] = field(default_factory=tuple)
    implemented: bool = False


def default_node_specs() -> tuple[RosNodeSpec, ...]:
    return (
        RosNodeSpec(
            name="mujoco_state_publisher",
            role="publish MuJoCo robot state",
            outputs=("/mujoco/robot_state", "/tf"),
        ),
        RosNodeSpec(
            name="mujoco_command_bridge",
            role="receive body-frame velocity and task commands",
            inputs=("/cmd_vel", "/mujoco/task_command"),
            outputs=("/mujoco/robot_command",),
        ),
        RosNodeSpec(
            name="mujoco_policy_runner",
            role="run exported policy from simulation state",
            inputs=("/mujoco/robot_state", "/mujoco/robot_command"),
            outputs=("/mujoco/action",),
        ),
        RosNodeSpec(
            name="mujoco_task_supervisor",
            role="switch or blend locomotion, recovery, grasp, and obstacle tasks",
            inputs=("/mujoco/task_command", "/mujoco/robot_state"),
            outputs=("/mujoco/active_task",),
        ),
    )
