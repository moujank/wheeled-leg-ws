"""Fixed terrain evaluation configuration, without simulator imports."""

import math


def configure_terrain_evaluation(env_cfg, level=None, speed=None):
    """Freeze curricula and optionally select a row and an exact forward target.

    The speed is reached through the normal acceleration ramp. These overrides
    deliberately require the terrain command rather than silently altering v1.
    """
    if level is None and speed is None:
        return
    command = getattr(env_cfg.commands, "base_velocity", None)
    if command is None or not hasattr(command, "stage_max_speeds"):
        raise ValueError("--terrain_level/--terrain_speed require the Wheeled-Leg-Terrains task")
    generator = env_cfg.scene.terrain.terrain_generator
    if level is not None and not 0 <= level < generator.num_rows:
        raise ValueError(f"--terrain_level must be in [0, {generator.num_rows - 1}]")
    if speed is not None and (not math.isfinite(speed) or not .20 < speed <= .40):
        raise ValueError("--terrain_speed must be greater than .20 and at most .40 m/s")
    for name in ("terrain_levels", "terrain_distribution", "speed_levels"):
        setattr(env_cfg.curriculum, name, None)
    command.rel_standing_envs = 0.0
    if level is not None:
        env_cfg.scene.terrain.max_init_terrain_level = level
    if speed is not None:
        command.min_forward_speed = speed
        command.stage_max_speeds = (speed,)
        command.initial_stage = command.final_stage = 0
        command.hard_terrain_max_speed = speed
        command.ranges.lin_vel_x = (speed, speed)


def set_fixed_terrain_level(env, level):
    """Set origins before the wrapper's first reset; retain terrain type columns."""
    if level is None:
        return
    terrain = env.scene.terrain
    terrain.terrain_levels[:] = level
    terrain.env_origins[:] = terrain.terrain_origins[level, terrain.terrain_types]
    env.scene.env_origins[:] = terrain.env_origins
