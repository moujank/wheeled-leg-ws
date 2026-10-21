# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Curriculum terrain mixture for the v2 wheeled-leg high-speed terrain task."""

import isaaclab.terrains as terrain_gen
from isaaclab.terrains.height_field.hf_terrains import random_uniform_terrain
from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg


def curriculum_rough_terrain(difficulty: float, cfg: terrain_gen.HfRandomUniformTerrainCfg):
    """Increase roughness from a flat surface to the configured maximum."""
    amplitude = cfg.noise_range[0] + difficulty * (cfg.noise_range[1] - cfg.noise_range[0])
    return random_uniform_terrain(difficulty, cfg.replace(noise_range=(-amplitude, amplitude)))


# Eight terrain families are kept in one generator so every environment sees the same
# curriculum rows while the columns provide a balanced mixture of terrain geometry.
# The maximum geometric excursion is approximately 0.20 m at difficulty=1.0:
# slopes use 0.05 * (8 m / 2), waves use +/-0.10 m, roughness uses +/-0.10 m,
# stairs reach about 0.20 m over the generated steps, and obstacles are +/-0.20 m.
WHEELED_LEG_TERRAINS_CFG = TerrainGeneratorCfg(
    seed=43,
    curriculum=True,
    size=(8.0, 8.0),
    border_width=10.0,
    num_rows=10,
    num_cols=24,
    horizontal_scale=0.1,
    vertical_scale=0.0025,
    slope_threshold=0.75,
    difficulty_range=(0.0, 1.0),
    use_cache=False,
    sub_terrains={
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.12),
        "slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.12,
            slope_range=(0.0, 0.05),
            platform_width=1.5,
            border_width=0.25,
        ),
        "slope_inverted": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.12,
            slope_range=(0.0, 0.05),
            platform_width=1.5,
            border_width=0.25,
        ),
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.14,
            step_height_range=(0.0, 0.03),
            step_width=0.5,
            platform_width=1.5,
            border_width=0.5,
            holes=False,
        ),
        "pyramid_stairs_inverted": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.14,
            step_height_range=(0.0, 0.03),
            step_width=0.5,
            platform_width=1.5,
            border_width=0.5,
            holes=False,
        ),
        "rough": terrain_gen.HfRandomUniformTerrainCfg(
            function=curriculum_rough_terrain,
            proportion=0.12,
            noise_range=(0.0, 0.10),
            noise_step=0.01,
            downsampled_scale=0.25,
            border_width=0.25,
        ),
        "wave": terrain_gen.HfWaveTerrainCfg(
            proportion=0.12,
            amplitude_range=(0.0, 0.10),
            num_waves=4,
            border_width=0.25,
        ),
        "discrete_obstacles": terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.12,
            obstacle_height_mode="choice",
            obstacle_width_range=(0.25, 0.75),
            obstacle_height_range=(0.0, 0.20),
            num_obstacles=20,
            platform_width=1.5,
            border_width=0.25,
        ),
    },
)


__all__ = ["WHEELED_LEG_TERRAINS_CFG", "curriculum_rough_terrain"]
