# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Gentle terrain mixture for learning wheeled-leg balance before harder obstacles."""

import isaaclab.terrains as terrain_gen
from isaaclab.terrains.height_field.hf_terrains import random_uniform_terrain
from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg


def curriculum_rough_terrain(difficulty: float, cfg: terrain_gen.HfRandomUniformTerrainCfg):
    """Scale roughness with difficulty; the built-in random terrain ignores difficulty."""
    # Use noise_range as the minimum/maximum amplitude of this curriculum.
    amplitude = cfg.noise_range[0] + difficulty * (cfg.noise_range[1] - cfg.noise_range[0])
    return random_uniform_terrain(difficulty, cfg.replace(noise_range=(-amplitude, amplitude)))


TERRAIN_BALANCE_TERRAINS_CFG = TerrainGeneratorCfg(
    seed=42,
    curriculum=True,
    size=(8.0, 8.0),
    border_width=10.0,
    num_rows=8,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.0025,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.30),
        "slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.15, slope_range=(0.0, 0.12), platform_width=1.0, border_width=0.25,
        ),
        "slope_inverted": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.15, slope_range=(0.0, 0.12), platform_width=1.0, border_width=0.25,
        ),
        "rough": terrain_gen.HfRandomUniformTerrainCfg(
            function=curriculum_rough_terrain,
            proportion=0.20,
            noise_range=(0.0, 0.015),
            noise_step=0.0025,
            downsampled_scale=0.25,
            border_width=0.25,
        ),
        "wave": terrain_gen.HfWaveTerrainCfg(
            proportion=0.10, amplitude_range=(0.0, 0.04), num_waves=2, border_width=0.25,
        ),
        "low_steps": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.05,
            step_height_range=(0.005, 0.04),
            step_width=0.5,
            platform_width=1.5,
            border_width=0.5,
            holes=False,
        ),
        "low_steps_inverted": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.05,
            step_height_range=(0.005, 0.04),
            step_width=0.5,
            platform_width=1.5,
            border_width=0.5,
            holes=False,
        ),
    },
)
