# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Only inverted pyramid stairs and positive random-height square blocks."""

import numpy as np
import trimesh

from isaaclab.terrains import MeshInvertedPyramidStairsTerrainCfg, SubTerrainBaseCfg, TerrainGeneratorCfg
from isaaclab.terrains.trimesh.mesh_terrains import inverted_pyramid_stairs_terrain
from isaaclab.utils import configclass


def obstacle_height(difficulty, height_range=(0.04, 0.20)):
    # Quantize to eight rows so the top training/evaluation row is exactly 20 cm.
    level = min(max(int(difficulty * 8), 0), 7)
    return height_range[0] + (height_range[1] - height_range[0]) * level / 7.0


def inverted_stairs_20cm(difficulty, cfg):
    height = obstacle_height(difficulty, cfg.step_height_range)
    return inverted_pyramid_stairs_terrain(difficulty, cfg.replace(step_height_range=(height, height)))


def random_height_squares(difficulty, cfg):
    height = obstacle_height(difficulty, cfg.height_range)
    floor = trimesh.creation.box(
        (cfg.size[0], cfg.size[1], 0.10),
        trimesh.transformations.translation_matrix((cfg.size[0] / 2, cfg.size[1] / 2, -0.05)),
    )
    meshes = [floor]
    center = np.array(cfg.size) / 2
    cells = (np.array(cfg.size) / cfg.cell_size).astype(int)
    for i in range(cells[0]):
        for j in range(cells[1]):
            xy = (np.array((i, j)) + 0.5) * cfg.cell_size
            # Leave a genuinely level reset platform (not a maximum-height origin).
            if np.all(np.abs(xy - center) < cfg.platform_width / 2 + cfg.cell_size / 2):
                continue
            z = height if (i + j) % 4 == 0 else np.random.uniform(0.25 * height, height)
            meshes.append(trimesh.creation.box(
                (cfg.cell_size - cfg.gap, cfg.cell_size - cfg.gap, z),
                trimesh.transformations.translation_matrix((xy[0], xy[1], z / 2)),
            ))
    return meshes, np.array((center[0], center[1], 0.0))


@configclass
class RandomHeightSquaresTerrainCfg(SubTerrainBaseCfg):
    function = random_height_squares
    height_range: tuple[float, float] = (0.04, 0.20)
    cell_size: float = 0.55
    gap: float = 0.02
    platform_width: float = 1.6


OBSTACLE_JUMP_TERRAINS_CFG = TerrainGeneratorCfg(
    seed=43, curriculum=True, size=(8.0, 8.0), border_width=5.0,
    num_rows=8, num_cols=20, horizontal_scale=0.05, vertical_scale=0.005,
    use_cache=False,
    sub_terrains={
        "inverted_pyramid": MeshInvertedPyramidStairsTerrainCfg(
            function=inverted_stairs_20cm, proportion=0.5,
            step_height_range=(0.04, 0.20), step_width=0.55,
            platform_width=1.8, border_width=0.3, holes=False,
        ),
        "random_height_squares": RandomHeightSquaresTerrainCfg(proportion=0.5),
    },
)
