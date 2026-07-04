import isaaclab.terrains as terrain_gen
from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg


BASE_LOCKED_TERRAINS_CFG = TerrainGeneratorCfg(
    curriculum=False,
    size=(8.0, 8.0),
    border_width=10.0,
    num_rows=10,
    num_cols=10,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.4, noise_range=(0.0, 0.02), noise_step=0.02, border_width=0.25
        ),
        "pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.3, slope_range=(0.0, 0.2), platform_width=2.0, border_width=0.25
        ),
        "pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.3, slope_range=(0.0, 0.2), platform_width=2.0, border_width=0.25
        ),
    },
)


BASE_LOCKED_HARD_TERRAINS_CFG = TerrainGeneratorCfg(
    curriculum=True,
    size=(8.0, 8.0),
    border_width=10.0,
    num_rows=10,
    num_cols=10,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.25, noise_range=(0.0, 0.06), noise_step=0.02, border_width=0.25
        ),
        "pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.2, slope_range=(0.0, 0.35), platform_width=2.0, border_width=0.25
        ),
        "pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.2, slope_range=(0.0, 0.35), platform_width=2.0, border_width=0.25
        ),
        "wave": terrain_gen.HfWaveTerrainCfg(
            proportion=0.15, amplitude_range=(0.0, 0.18), num_waves=4, border_width=0.25
        ),
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.1,
            step_height_range=(0.0, 0.08),
            step_width=0.35,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.1,
            step_height_range=(0.0, 0.08),
            step_width=0.35,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
    },
)
