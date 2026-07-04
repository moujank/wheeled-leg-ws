import isaaclab.terrains as terrain_gen
from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg


ARM_FREE_LOCOMOTION_TERRAINS_CFG = TerrainGeneratorCfg(
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
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.25),
        "pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.2, slope_range=(0.0, 0.35), platform_width=2.0, border_width=0.25
        ),
        "pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.2, slope_range=(0.0, 0.35), platform_width=2.0, border_width=0.25
        ),
        "wave": terrain_gen.HfWaveTerrainCfg(
            proportion=0.15, amplitude_range=(0.0, 0.15), num_waves=4, border_width=0.25
        ),
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.2, noise_range=(0.0, 0.05), noise_step=0.02, border_width=0.25
        ),
    },
)

