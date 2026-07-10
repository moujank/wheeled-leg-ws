import isaaclab.terrains as terrain_gen
from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg


# Base-locked 课程学习地形。
# 设计原则：
# 1. 固定为 5 类地形，分别覆盖平地、粗糙、波浪、斜坡、台阶；
# 2. 每一类内部都提供从低到高的参数范围，便于 curriculum 沿行方向逐步增加难度；
# 3. 先学最容易稳定站立和速度跟踪的地形，再逐步引入连续起伏和离散高差。
BASE_LOCKED_TERRAINS_CFG = TerrainGeneratorCfg(
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
        # 1. 平地：课程学习起点，用于先建立基础平衡与速度跟踪能力。
        "flat": terrain_gen.MeshPlaneTerrainCfg(
            proportion=0.20,
        ),
        # 2. 粗糙地：在平地基础上加入低幅值随机高度扰动，先训练抗小扰动能力。
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.20,
            noise_range=(0.0, 0.035),
            noise_step=0.01,
            border_width=0.25,
        ),
        # 3. 波浪地：连续周期起伏，比随机粗糙更强调前后姿态协调和轮足连续接触稳定性。
        "wave": terrain_gen.HfWaveTerrainCfg(
            proportion=0.20,
            amplitude_range=(0.0, 0.12),
            num_waves=3,
            border_width=0.25,
        ),
        # 4. 斜坡地：引入持续坡度变化，训练上坡 / 下坡时的重心控制和推进分配。
        "pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.20,
            slope_range=(0.0, 0.22),
            platform_width=2.5,
            border_width=0.25,
        ),
        # 5. 台阶地：离散高差通常最难，放在课程学习后段训练更合理。
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.20,
            step_height_range=(0.0, 0.06),
            step_width=0.40,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
    },
)


# 更高难度的 base-locked 课程学习地形。
# 保持与标准版相同的 5 类地形和训练顺序，只提升难度上限，
# 这样可以复用同一套课程学习逻辑，而不是切换到完全不同的地形分布。
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
        # 1. 平地：即使在 hard 配置中也保留少量最简单样本，稳定课程学习前期收敛。
        "flat": terrain_gen.MeshPlaneTerrainCfg(
            proportion=0.12,
        ),
        # 2. 粗糙地：提高随机起伏高度，增强对局部突变接触的鲁棒性要求。
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.22,
            noise_range=(0.0, 0.06),
            noise_step=0.01,
            border_width=0.25,
        ),
        # 3. 波浪地：提高振幅和波频，增加连续起伏对机体俯仰 / 轮地接触的扰动。
        "wave": terrain_gen.HfWaveTerrainCfg(
            proportion=0.22,
            amplitude_range=(0.0, 0.18),
            num_waves=4,
            border_width=0.25,
        ),
        # 4. 斜坡地：提高最大坡度，强化持续重力分量下的推进与姿态稳定能力。
        "pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.22,
            slope_range=(0.0, 0.35),
            platform_width=2.0,
            border_width=0.25,
        ),
        # 5. 台阶地：提高台阶高度并缩窄步宽，是 hard 配置中的主要难点之一。
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.22,
            step_height_range=(0.0, 0.10),
            step_width=0.35,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
    },
)
