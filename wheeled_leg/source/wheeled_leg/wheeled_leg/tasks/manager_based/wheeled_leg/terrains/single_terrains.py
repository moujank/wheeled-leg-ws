"""单地形配置集合。

该文件将仓库里当前官方训练配置中实际会生成的地形类型拆成“一个配置只包含一种地形”。
这样做的目的有两个：
1. 便于单独训练 / 调试某一种地形；
2. 便于明确观察每种 terrain 参数本身对策略的影响。

说明：
- 这里的“官方代码可能生成的单独地形”按本仓库当前 terrain 配置实际使用到的类型整理；
- 保持和现有 terrain 模块一致，使用 ``TerrainGeneratorCfg`` 常量而不是自定义包装类；
- 每个 ``sub_terrains`` 字典中只保留一个子地形条目。
"""

import isaaclab.terrains as terrain_gen
from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg


# 平地：完全平坦的平面地形，适合做功能验证、动作接口检查和最基础训练。
FLAT_SINGLE_TERRAIN_CFG = TerrainGeneratorCfg(
    # 是否按课程学习方式沿着网格逐步提升难度。平地一般不需要课程学习。
    curriculum=False,
    # 每个子地形 patch 的尺寸，单位为米，格式为 (长, 宽)。
    size=(8.0, 8.0),
    # 整个 terrain 网格外围额外留出的安全边界宽度，避免机器人直接走出地形区域。
    border_width=10.0,
    # terrain 网格的行数。单地形时保留多行，便于并行复制相同地形。
    num_rows=10,
    # terrain 网格的列数。单地形时保留多列，便于并行复制相同地形。
    num_cols=10,
    # 高度场在水平方向的离散分辨率，越小表示网格越细。
    horizontal_scale=0.1,
    # 高度场在竖直方向的离散分辨率，越小表示高度变化越精细。
    vertical_scale=0.005,
    # 将过陡坡面裁成垂直表面的阈值，主要用于高度场转网格时控制可行走边界。
    slope_threshold=0.75,
    # 是否启用 terrain 缓存。调参数阶段通常关闭，避免旧缓存干扰。
    use_cache=False,
    sub_terrains={
        "flat": terrain_gen.MeshPlaneTerrainCfg(
            # 该子地形在混合采样时所占比例。单地形配置里固定为 1.0。
            proportion=1.0,
        ),
    },
)


# 金字塔斜坡：中心附近有平台，向四周形成斜坡，适合测试上坡 / 下坡通过能力。
PYRAMID_SLOPE_SINGLE_TERRAIN_CFG = TerrainGeneratorCfg(
    # 是否按课程学习方式沿着网格逐步提升难度。单坡地默认关闭。
    curriculum=False,
    # 每个子地形 patch 的尺寸，单位为米，格式为 (长, 宽)。
    size=(8.0, 8.0),
    # 整个 terrain 网格外围额外留出的安全边界宽度。
    border_width=10.0,
    # terrain 网格行数。
    num_rows=10,
    # terrain 网格列数。
    num_cols=10,
    # 高度场水平方向离散分辨率。
    horizontal_scale=0.1,
    # 高度场竖直方向离散分辨率。
    vertical_scale=0.005,
    # 坡面裁切阈值。
    slope_threshold=0.75,
    # 是否启用 terrain 缓存。
    use_cache=False,
    sub_terrains={
        "pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            # 该子地形在混合采样时所占比例。单地形配置里固定为 1.0。
            proportion=1.0,
            # 坡度范围，课程学习开启时会在该范围内分布难度。
            slope_range=(0.0, 0.35),
            # 中央平台宽度，单位米。平台越大，机器人初始立足区域越宽裕。
            platform_width=2.0,
            # 子地形局部边界宽度，避免坡面直接贴边。
            border_width=0.25,
        ),
    },
)


# 反金字塔斜坡：中心是向下或向内收的反坡结构，适合测试下沉 / 脱困 / 姿态恢复能力。
INVERTED_PYRAMID_SLOPE_SINGLE_TERRAIN_CFG = TerrainGeneratorCfg(
    # 是否按课程学习方式沿着网格逐步提升难度。单反坡默认关闭。
    curriculum=False,
    # 每个子地形 patch 的尺寸，单位为米，格式为 (长, 宽)。
    size=(8.0, 8.0),
    # 整个 terrain 网格外围额外留出的安全边界宽度。
    border_width=10.0,
    # terrain 网格行数。
    num_rows=10,
    # terrain 网格列数。
    num_cols=10,
    # 高度场水平方向离散分辨率。
    horizontal_scale=0.1,
    # 高度场竖直方向离散分辨率。
    vertical_scale=0.005,
    # 坡面裁切阈值。
    slope_threshold=0.75,
    # 是否启用 terrain 缓存。
    use_cache=False,
    sub_terrains={
        "pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            # 该子地形在混合采样时所占比例。单地形配置里固定为 1.0。
            proportion=1.0,
            # 反坡坡度范围，值越大，中心附近的高差变化越激烈。
            slope_range=(0.0, 0.35),
            # 中央平台宽度，单位米，用于控制中心相对平坦区域大小。
            platform_width=2.0,
            # 子地形局部边界宽度，避免反坡结构紧贴边缘。
            border_width=0.25,
        ),
    },
)


# 波浪地形：地表呈周期起伏，适合测试连续小幅高度扰动下的跟踪与稳定性。
WAVE_SINGLE_TERRAIN_CFG = TerrainGeneratorCfg(
    # 是否按课程学习方式沿着网格逐步提升难度。波浪地形默认关闭。
    curriculum=False,
    # 每个子地形 patch 的尺寸，单位为米，格式为 (长, 宽)。
    size=(8.0, 8.0),
    # 整个 terrain 网格外围额外留出的安全边界宽度。
    border_width=10.0,
    # terrain 网格行数。
    num_rows=10,
    # terrain 网格列数。
    num_cols=10,
    # 高度场水平方向离散分辨率。
    horizontal_scale=0.1,
    # 高度场竖直方向离散分辨率。
    vertical_scale=0.005,
    # 坡面裁切阈值。
    slope_threshold=0.75,
    # 是否启用 terrain 缓存。
    use_cache=False,
    sub_terrains={
        "wave": terrain_gen.HfWaveTerrainCfg(
            # 该子地形在混合采样时所占比例。单地形配置里固定为 1.0。
            proportion=1.0,
            # 波浪振幅范围，决定波峰与波谷的高度差。
            amplitude_range=(0.0, 0.18),
            # 波的数量，影响一个 patch 内波峰波谷的频率与密度。
            num_waves=4,
            # 子地形局部边界宽度，避免波浪结构直接贴边。
            border_width=0.25,
        ),
    },
)


# 随机粗糙地形：在平面上叠加离散随机高度噪声，适合测试抗随机扰动与足端容错能力。
RANDOM_ROUGH_SINGLE_TERRAIN_CFG = TerrainGeneratorCfg(
    # 是否按课程学习方式沿着网格逐步提升难度。随机粗糙地默认关闭。
    curriculum=False,
    # 每个子地形 patch 的尺寸，单位为米，格式为 (长, 宽)。
    size=(8.0, 8.0),
    # 整个 terrain 网格外围额外留出的安全边界宽度。
    border_width=10.0,
    # terrain 网格行数。
    num_rows=10,
    # terrain 网格列数。
    num_cols=10,
    # 高度场水平方向离散分辨率。
    horizontal_scale=0.1,
    # 高度场竖直方向离散分辨率。
    vertical_scale=0.005,
    # 坡面裁切阈值。
    slope_threshold=0.75,
    # 是否启用 terrain 缓存。
    use_cache=False,
    sub_terrains={
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            # 该子地形在混合采样时所占比例。单地形配置里固定为 1.0。
            proportion=1.0,
            # 随机高度噪声范围，单位米，决定粗糙度上下界。
            noise_range=(0.0, 0.06),
            # 噪声离散步长，越大表示高度值更稀疏，越小表示粗糙变化更细。
            noise_step=0.02,
            # 子地形局部边界宽度，避免粗糙块直接贴边。
            border_width=0.25,
        ),
    },
)


# 金字塔台阶：围绕中央平台生成逐级抬升的台阶，适合测试离散高差和连续跨步能力。
PYRAMID_STAIRS_SINGLE_TERRAIN_CFG = TerrainGeneratorCfg(
    # 台阶地形更适合配合课程学习逐步增加高度差，因此这里默认开启 curriculum。
    curriculum=True,
    # 每个子地形 patch 的尺寸，单位为米，格式为 (长, 宽)。
    size=(8.0, 8.0),
    # 整个 terrain 网格外围额外留出的安全边界宽度。
    border_width=10.0,
    # terrain 网格行数。
    num_rows=10,
    # terrain 网格列数。
    num_cols=10,
    # 高度场水平方向离散分辨率。
    horizontal_scale=0.1,
    # 高度场竖直方向离散分辨率。
    vertical_scale=0.005,
    # 坡面裁切阈值。
    slope_threshold=0.75,
    # 是否启用 terrain 缓存。
    use_cache=False,
    sub_terrains={
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            # 该子地形在混合采样时所占比例。单地形配置里固定为 1.0。
            proportion=1.0,
            # 台阶高度范围，课程学习时会从低台阶逐步增加到高台阶。
            step_height_range=(0.0, 0.08),
            # 单级台阶的水平宽度，单位米。越宽越容易落脚，越窄越考验步态切换。
            step_width=0.35,
            # 中央平台宽度，单位米，用于提供初始站立或过渡区域。
            platform_width=3.0,
            # 台阶结构外围额外边界宽度，降低边缘碰撞与跌落风险。
            border_width=1.0,
            # 是否在台阶里打洞。False 表示实体连续台阶；True 会生成带空洞的更难版本。
            holes=False,
        ),
    },
)


# 反金字塔台阶：台阶向中心逐级下沉，适合测试下台阶、坑状结构和恢复能力。
INVERTED_PYRAMID_STAIRS_SINGLE_TERRAIN_CFG = TerrainGeneratorCfg(
    # 反台阶同样适合课程学习，便于逐步增加下沉台阶的难度。
    curriculum=True,
    # 每个子地形 patch 的尺寸，单位为米，格式为 (长, 宽)。
    size=(8.0, 8.0),
    # 整个 terrain 网格外围额外留出的安全边界宽度。
    border_width=10.0,
    # terrain 网格行数。
    num_rows=10,
    # terrain 网格列数。
    num_cols=10,
    # 高度场水平方向离散分辨率。
    horizontal_scale=0.1,
    # 高度场竖直方向离散分辨率。
    vertical_scale=0.005,
    # 坡面裁切阈值。
    slope_threshold=0.75,
    # 是否启用 terrain 缓存。
    use_cache=False,
    sub_terrains={
        "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            # 该子地形在混合采样时所占比例。单地形配置里固定为 1.0。
            proportion=1.0,
            # 台阶高度范围，决定每一级向下的高差幅度。
            step_height_range=(0.0, 0.08),
            # 单级台阶的水平宽度，单位米，影响下台阶时的落脚空间。
            step_width=0.35,
            # 中央平台宽度，单位米，用于控制中心区域大小。
            platform_width=3.0,
            # 台阶结构外围额外边界宽度，避免结构太靠近 patch 边缘。
            border_width=1.0,
            # 是否生成带孔洞的版本。当前关闭，保持为标准实体反台阶。
            holes=False,
        ),
    },
)


__all__ = [
    "FLAT_SINGLE_TERRAIN_CFG",
    "PYRAMID_SLOPE_SINGLE_TERRAIN_CFG",
    "INVERTED_PYRAMID_SLOPE_SINGLE_TERRAIN_CFG",
    "WAVE_SINGLE_TERRAIN_CFG",
    "RANDOM_ROUGH_SINGLE_TERRAIN_CFG",
    "PYRAMID_STAIRS_SINGLE_TERRAIN_CFG",
    "INVERTED_PYRAMID_STAIRS_SINGLE_TERRAIN_CFG",
]
