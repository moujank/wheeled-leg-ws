"""Sampling phases for low-speed traversal. No changes to robot or v1 settings."""

# Exact numbers of columns out of 24, in canonical terrain-family order.
PHASE_COLUMNS = {
    'foundation': (3, 3, 3, 3, 3, 3, 3, 3),
    'focus': (2, 1, 1, 3, 5, 6, 1, 5),
    'retention': (3, 2, 2, 3, 4, 4, 2, 4),
}
TERRAIN_NAMES = ('flat', 'slope', 'slope_inverted', 'pyramid_stairs',
                 'pyramid_stairs_inverted', 'rough', 'wave', 'discrete_obstacles')


def configure_traversal_phase(cfg, phase):
    """Keep geometric difficulty and observation/action interfaces comparable."""
    if phase not in PHASE_COLUMNS:
        raise ValueError(f'Unknown traversal phase: {phase}')
    command = cfg.commands.base_velocity
    if not hasattr(command, 'stage_max_speeds'):
        raise ValueError('Traversal phases require the Terrains task')
    generator = cfg.scene.terrain.terrain_generator
    if tuple(generator.sub_terrains) != TERRAIN_NAMES:
        raise ValueError('Traversal phases require all eight canonical terrain families')
    generator.num_cols = 24
    for name, columns in zip(TERRAIN_NAMES, PHASE_COLUMNS[phase], strict=True):
        generator.sub_terrains[name].proportion = columns / 24
    cfg.traversal_phase = phase
    # Start gently even when loading a mature policy; promotion proves ability
    # under the new reward/command distribution. No geometry is made easier.
    cfg.scene.terrain.max_init_terrain_level = 0
