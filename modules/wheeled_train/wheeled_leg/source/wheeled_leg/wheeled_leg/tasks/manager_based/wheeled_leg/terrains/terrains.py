"""Compatibility aliases for task-specific terrain configurations."""

from .arm_free_locomotion import ARM_FREE_LOCOMOTION_TERRAINS_CFG
from .base_locked import BASE_LOCKED_TERRAINS_CFG
from .loaded_grasp_obstacle import LOADED_GRASP_OBSTACLE_TERRAINS_CFG
from .obstacle import OBSTACLE_TERRAINS_CFG

# Old names kept for existing environment files during the migration.
GRAVEL_TERRAINS_CFG = BASE_LOCKED_TERRAINS_CFG
ROUGH_TERRAINS_CFG = ARM_FREE_LOCOMOTION_TERRAINS_CFG
PYRAMID_TERRAINS_CFG = OBSTACLE_TERRAINS_CFG
