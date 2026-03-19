"""Compatibility wrapper over the shared quad_trajectories package.

This package preserves the legacy contraction-workspace trajectory names while
delegating all implementations to quad_trajectories so there is only one real
trajectory source of truth.
"""

import enum
from typing import Dict

import jax.numpy as jnp

from quad_trajectories import (
    HARDWARE_HEIGHT,
    SIM_HEIGHT,
    TrajContext,
    figure_eight_contraction,
    fig8_heading_contraction,
    hover_contraction,
    spiral_contraction,
    trefoil_contraction,
)

try:
    from enum import StrEnum
except ImportError:
    class StrEnum(str, enum.Enum):
        """Minimal StrEnum compatible with stdlib's StrEnum."""
        pass


HW_HEIGHT = HARDWARE_HEIGHT


class TrajectoryType(StrEnum):
    """Legacy contraction-workspace trajectory names."""
    HOVER = "hover"
    SPIRAL = "spiral"
    FIGURE_EIGHT = "figure_eight"
    TREFOIL = "trefoil"
    FIG8_HEADING = "fig8_heading"
    F8_CONTRACTION = "f8_contraction"


hover = hover_contraction
spiral = spiral_contraction
figure_eight = figure_eight_contraction
trefoil = trefoil_contraction
fig8_heading = fig8_heading_contraction
f8_contraction = figure_eight_contraction

TRAJ_REGISTRY: Dict[TrajectoryType, object] = {
    TrajectoryType.HOVER: hover,
    TrajectoryType.SPIRAL: spiral,
    TrajectoryType.FIGURE_EIGHT: figure_eight,
    TrajectoryType.TREFOIL: trefoil,
    TrajectoryType.FIG8_HEADING: fig8_heading,
    TrajectoryType.F8_CONTRACTION: f8_contraction,
}


__all__ = [
    "SIM_HEIGHT",
    "HW_HEIGHT",
    "TrajContext",
    "TrajectoryType",
    "TRAJ_REGISTRY",
    "hover",
    "spiral",
    "figure_eight",
    "trefoil",
    "fig8_heading",
    "f8_contraction",
    "jnp",
]
