"""Local compatibility re-exports from the shared trajectory package."""

from quad_trajectories import (
    HARDWARE_HEIGHT as HW_HEIGHT,
    SIM_HEIGHT,
    TRAJ_REGISTRY,
    TrajContext,
    TrajectoryType,
    fig8_contraction,
    fig8_heading_contraction as fig8_heading,
    hover_contraction as hover,
    spiral_contraction as spiral,
    trefoil_contraction as trefoil,
)

__all__ = [
    "SIM_HEIGHT",
    "HW_HEIGHT",
    "TrajContext",
    "TrajectoryType",
    "TRAJ_REGISTRY",
    "hover",
    "spiral",
    "fig8_contraction",
    "trefoil",
    "fig8_heading",
]
