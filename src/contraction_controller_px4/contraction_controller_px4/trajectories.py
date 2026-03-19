"""Local compatibility re-exports from the shared trajectory package."""

from quad_trajectories import (
    HARDWARE_HEIGHT as HW_HEIGHT,
    SIM_HEIGHT,
    TRAJ_REGISTRY,
    TrajContext,
    TrajectoryType,
    f8_contraction,
    fig8_heading_contraction as fig8_heading,
    figure_eight_contraction as figure_eight,
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
    "figure_eight",
    "trefoil",
    "fig8_heading",
    "f8_contraction",
]
