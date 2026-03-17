"""Shared trajectory definitions for quadrotor controllers.

Each function has signature (t: float, ctx: TrajContext) -> jnp.ndarray[4]
returning [px, py, pz, psi] in NED (z negative = above ground).
Derivatives are computed externally via jax.jacfwd as needed by flat_to_x_u.
"""

import enum
from dataclasses import dataclass
from typing import Dict, Optional

import jax.numpy as jnp

try:
    from enum import StrEnum
except ImportError:
    class StrEnum(str, enum.Enum):
        pass

SIM_HEIGHT   = 3.0   # nominal sim height above ground in NED (pz = -SIM_HEIGHT)
HW_HEIGHT    = 0.85  # nominal hw height above ground in NED (pz = -HW_HEIGHT)


@dataclass(frozen=True)
class TrajContext:
    sim: bool
    hover_mode: Optional[int] = None
    spin: bool = False
    double_speed: bool = False
    short: bool = False


class TrajectoryType(StrEnum):
    HOVER          = "hover"
    SPIRAL         = "spiral"
    FIGURE_EIGHT   = "figure_eight"
    TREFOIL        = "trefoil"
    FIG8_HEADING   = "fig8_heading"
    F8_CONTRACTION = "f8_contraction"


# -- Trajectory functions ------------------------------------------------------

def hover(t: float, ctx: TrajContext) -> jnp.ndarray:
    height = SIM_HEIGHT if ctx.sim else HW_HEIGHT
    mode = ctx.hover_mode if ctx.hover_mode is not None else 1
    hover_dict = {
        1: jnp.array([0.0,  0.0, -height, 0.0]),
        2: jnp.array([0.0,  0.8, -height, 0.0]),
        3: jnp.array([0.8,  0.0, -height, 0.0]),
        4: jnp.array([0.8,  0.8, -height, 0.0]),
        # sim-only modes keep their original absolute heights
        5: jnp.array([0.0,  0.0, -10.0,   0.0]),
        6: jnp.array([1.0,  1.0,  -4.0,   0.0]),
        7: jnp.array([0.0, 10.0,  -5.0,   0.0]),
        8: jnp.array([1.0,  1.0,  -3.0,   0.0]),
    }
    if mode not in hover_dict:
        raise ValueError(f"Unknown hover mode: {mode}")
    if not ctx.sim and mode > 4:
        raise RuntimeError(f"Hover mode {mode} is sim-only")
    return hover_dict[mode]


def figure_eight(t: float, ctx: TrajContext) -> jnp.ndarray:
    """Larger figure-eight in sim, scaled version on hardware."""
    height = SIM_HEIGHT if ctx.sim else HW_HEIGHT
    R = 5.0 if ctx.sim else 0.4
    T = 10.0

    px  = R * jnp.sin(2 * jnp.pi * t / T)
    py  = R * jnp.sin(4 * jnp.pi * t / T) / 2.0
    pz  = -height
    psi = 0.0

    return jnp.array([px, py, pz, psi], dtype=jnp.float32)


def fig8_heading(t: float, ctx: TrajContext) -> jnp.ndarray:
    """Heading figure-eight in sim, scaled version on hardware."""
    height = SIM_HEIGHT if ctx.sim else HW_HEIGHT
    R = 3.0 if ctx.sim else 0.4
    T = 15.0
    s = 2 * jnp.pi * t / T

    px  = R * jnp.sin(s)
    py  = R * jnp.sin(2 * s) / 2.0
    pz  = -height
    psi = jnp.arctan2(jnp.cos(2 * s), jnp.cos(s))

    return jnp.array([px, py, pz, psi], dtype=jnp.float32)


def spiral(t: float, ctx: TrajContext) -> jnp.ndarray:
    """Retracing helix: spirals up, then exactly retraces on the way down.

    Sim bounds:
        height in [1.5, 3.0] m above ground
        radius = 5.0 m

    Hardware bounds:
        height in [0.5, 1.8] m above ground
        radius = 0.5 m
    """
    h_low = 1.5 if ctx.sim else 0.5
    h_high = 3.0 if ctx.sim else 1.8
    R = 5.0 if ctx.sim else 0.5

    cycle_time = 15.0
    num_turns = 2.0

    t_cycle = jnp.mod(t, cycle_time)
    T_half = cycle_time / 2.0
    up_mask = t_cycle <= T_half

    tau_up = t_cycle / T_half
    tau_down = (t_cycle - T_half) / T_half

    z_up = h_low + (h_high - h_low) * tau_up
    z_down = h_high - (h_high - h_low) * tau_down
    z_height = jnp.where(up_mask, z_up, z_down)

    theta_up = 2.0 * jnp.pi * num_turns * tau_up
    theta_down = 2.0 * jnp.pi * num_turns * (1.0 - tau_down)
    theta = jnp.where(up_mask, theta_up, theta_down)

    px = R * jnp.cos(theta)
    py = R * jnp.sin(theta)
    pz = -z_height
    psi = 0.0

    return jnp.array([px, py, pz, psi], dtype=jnp.float32)


def trefoil(t: float, ctx: TrajContext) -> jnp.ndarray:
    """3D trefoil with bounded altitude in sim and hardware."""
    T = 15.0
    R = 2.0 if ctx.sim else 0.3
    s = 2 * jnp.pi * t / T

    h_low = 1.5 if ctx.sim else 0.5
    h_high = 3.0 if ctx.sim else 1.8
    h_mid = 0.5 * (h_low + h_high)
    h_amp = 0.5 * (h_high - h_low)

    px = R * (jnp.sin(s) + 2 * jnp.sin(2 * s))
    py = R * (jnp.cos(s) - 2 * jnp.cos(2 * s))
    pz = -(h_mid + h_amp * jnp.sin(3 * s))
    psi = 0.0

    return jnp.array([px, py, pz, psi], dtype=jnp.float32)


# -- Registry ------------------------------------------------------------------

TRAJ_REGISTRY: Dict[TrajectoryType, object] = {
    TrajectoryType.HOVER:          hover,
    TrajectoryType.SPIRAL:         spiral,
    TrajectoryType.FIGURE_EIGHT:   figure_eight,
    TrajectoryType.TREFOIL:        trefoil,
    TrajectoryType.FIG8_HEADING:   fig8_heading,
    TrajectoryType.F8_CONTRACTION: figure_eight,
}
