"""Self-contained trajectory definitions for the contraction controller.

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

SIM_HEIGHT   = 3.0   # m above ground in NED (pz = -SIM_HEIGHT)
HW_HEIGHT    = 0.85  # m above ground in NED (pz = -HW_HEIGHT)


@dataclass(frozen=True)
class TrajContext:
    sim: bool
    hover_mode: Optional[int] = None


class TrajectoryType(StrEnum):
    HOVER        = "hover"
    SPIRAL       = "spiral"
    FIGURE_EIGHT = "figure_eight"
    TREFOIL      = "trefoil"
    FIG8_HEADING = "fig8_heading"


# ── Trajectory functions ──────────────────────────────────────────────────────

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


def spiral(t: float, ctx: TrajContext) -> jnp.ndarray:
    height = SIM_HEIGHT if ctx.sim else HW_HEIGHT
    R = 2.0 if ctx.sim else 0.5
    T = 7.5
    px  = R * jnp.sin(2 * jnp.pi * t / T)
    py  = R * jnp.cos(2 * jnp.pi * t / T)
    pz  = -height
    psi = 0.0
    return jnp.array([px, py, pz, psi], dtype=jnp.float32)


def figure_eight(t: float, ctx: TrajContext) -> jnp.ndarray:
    height = SIM_HEIGHT if ctx.sim else HW_HEIGHT
    R = 2.0 if ctx.sim else 0.5
    T = 10.0
    px  = R * jnp.sin(2 * jnp.pi * t / T)
    py  = R * jnp.sin(4 * jnp.pi * t / T) / 2
    pz  = -height
    psi = 0.0
    return jnp.array([px, py, pz, psi], dtype=jnp.float32)


def trefoil(t: float, ctx: TrajContext) -> jnp.ndarray:
    height = SIM_HEIGHT if ctx.sim else HW_HEIGHT
    R     = 1.0  if ctx.sim else 0.3
    z_amp = 0.4  if ctx.sim else 0.0   # vertical excursion; flat on hardware
    T = 15.0
    s   = 2 * jnp.pi * t / T
    px  = R * (jnp.sin(s) + 2 * jnp.sin(2 * s))
    py  = R * (jnp.cos(s) - 2 * jnp.cos(2 * s))
    pz  = z_amp * (-jnp.sin(3 * s)) - height
    psi = 0.0
    return jnp.array([px, py, pz, psi], dtype=jnp.float32)


def fig8_heading(t: float, ctx: TrajContext) -> jnp.ndarray:
    height = SIM_HEIGHT if ctx.sim else HW_HEIGHT
    R = 1.5 if ctx.sim else 0.4
    T = 15.0
    s   = 2 * jnp.pi * t / T
    px  = R * jnp.sin(s)
    py  = R * jnp.sin(2 * s) / 2
    pz  = -height
    psi = jnp.arctan2(jnp.cos(2 * s), jnp.cos(s))
    return jnp.array([px, py, pz, psi], dtype=jnp.float32)


# ── Registry ──────────────────────────────────────────────────────────────────

TRAJ_REGISTRY: Dict[TrajectoryType, object] = {
    TrajectoryType.HOVER:        hover,
    TrajectoryType.SPIRAL:       spiral,
    TrajectoryType.FIGURE_EIGHT: figure_eight,
    TrajectoryType.TREFOIL:      trefoil,
    TrajectoryType.FIG8_HEADING: fig8_heading,
}
