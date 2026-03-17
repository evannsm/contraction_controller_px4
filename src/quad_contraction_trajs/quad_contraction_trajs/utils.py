"""Differential-flatness utilities for quadrotor trajectory tracking.

Provides flat_to_x_u (single-point), generate_reference_trajectory (position +
velocity over a horizon), and generate_feedforward_trajectory (full state +
control over a horizon via flat_to_x_u).
"""

import jax
import jax.numpy as jnp

GRAVITY: float = 9.81


def flat_to_x_u(t, flat_output):
    """Compute state x_ff and feedforward control u_ff from flat outputs via autodiff.

    The flat output is [px, py, pz, psi]. Derivatives are computed with jax.jacfwd.
    State:   x = [px, py, pz, vx, vy, vz, f, phi, th, psi]
               where f = specific thrust (N/kg), phi = roll, th = pitch
    Control: u = [df, dphi, dth, dpsi]  (rates of f, roll, pitch, yaw)
    """
    g = GRAVITY
    px, py, pz, psi = flat_output(t)
    vx, vy, vz, dpsi = jax.jacfwd(flat_output)(t)

    def f_th_phi(t):
        ax, ay, az = jax.jacfwd(jax.jacfwd(flat_output))(t)[:3]
        f = jnp.sqrt(ax**2 + ay**2 + (az - g) ** 2)
        th = jnp.arcsin(-ax / f)
        phi = jnp.arctan2(ay, g - az)
        return jnp.array([f, th, phi])

    f, th, phi = f_th_phi(t)
    df, dth, dphi = jax.jacfwd(f_th_phi)(t)
    x_ff = jnp.array([px, py, pz, vx, vy, vz, f, phi, th, psi], dtype=jnp.float32)
    u_ff = jnp.array([df, dphi, dth, dpsi], dtype=jnp.float32)
    return x_ff, u_ff


def flat_to_x(t, flat_output):
    """Compute reference state x_ff only -- skips u_ff (3rd-order) derivatives."""
    g = GRAVITY
    px, py, pz, psi = flat_output(t)
    vx, vy, vz, _ = jax.jacfwd(flat_output)(t)
    ax, ay, az = jax.jacfwd(jax.jacfwd(flat_output))(t)[:3]
    f = jnp.sqrt(ax**2 + ay**2 + (az - g) ** 2)
    th = jnp.arcsin(-ax / f)
    phi = jnp.arctan2(ay, g - az)
    return jnp.array([px, py, pz, vx, vy, vz, f, phi, th, psi], dtype=jnp.float32)


def generate_reference_trajectory(traj_func, t_start, horizon, num_steps, ctx):
    """Evaluate a trajectory and its first derivative over a time horizon.

    Parameters
    ----------
    traj_func : callable(t, ctx) -> jnp.ndarray[4]
    t_start   : float  -- start time
    horizon   : float  -- time horizon (0.0 for single-point evaluation)
    num_steps : int    -- number of evaluation points
    ctx       : TrajContext

    Returns
    -------
    r    : jnp.ndarray (num_steps, 4)  -- [px, py, pz, psi]
    rdot : jnp.ndarray (num_steps, 4)  -- [vx, vy, vz, dpsi]
    """
    if num_steps == 1:
        times = jnp.array([t_start])
    else:
        times = jnp.linspace(t_start, t_start + horizon, num_steps)

    def _eval(t):
        r = traj_func(t, ctx)
        rdot = jax.jacfwd(traj_func)(t, ctx)
        return r, rdot

    r, rdot = jax.vmap(_eval)(times)
    return r, rdot


def generate_feedforward_trajectory(traj_fn, ctx, t_start, horizon, num_steps):
    """Evaluate flat_to_x_u over a prediction horizon (for NMPC feedforward).

    Parameters
    ----------
    traj_fn   : callable(t, ctx) -> jnp.ndarray[4]
    ctx       : TrajContext
    t_start   : float  -- start time
    horizon   : float  -- prediction horizon
    num_steps : int    -- number of evaluation points

    Returns
    -------
    x_ff : jnp.ndarray (num_steps, 10) -- [px,py,pz, vx,vy,vz, f, phi, th, psi]
    u_ff : jnp.ndarray (num_steps, 4)  -- [df, dphi, dth, dpsi]
    """
    times = jnp.linspace(t_start, t_start + horizon, num_steps)

    def _eval(t):
        flat_output = lambda s: traj_fn(s, ctx)
        return flat_to_x_u(t, flat_output)

    x_ff, u_ff = jax.vmap(_eval)(times)
    return x_ff, u_ff
