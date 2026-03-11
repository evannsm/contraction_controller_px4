"""Contraction controller core functions."""

import jax
import jax.numpy as jnp
import numpy as np
import control as ctrl
from pathlib import Path

GRAVITY = 9.806

X_EQ = jnp.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0], dtype=jnp.float32)

K_EQ = jnp.array([
    [ 1.3687341e-15,  9.2672128e-16,  1.0000000e+00,  8.3074916e-16,
      7.3950095e-16,  2.4142137e+00, -2.4142137e+00,  1.1556979e-15,
     -1.7440397e-15, -3.3548259e-16],
    [ 7.6503672e-16, -1.0000000e+00, -6.5669665e-16,  1.6588481e-16,
     -1.4515237e+00, -1.6074038e-15,  1.1556979e-15, -5.4294472e+00,
     -2.2995322e-15, -1.2982682e-17],
    [ 1.0000000e+00, -6.8869814e-16,  2.2466232e-15,  1.4515237e+00,
     -8.2817958e-16,  3.4353443e-15, -1.7440397e-15, -2.2995322e-15,
     -5.4294472e+00, -2.6573605e-16],
    [-6.2797133e-18, -1.8035081e-16,  5.7418685e-16,  4.5449832e-17,
     -1.0375176e-16,  1.1316697e-15, -3.3548259e-16, -1.2982682e-17,
     -2.6573605e-16, -1.0000000e+00],
], dtype=jnp.float32)


# ── NED quadrotor dynamics for linearisation ──────────────────────────────────
# State:  [px, py, pz, vx, vy, vz, f, phi, theta, psi]
#   f     = thrust acceleration (T/m), phi = roll, theta = pitch, psi = yaw
# Input:  [df, dphi, dtheta, dpsi]  (all are direct rate commands)

def quad_dynamics_ned(x: jnp.ndarray, u: jnp.ndarray) -> jnp.ndarray:
    """NED quadrotor dynamics xdot = f(x, u) for jacfwd linearisation."""
    _, _, _, vx, vy, vz, f, phi, theta, psi = x
    df, dphi, dtheta, dpsi = u

    sp, cp = jnp.sin(phi),   jnp.cos(phi)
    st, ct = jnp.sin(theta),  jnp.cos(theta)
    sy, cy = jnp.sin(psi),    jnp.cos(psi)

    dvx = -f * (cy * st * cp + sy * sp)
    dvy = -f * (sy * st * cp - cy * sp)
    dvz =  GRAVITY - f * ct * cp

    return jnp.array([vx, vy, vz, dvx, dvy, dvz, df, dphi, dtheta, dpsi],
                     dtype=jnp.float32)


# ── Time-varying LQR ──────────────────────────────────────────────────────────

_N_STATE = 10
_N_INPUT = 4

# B is constant: only [6:10, 0:4] = I_4 (rate inputs map directly to state rates)
_B_np = np.zeros((_N_STATE, _N_INPUT), dtype=np.float64)
_B_np[6:, :] = np.eye(_N_INPUT, dtype=np.float64)

# State:   [px,  py,  pz,   vx,  vy,  vz,   f,   phi, theta, psi]
# Control: [df,  dphi, dtheta, dpsi]
# Conservative tuning: modest z/yaw emphasis, R stays near identity.
Q_LQR = np.diag([2.0, 2.0, 5.0, 1.0, 1.0, 2.0, 1.0, 2.0, 2.0, 8.0])
R_LQR = np.diag([1.0, 1.0, 1.0, 0.5])

# JIT-compiled Jacobian — Jacobian of NED dynamics w.r.t. state and input
_jac_dynamics = jax.jit(jax.jacfwd(quad_dynamics_ned, argnums=(0, 1)))


def compute_lqr_gain(
    x0: jnp.ndarray,
    u0: jnp.ndarray,
) -> jnp.ndarray:
    """Compute LQR gain linearised around operating point (x0, u0).

    x0 should be the actual vehicle state [px,py,pz, vx,vy,vz, f, phi, theta, psi].
    Linearises the NED dynamics via JIT-compiled jacfwd, then solves the
    continuous-time algebraic Riccati equation (CARE) via control.lqr in float64.

    Sign convention: returns K such that the stabilising correction term is
    +K @ (x - x_ff), matching the K_EQ sign convention (K_EQ = -K_standard).
    """
    A, _ = _jac_dynamics(x0, u0)
    A_np = np.array(A, dtype=np.float64)
    K_standard, _, _ = ctrl.lqr(A_np, _B_np, Q_LQR, R_LQR)
    # Negate: control.lqr returns K for u = -K*(x-x_ref), but the
    # contraction law uses +K*(x-x_ref), so K_here = -K_standard.
    return jnp.array(-K_standard, dtype=jnp.float32)


def control(x: jnp.ndarray, control_net) -> jnp.ndarray:
    """Contraction control policy evaluated at state x.

    Parameters
    ----------
    x : jnp.ndarray, shape (10,)
        Current state [px, py, pz, vx, vy, vz, az, roll, pitch, yaw].
    control_net : immrax.NeuralNetwork
        Trained neural network controller loaded from the Controller directory.

    Returns
    -------
    jnp.ndarray, shape (4,)
        Raw control [T (N), p_cmd (rad/s), q_cmd (rad/s), r_cmd (rad/s)].
    """
    return control_net(x) + K_EQ @ (x - X_EQ)


def contraction_control(
    x: jnp.ndarray,
    x_ff: jnp.ndarray,
    u_ff: jnp.ndarray,
    control_net,
    K: jnp.ndarray | None = None,
) -> jnp.ndarray:
    """Full contraction tracking control law.

    u = net(x) - net(x_ff) + K @ (x - x_ff) + u_ff

    Parameters
    ----------
    x : jnp.ndarray, shape (10,)
        Current state.
    x_ff : jnp.ndarray, shape (10,)
        Feedforward (reference) state at current time.
    u_ff : jnp.ndarray, shape (4,)
        Feedforward control that keeps the system on the reference trajectory.
    control_net : immrax.NeuralNetwork
        Trained neural network controller.
    K : jnp.ndarray, shape (4, 10), optional
        LQR gain to use.  Defaults to K_EQ if None.

    Returns
    -------
    jnp.ndarray, shape (4,)
        Control input [T (N), p_cmd (rad/s), q_cmd (rad/s), r_cmd (rad/s)].
    """
    K_use = K_EQ if K is None else K

    # Normalize actual yaw to be within π of reference yaw so that:
    #   1. The yaw error fed to K is in [-π, π] (no 2π jumps)
    #   2. The NN sees a yaw consistent with x_ff (not ±2π away from it)
    yaw_diff = x[9] - x_ff[9]
    n_wraps = jnp.round(yaw_diff / (2 * jnp.pi))
    x_aligned = x.at[9].set(x[9] - n_wraps * 2 * jnp.pi)

    e = x_aligned - x_ff
    u = control_net(x_aligned) - control_net(x_ff) + K_use @ e + u_ff
    return u


def load_control_net(controller_dir: str | Path):
    """Load the NeuralNetwork from the given directory.

    Parameters
    ----------
    controller_dir : str or Path
        Path to the Controller directory containing arch.txt and model.eqx.

    Returns
    -------
    immrax.NeuralNetwork
    """
    import immrax as irx  # type: ignore[import]
    return irx.NeuralNetwork(str(controller_dir))
