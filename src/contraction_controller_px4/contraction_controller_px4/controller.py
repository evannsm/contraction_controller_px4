"""Contraction controller core functions."""

import jax.numpy as jnp
from pathlib import Path


X_EQ = jnp.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0])

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
) -> jnp.ndarray:
    """Full contraction tracking control law.

    u = control(x, net) - control(x_ff, net) + u_ff

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

    Returns
    -------
    jnp.ndarray, shape (4,)
        Control input [T (N), p_cmd (rad/s), q_cmd (rad/s), r_cmd (rad/s)].
    """
    u = control(x, control_net) - control(x_ff, control_net) + u_ff
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
    import jax
    import immrax as irx  # type: ignore[import]
    jax.config.update("jax_enable_x64", False)  # model weights are float32
    return irx.NeuralNetwork(str(controller_dir))
