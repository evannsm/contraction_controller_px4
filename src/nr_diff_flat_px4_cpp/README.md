# nr_diff_flat_px4_cpp

C++ implementation of the Newton-Raphson Standard offboard controller for PX4-based quadrotors.

## Overview

This package provides a real-time Newton-Raphson controller that runs as a ROS 2 node. It tracks 3D trajectories (position + yaw) by iteratively solving for the body-rate/thrust input via Jacobian inversion of the forward prediction.

**Key features:**
- C++ implementation for low-latency control (~100 Hz)
- Supports all trajectory types from `quad_trajectories_cpp`
- Differential-flatness feedforward for `fig8_contraction` (via `--ff` flag)
- Data logging via `ros2_logger_cpp`
- Battery voltage compensation on thrust normalization

## Build

```bash
cd ~/ws_px4_work
colcon build --packages-select quad_platforms_cpp quad_trajectories_cpp nr_diff_flat_px4_cpp
source install/setup.bash
```

## Run

### Basic usage

```bash
# Simulation — figure-8 contraction at 2x speed
ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory fig8_contraction --double-speed

# Simulation — figure-8 contraction with feedforward at 2x speed
ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory fig8_contraction --double-speed --ff

# Simulation — helix with spin and logging
ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory helix --double-speed --spin --log

# Hardware — hover at position 1
ros2 run nr_diff_flat_px4_cpp run_node --platform hw --trajectory hover --hover-mode 1
```

### With logging

```bash
# Auto-generated filename: sim_nr_std_fig8_contraction_1x.csv
ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory fig8_contraction --log

# Auto-generated filename: sim_nr_std_fig8_contraction_ff_1x.csv
ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory fig8_contraction --ff --log

# Auto-generated filename: sim_nr_std_fig8_contraction_ff_2x.csv
ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory fig8_contraction --ff --double-speed --log

# Custom filename
ros2 run nr_diff_flat_px4_cpp run_node --platform sim --trajectory helix --log --log-file my_experiment
```

## CLI Options

| Argument | Required | Values | Description |
|----------|----------|--------|-------------|
| `--platform` | Yes | `sim`, `hw` | Platform type |
| `--trajectory` | Yes | `hover`, `yaw_only`, `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix`, `sawtooth`, `triangle`, `fig8_contraction` | Trajectory type |
| `--hover-mode` | If hover | `1-8` (sim), `1-4` (hw) | Hover position |
| `--double-speed` | No | flag | 2x trajectory speed |
| `--short` | No | flag | Short fig8_vert variant |
| `--spin` | No | flag | Enable yaw rotation for circle_horz / helix |
| `--ff` | No | flag | Enable feedforward injection + add `_ff` to log filename (only valid with `fig8_contraction`) |
| `--log` | No | flag | Enable data logging (auto-generates filename) |
| `--log-file` | No | string | Custom log filename (requires `--log`) |
| `--flight-period` | No | float | Override default flight duration (sim: 30s, hw: 60s) |

## Feedforward (`--ff`)

When `--ff` is passed with `--trajectory=fig8_contraction`, differential-flatness inversion is applied at every control step:

1. Analytically computes `x_ff = [px, py, pz, vx, vy, vz, f_specific, phi, th, psi]` and `u_ff = [df, dphi, dth, dpsi]` from the fig8_contraction trajectory at the lookahead time.
2. Converts Euler-rate feedforward `[dphi, dth, dpsi]` to body-rate space via `T⁻¹ @ euler_rates` (where T is the ZYX kinematic matrix evaluated at the current roll/pitch).
3. Sets the operating point for the NR Jacobian to `[mass × f_specific, p_ff, q_ff, r_ff]` before solving.

This resets the NR linearization point to the feedforward trajectory each step — no accumulation, and the Jacobian is evaluated at the correct operating point.

## Log Filename Format

```
{platform}_nr_std_{trajectory}[_ff]_{speed}[_short][_spin].csv
```

Examples:
- `sim_nr_std_helix_2x_spin.csv`
- `sim_nr_std_fig8_contraction_1x.csv`
- `sim_nr_std_fig8_contraction_ff_2x.csv`
- `hw_nr_std_fig8_vert_2x_short.csv`
