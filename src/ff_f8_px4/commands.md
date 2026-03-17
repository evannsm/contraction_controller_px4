# ff_f8_px4 Commands

Feedforward controller for `f8_contraction` that publishes `[throttle, p, q, r]`
derived from differential-flatness inversion. Pure open-loop is the default;
`--p-feedback` adds a small proportional correction using odometry, and
`--ramp-seconds` blends from hover commands into feedforward at startup.

The recommended operating mode is:

```bash
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --ramp-seconds 4.0 --log
```

## Build

```bash
cd ~/ws_px4_work
colcon build --packages-select quad_platforms quad_trajectories ff_f8_px4
source install/setup.bash
```

## Run Commands

### Simulation

```bash
# 1x, pure ff, default ramp
ros2 run ff_f8_px4 run_node --platform sim
ros2 run ff_f8_px4 run_node --platform sim --log
# -> sim_ff_f8_ramp2p0s_1x.csv

# 1x, pure ff, longer ramp
ros2 run ff_f8_px4 run_node --platform sim --ramp-seconds 4.0 --log
# -> sim_ff_f8_ramp4p0s_1x.csv

# 1x, pure ff, no ramp
ros2 run ff_f8_px4 run_node --platform sim --ramp-seconds 0 --log
# -> sim_ff_f8_1x.csv

# 1x, ff + p-feedback, default ramp
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --log
# -> sim_ff_f8_pfb_ramp2p0s_1x.csv

# 1x, ff + p-feedback, longer ramp
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --ramp-seconds 4.0 --log
# -> sim_ff_f8_pfb_ramp4p0s_1x.csv

# 1x, ff + p-feedback, no ramp
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --ramp-seconds 0 --log
# -> sim_ff_f8_pfb_1x.csv

# 2x, pure ff, default ramp
ros2 run ff_f8_px4 run_node --platform sim --double-speed --log
# -> sim_ff_f8_ramp2p0s_2x.csv

# 2x, pure ff, longer ramp
ros2 run ff_f8_px4 run_node --platform sim --double-speed --ramp-seconds 4.0 --log
# -> sim_ff_f8_ramp4p0s_2x.csv

# 2x, pure ff, no ramp
ros2 run ff_f8_px4 run_node --platform sim --double-speed --ramp-seconds 0 --log
# -> sim_ff_f8_2x.csv

# 2x, ff + p-feedback, default ramp
ros2 run ff_f8_px4 run_node --platform sim --double-speed --p-feedback --log
# -> sim_ff_f8_pfb_ramp2p0s_2x.csv

# 2x, ff + p-feedback, longer ramp
ros2 run ff_f8_px4 run_node --platform sim --double-speed --p-feedback --ramp-seconds 4.0 --log
# -> sim_ff_f8_pfb_ramp4p0s_2x.csv

# 2x, ff + p-feedback, no ramp
ros2 run ff_f8_px4 run_node --platform sim --double-speed --p-feedback --ramp-seconds 0 --log
# -> sim_ff_f8_pfb_2x.csv

# Custom log filename
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --log --log-file my_ff_run
```

### Hardware

```bash
ros2 run ff_f8_px4 run_node --platform hw --log
# -> hw_ff_f8_ramp2p0s_1x.csv

ros2 run ff_f8_px4 run_node --platform hw --ramp-seconds 4.0 --log
# -> hw_ff_f8_ramp4p0s_1x.csv

ros2 run ff_f8_px4 run_node --platform hw --ramp-seconds 0 --log
# -> hw_ff_f8_1x.csv

ros2 run ff_f8_px4 run_node --platform hw --double-speed --log
# -> hw_ff_f8_ramp2p0s_2x.csv

ros2 run ff_f8_px4 run_node --platform hw --double-speed --ramp-seconds 4.0 --log
# -> hw_ff_f8_ramp4p0s_2x.csv

ros2 run ff_f8_px4 run_node --platform hw --double-speed --ramp-seconds 0 --log
# -> hw_ff_f8_2x.csv

ros2 run ff_f8_px4 run_node --platform hw --p-feedback --log
# -> hw_ff_f8_pfb_ramp2p0s_1x.csv

ros2 run ff_f8_px4 run_node --platform hw --p-feedback --ramp-seconds 4.0 --log
# -> hw_ff_f8_pfb_ramp4p0s_1x.csv

ros2 run ff_f8_px4 run_node --platform hw --p-feedback --ramp-seconds 0 --log
# -> hw_ff_f8_pfb_1x.csv

ros2 run ff_f8_px4 run_node --platform hw --double-speed --p-feedback --log
# -> hw_ff_f8_pfb_ramp2p0s_2x.csv

ros2 run ff_f8_px4 run_node --platform hw --double-speed --p-feedback --ramp-seconds 4.0 --log
# -> hw_ff_f8_pfb_ramp4p0s_2x.csv

ros2 run ff_f8_px4 run_node --platform hw --double-speed --p-feedback --ramp-seconds 0 --log
# -> hw_ff_f8_pfb_2x.csv
```

## Arguments Reference

| Argument | Required | Values | Description |
|----------|----------|--------|-------------|
| `--platform` | Yes | `sim`, `hw` | Platform type |
| `--double-speed` | No | flag | Mark log with `_2x` (trajectory period is fixed at 10s) |
| `--p-feedback` | No | flag | Add light proportional position / attitude feedback on top of feedforward |
| `--ramp-seconds` | No | float | Blend from hover commands into feedforward over this many seconds (`0` disables) |
| `--log` | No | flag | Enable data logging |
| `--log-file` | No | string | Custom log filename (requires `--log`) |
| `--flight-period` | No | float | Override default flight duration (sim: 30s, hw: 60s) |

## Log Filename Format

```text
{platform}_ff_f8[_pfb][_rampXs]_{speed}.csv
```

Examples:

- `sim_ff_f8_ramp2p0s_1x.csv`
- `sim_ff_f8_pfb_ramp2p0s_1x.csv`
- `sim_ff_f8_pfb_ramp4p0s_1x.csv`
- `hw_ff_f8_pfb_ramp2p0s_2x.csv`

## Comparison with NR

| Run | Command | Log file |
|-----|---------|----------|
| NR feedback only | `ros2 run newton_raphson_px4 run_node --platform sim --trajectory f8_contraction --log` | `sim_nr_std_f8_contraction_1x.csv` |
| NR + feedforward | `ros2 run newton_raphson_px4 run_node --platform sim --trajectory f8_contraction --ff --log` | `sim_nr_std_f8_contraction_ff_1x.csv` |
| Pure feedforward | `ros2 run ff_f8_px4 run_node --platform sim --log` | `sim_ff_f8_ramp2p0s_1x.csv` |
| Feedforward + P feedback | `ros2 run ff_f8_px4 run_node --platform sim --p-feedback --log` | `sim_ff_f8_pfb_ramp2p0s_1x.csv` |

## Control Summary

- `flat_to_x_u(...)` generates nominal position, velocity, thrust, and Euler-angle rates from the figure-8 flat output.
- those Euler-angle rates are converted to body rates `[p, q, r]` before sending them to PX4.
- `x_ff[6]` is multiplied by quadrotor mass to obtain force before converting to throttle.
- `--p-feedback` adds light error correction and damping around the nominal feedforward.
- `--ramp-seconds` smooths the hover-to-trajectory transition in both trajectory timing and commanded inputs.
