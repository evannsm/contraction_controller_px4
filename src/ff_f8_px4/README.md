# ff_f8_px4

ROS 2 package for the `f8_contraction` trajectory that publishes body-rate and thrust
commands derived from differential-flatness feedforward. The package now supports
three layers that can be combined:

- flatness feedforward for the nominal `f8_contraction` motion
- an optional startup ramp so the controller does not jump instantly from hover to the moving trajectory
- an optional light feedback layer (`--p-feedback`) that adds position, velocity, attitude, and body-rate damping

## Modes

- Default: pure open-loop feedforward
- `--p-feedback`: feedforward plus light proportional position / attitude correction
- `--ramp-seconds`: startup blend from hover commands into feedforward (default `2.0`)

## How It Works

At each control tick the node evaluates the `f8_contraction` flat output and runs
`flat_to_x_u(...)` from `quad_trajectories`. That returns:

- `x_ff = [px, py, pz, vx, vy, vz, f_specific, phi, th, psi]`
- `u_ff = [df, dphi, dth, dpsi]`

The controller then:

1. Converts `u_ff[1:4]` from Euler-angle rates into body rates `[p, q, r]`.
2. Converts `x_ff[6]` from specific thrust to force with `force = mass * f_specific`.
3. Converts force into PX4 throttle with the platform-specific thrust curve.
4. Publishes `VehicleRatesSetpoint` with `[throttle, p, q, r]`.

When `--p-feedback` is enabled, the node adds small corrections on top of the flatness command:

- XY position and velocity error adjust desired roll/pitch
- Z position and velocity error adjust collective thrust
- yaw error adjusts yaw rate
- measured body rates are damped directly before publishing

When `--ramp-seconds > 0`, the controller also does two startup-smoothing steps:

- it time-warps the trajectory at the start so reference speed rises gradually
- it blends commanded inputs from hover `[hover thrust, 0, 0, 0]` into the feedforward command

This is important because the vehicle begins from hover, while the figure-8 reference
implies nonzero velocity and nonzero angular-rate demand almost immediately.

## Build

```bash
cd ~/ws_px4_work
colcon build --packages-select quad_platforms quad_trajectories ff_f8_px4
source install/setup.bash
```

## Example Commands

```bash
# Pure feedforward, default ramp
ros2 run ff_f8_px4 run_node --platform sim --log

# Pure feedforward, no ramp
ros2 run ff_f8_px4 run_node --platform sim --ramp-seconds 0 --log

# Feedforward with light proportional feedback, default ramp
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --log

# Feedforward with light proportional feedback, longer ramp
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --ramp-seconds 4.0 --log

# Feedforward with light proportional feedback, no ramp
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --ramp-seconds 0 --log

# Double speed variants
ros2 run ff_f8_px4 run_node --platform sim --double-speed --log
ros2 run ff_f8_px4 run_node --platform sim --double-speed --p-feedback --log

# Hardware
ros2 run ff_f8_px4 run_node --platform hw --p-feedback --log
```

## Command Matrix

### Simulation

```bash
# 1x, pure ff
ros2 run ff_f8_px4 run_node --platform sim --log
ros2 run ff_f8_px4 run_node --platform sim --ramp-seconds 4.0 --log
ros2 run ff_f8_px4 run_node --platform sim --ramp-seconds 0 --log

# 1x, ff + p-feedback
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --log
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --ramp-seconds 4.0 --log
ros2 run ff_f8_px4 run_node --platform sim --p-feedback --ramp-seconds 0 --log

# 2x, pure ff
ros2 run ff_f8_px4 run_node --platform sim --double-speed --log
ros2 run ff_f8_px4 run_node --platform sim --double-speed --ramp-seconds 4.0 --log
ros2 run ff_f8_px4 run_node --platform sim --double-speed --ramp-seconds 0 --log

# 2x, ff + p-feedback
ros2 run ff_f8_px4 run_node --platform sim --double-speed --p-feedback --log
ros2 run ff_f8_px4 run_node --platform sim --double-speed --p-feedback --ramp-seconds 4.0 --log
ros2 run ff_f8_px4 run_node --platform sim --double-speed --p-feedback --ramp-seconds 0 --log
```

### Hardware

```bash
# 1x, pure ff
ros2 run ff_f8_px4 run_node --platform hw --log
ros2 run ff_f8_px4 run_node --platform hw --ramp-seconds 4.0 --log
ros2 run ff_f8_px4 run_node --platform hw --ramp-seconds 0 --log

# 1x, ff + p-feedback
ros2 run ff_f8_px4 run_node --platform hw --p-feedback --log
ros2 run ff_f8_px4 run_node --platform hw --p-feedback --ramp-seconds 4.0 --log
ros2 run ff_f8_px4 run_node --platform hw --p-feedback --ramp-seconds 0 --log

# 2x, pure ff
ros2 run ff_f8_px4 run_node --platform hw --double-speed --log
ros2 run ff_f8_px4 run_node --platform hw --double-speed --ramp-seconds 4.0 --log
ros2 run ff_f8_px4 run_node --platform hw --double-speed --ramp-seconds 0 --log

# 2x, ff + p-feedback
ros2 run ff_f8_px4 run_node --platform hw --double-speed --p-feedback --log
ros2 run ff_f8_px4 run_node --platform hw --double-speed --p-feedback --ramp-seconds 4.0 --log
ros2 run ff_f8_px4 run_node --platform hw --double-speed --p-feedback --ramp-seconds 0 --log
```

## Arguments

| Argument | Description |
|----------|-------------|
| `--platform {sim,hw}` | Platform type |
| `--double-speed` | Mark log filename with `_2x` (trajectory period remains fixed at 10s) |
| `--p-feedback` | Add light proportional position / attitude feedback |
| `--ramp-seconds` | Startup blend duration in seconds; `0` disables the ramp |
| `--log` | Enable logging |
| `--log-file NAME` | Custom log filename |
| `--flight-period SECONDS` | Override default flight duration |

## Practical Guidance

- `pure ff` is mainly useful as a baseline comparison and is sensitive to mismatch.
- `--p-feedback` is the recommended mode if you want the controller to actually track the trajectory reasonably.
- increasing `--ramp-seconds` reduces the startup discontinuity when switching from hover to figure-8 flight.
- the hover / return position is the actual first `f8_contraction` reference point, not a generic hardcoded hover point.
- logs are saved under `src/data_analysis/log_files/ff_f8_px4/`.

## Log Filenames

```text
{platform}_ff_f8[_pfb][_rampXs]_{speed}.csv
```

Examples:

- `sim_ff_f8_ramp2p0s_1x.csv`
- `sim_ff_f8_pfb_ramp2p0s_1x.csv`
- `hw_ff_f8_pfb_ramp2p0s_2x.csv`

## Technical Note

A more detailed derivation is available in:

- `docs/ff_f8_controller.qmd`
- rendered PDF: `docs/ff_f8_controller.pdf`
