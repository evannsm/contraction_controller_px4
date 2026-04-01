# Hardware Commands

## Docker / Workspace

```bash
make build
make run
make run ROS_DOMAIN_ID=42
make build_ros
make build_ros PACKAGES="pkg1 pkg2"
make build_ros UP_TO=nmpc_acados_px4_cpp
make clean_build_ros
make generate_nmpc_solver
make generate_nmpc_solver PLATFORM=hw FORCE=1
make attach
make stop
make kill
```

## Shared Run Suffixes

All run commands below are unlogged as written. Add these suffixes when needed:

```bash
LOG=1
LOG=1 LOG_FILE=my_run
FLIGHT_PERIOD=60
```

Python NR / diff-flat / NMPC also support:

```bash
PYJOULES=1
LOG=1 PYJOULES=1
```

Contraction-only extras:

```bash
CONTROLLER_DIR=src/contraction_controller_px4/contraction_controller_px4/params/new_params
NO_FEEDFORWARD=1
```

`ff_f8`-only extras:

```bash
P_FEEDBACK=1
RAMP_SECONDS=0
RAMP_SECONDS=4.0
```

## Actual Modifier Support

- `HOVER_MODE` is required for `hover` and `hover_contraction`; hardware allows `1..4`.
- `DOUBLE_SPEED=1` is meaningful for `yaw_only`, `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix`, `sawtooth`, and `triangle`.
- `SPIN=1` is meaningful for `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix`, `sawtooth`, and `triangle`.
- `SHORT=1` is only meaningful for `fig8_vert`.
- `FF=1` is only valid for `fig8_contraction`.
- `CTRL_TYPE=jax|numpy` is only meaningful for `make run_nr_diff_flat`.
- `fig8_contraction` ignores `DOUBLE_SPEED=1`, so no 2x command is listed for it.
- `make run_ff_f8` accepts `P_FEEDBACK=1` and `RAMP_SECONDS=<seconds>` in addition to `DOUBLE_SPEED=1`.

## Contraction Controller (Python)

Use `make run_contraction ...`. Add `LOG=1`, `LOG_FILE=...`, `FLIGHT_PERIOD=...`, `CONTROLLER_DIR=...`, or `NO_FEEDFORWARD=1` as needed.

```bash
# Hover contraction
make run_contraction PLATFORM=hw TRAJECTORY=hover_contraction HOVER_MODE=1
make run_contraction PLATFORM=hw TRAJECTORY=hover_contraction HOVER_MODE=2
make run_contraction PLATFORM=hw TRAJECTORY=hover_contraction HOVER_MODE=3
make run_contraction PLATFORM=hw TRAJECTORY=hover_contraction HOVER_MODE=4

# Figure-8 contraction
make run_contraction PLATFORM=hw TRAJECTORY=fig8_contraction
make run_contraction PLATFORM=hw TRAJECTORY=fig8_contraction NO_FEEDFORWARD=1

# Figure-8 heading contraction
make run_contraction PLATFORM=hw TRAJECTORY=fig8_heading_contraction
make run_contraction PLATFORM=hw TRAJECTORY=fig8_heading_contraction NO_FEEDFORWARD=1

# Spiral contraction
make run_contraction PLATFORM=hw TRAJECTORY=spiral_contraction
make run_contraction PLATFORM=hw TRAJECTORY=spiral_contraction NO_FEEDFORWARD=1

# Trefoil contraction
make run_contraction PLATFORM=hw TRAJECTORY=trefoil_contraction
make run_contraction PLATFORM=hw TRAJECTORY=trefoil_contraction NO_FEEDFORWARD=1
```

## Newton-Raphson (Python)

Use `make run_newton_raphson ...`. Add `LOG=1`, `LOG_FILE=...`, `FLIGHT_PERIOD=...`, and optionally `PYJOULES=1`.

```bash
# Hover
make run_newton_raphson PLATFORM=hw TRAJECTORY=hover HOVER_MODE=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=hover HOVER_MODE=2
make run_newton_raphson PLATFORM=hw TRAJECTORY=hover HOVER_MODE=3
make run_newton_raphson PLATFORM=hw TRAJECTORY=hover HOVER_MODE=4

# Hover contraction
make run_newton_raphson PLATFORM=hw TRAJECTORY=hover_contraction

# Yaw only
make run_newton_raphson PLATFORM=hw TRAJECTORY=yaw_only
make run_newton_raphson PLATFORM=hw TRAJECTORY=yaw_only DOUBLE_SPEED=1

# Circle horizontal
make run_newton_raphson PLATFORM=hw TRAJECTORY=circle_horz
make run_newton_raphson PLATFORM=hw TRAJECTORY=circle_horz SPIN=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=circle_horz DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=circle_horz SPIN=1 DOUBLE_SPEED=1

# Circle vertical
make run_newton_raphson PLATFORM=hw TRAJECTORY=circle_vert
make run_newton_raphson PLATFORM=hw TRAJECTORY=circle_vert SPIN=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=circle_vert DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=circle_vert SPIN=1 DOUBLE_SPEED=1

# Figure-8 horizontal
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_horz
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_horz DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1 DOUBLE_SPEED=1

# Figure-8 vertical
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_vert
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_vert DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1 DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1 DOUBLE_SPEED=1

# Helix
make run_newton_raphson PLATFORM=hw TRAJECTORY=helix
make run_newton_raphson PLATFORM=hw TRAJECTORY=helix SPIN=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=helix DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=helix SPIN=1 DOUBLE_SPEED=1

# Sawtooth
make run_newton_raphson PLATFORM=hw TRAJECTORY=sawtooth
make run_newton_raphson PLATFORM=hw TRAJECTORY=sawtooth SPIN=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=sawtooth DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=sawtooth SPIN=1 DOUBLE_SPEED=1

# Triangle
make run_newton_raphson PLATFORM=hw TRAJECTORY=triangle
make run_newton_raphson PLATFORM=hw TRAJECTORY=triangle SPIN=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=triangle DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=hw TRAJECTORY=triangle SPIN=1 DOUBLE_SPEED=1

# Figure-8 contraction
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_contraction
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_contraction FF=1

# Figure-8 heading contraction
make run_newton_raphson PLATFORM=hw TRAJECTORY=fig8_heading_contraction

# Spiral contraction
make run_newton_raphson PLATFORM=hw TRAJECTORY=spiral_contraction

# Trefoil contraction
make run_newton_raphson PLATFORM=hw TRAJECTORY=trefoil_contraction
```

## Newton-Raphson Diff-Flat (Python)

Use `make run_nr_diff_flat ...`. Add `LOG=1`, `LOG_FILE=...`, `FLIGHT_PERIOD=...`, and optionally `PYJOULES=1` or `CTRL_TYPE=numpy`.

```bash
# Hover
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=hover HOVER_MODE=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=hover HOVER_MODE=2
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=hover HOVER_MODE=3
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=hover HOVER_MODE=4

# Hover contraction
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=hover_contraction

# Yaw only
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=yaw_only
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=yaw_only DOUBLE_SPEED=1

# Circle horizontal
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=circle_horz
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=circle_horz SPIN=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=circle_horz DOUBLE_SPEED=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=circle_horz SPIN=1 DOUBLE_SPEED=1

# Circle vertical
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=circle_vert
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=circle_vert SPIN=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=circle_vert DOUBLE_SPEED=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=circle_vert SPIN=1 DOUBLE_SPEED=1

# Figure-8 horizontal
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_horz
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_horz DOUBLE_SPEED=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1 DOUBLE_SPEED=1

# Figure-8 vertical
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_vert
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_vert DOUBLE_SPEED=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 DOUBLE_SPEED=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1 DOUBLE_SPEED=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1 DOUBLE_SPEED=1

# Helix
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=helix
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=helix SPIN=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=helix DOUBLE_SPEED=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=helix SPIN=1 DOUBLE_SPEED=1

# Sawtooth
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=sawtooth
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=sawtooth SPIN=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=sawtooth DOUBLE_SPEED=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=sawtooth SPIN=1 DOUBLE_SPEED=1

# Triangle
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=triangle
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=triangle SPIN=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=triangle DOUBLE_SPEED=1
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=triangle SPIN=1 DOUBLE_SPEED=1

# Figure-8 contraction
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_contraction

# Figure-8 heading contraction
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=fig8_heading_contraction

# Spiral contraction
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=spiral_contraction

# Trefoil contraction
make run_nr_diff_flat PLATFORM=hw TRAJECTORY=trefoil_contraction
```

## Newton-Raphson C++

Use `make run_newton_raphson_cpp ...`. Add `LOG=1`, `LOG_FILE=...`, or `FLIGHT_PERIOD=...` as needed.

```bash
# Hover
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=hover HOVER_MODE=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=hover HOVER_MODE=2
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=hover HOVER_MODE=3
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=hover HOVER_MODE=4

# Hover contraction
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=hover_contraction

# Yaw only
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=yaw_only
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=yaw_only DOUBLE_SPEED=1

# Circle horizontal
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=circle_horz
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=circle_horz SPIN=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=circle_horz DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=circle_horz SPIN=1 DOUBLE_SPEED=1

# Circle vertical
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=circle_vert
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=circle_vert SPIN=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=circle_vert DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=circle_vert SPIN=1 DOUBLE_SPEED=1

# Figure-8 horizontal
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_horz
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_horz DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1 DOUBLE_SPEED=1

# Figure-8 vertical
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_vert
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_vert DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1 DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1 DOUBLE_SPEED=1

# Helix
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=helix
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=helix SPIN=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=helix DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=helix SPIN=1 DOUBLE_SPEED=1

# Sawtooth
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=sawtooth
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=sawtooth SPIN=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=sawtooth DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=sawtooth SPIN=1 DOUBLE_SPEED=1

# Triangle
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=triangle
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=triangle SPIN=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=triangle DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=triangle SPIN=1 DOUBLE_SPEED=1

# Figure-8 contraction
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_contraction
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_contraction FF=1

# Figure-8 heading contraction
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=fig8_heading_contraction

# Spiral contraction
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=spiral_contraction

# Trefoil contraction
make run_newton_raphson_cpp PLATFORM=hw TRAJECTORY=trefoil_contraction
```

## NMPC Acados (Python)

Use `make run_nmpc ...`. Add `LOG=1`, `LOG_FILE=...`, `FLIGHT_PERIOD=...`, and optionally `PYJOULES=1`.

```bash
# Hover
make run_nmpc PLATFORM=hw TRAJECTORY=hover HOVER_MODE=1
make run_nmpc PLATFORM=hw TRAJECTORY=hover HOVER_MODE=2
make run_nmpc PLATFORM=hw TRAJECTORY=hover HOVER_MODE=3
make run_nmpc PLATFORM=hw TRAJECTORY=hover HOVER_MODE=4

# Hover contraction
make run_nmpc PLATFORM=hw TRAJECTORY=hover_contraction

# Yaw only
make run_nmpc PLATFORM=hw TRAJECTORY=yaw_only
make run_nmpc PLATFORM=hw TRAJECTORY=yaw_only DOUBLE_SPEED=1

# Circle horizontal
make run_nmpc PLATFORM=hw TRAJECTORY=circle_horz
make run_nmpc PLATFORM=hw TRAJECTORY=circle_horz SPIN=1
make run_nmpc PLATFORM=hw TRAJECTORY=circle_horz DOUBLE_SPEED=1
make run_nmpc PLATFORM=hw TRAJECTORY=circle_horz SPIN=1 DOUBLE_SPEED=1

# Circle vertical
make run_nmpc PLATFORM=hw TRAJECTORY=circle_vert
make run_nmpc PLATFORM=hw TRAJECTORY=circle_vert SPIN=1
make run_nmpc PLATFORM=hw TRAJECTORY=circle_vert DOUBLE_SPEED=1
make run_nmpc PLATFORM=hw TRAJECTORY=circle_vert SPIN=1 DOUBLE_SPEED=1

# Figure-8 horizontal
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_horz
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_horz DOUBLE_SPEED=1
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1 DOUBLE_SPEED=1

# Figure-8 vertical
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_vert
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_vert DOUBLE_SPEED=1
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 DOUBLE_SPEED=1
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1 DOUBLE_SPEED=1
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1 DOUBLE_SPEED=1

# Helix
make run_nmpc PLATFORM=hw TRAJECTORY=helix
make run_nmpc PLATFORM=hw TRAJECTORY=helix SPIN=1
make run_nmpc PLATFORM=hw TRAJECTORY=helix DOUBLE_SPEED=1
make run_nmpc PLATFORM=hw TRAJECTORY=helix SPIN=1 DOUBLE_SPEED=1

# Sawtooth
make run_nmpc PLATFORM=hw TRAJECTORY=sawtooth
make run_nmpc PLATFORM=hw TRAJECTORY=sawtooth SPIN=1
make run_nmpc PLATFORM=hw TRAJECTORY=sawtooth DOUBLE_SPEED=1
make run_nmpc PLATFORM=hw TRAJECTORY=sawtooth SPIN=1 DOUBLE_SPEED=1

# Triangle
make run_nmpc PLATFORM=hw TRAJECTORY=triangle
make run_nmpc PLATFORM=hw TRAJECTORY=triangle SPIN=1
make run_nmpc PLATFORM=hw TRAJECTORY=triangle DOUBLE_SPEED=1
make run_nmpc PLATFORM=hw TRAJECTORY=triangle SPIN=1 DOUBLE_SPEED=1

# Figure-8 contraction
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_contraction
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_contraction FF=1

# Figure-8 heading contraction
make run_nmpc PLATFORM=hw TRAJECTORY=fig8_heading_contraction

# Spiral contraction
make run_nmpc PLATFORM=hw TRAJECTORY=spiral_contraction

# Trefoil contraction
make run_nmpc PLATFORM=hw TRAJECTORY=trefoil_contraction
```

## NMPC Acados C++

`make run_nmpc_cpp ...` auto-checks the solver stamp, regenerates when the platform/formulation changed, rebuilds the C++ package, and then runs. `make generate_nmpc_solver PLATFORM=sim|hw` is still available as a manual convenience target.

```bash
# Hover
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=hover HOVER_MODE=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=hover HOVER_MODE=2
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=hover HOVER_MODE=3
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=hover HOVER_MODE=4

# Hover contraction
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=hover_contraction

# Yaw only
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=yaw_only
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=yaw_only DOUBLE_SPEED=1

# Circle horizontal
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=circle_horz
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=circle_horz SPIN=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=circle_horz DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=circle_horz SPIN=1 DOUBLE_SPEED=1

# Circle vertical
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=circle_vert
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=circle_vert SPIN=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=circle_vert DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=circle_vert SPIN=1 DOUBLE_SPEED=1

# Figure-8 horizontal
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_horz
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_horz DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_horz SPIN=1 DOUBLE_SPEED=1

# Figure-8 vertical
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_vert
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_vert DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_vert SHORT=1 DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_vert SPIN=1 SHORT=1 DOUBLE_SPEED=1

# Helix
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=helix
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=helix SPIN=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=helix DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=helix SPIN=1 DOUBLE_SPEED=1

# Sawtooth
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=sawtooth
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=sawtooth SPIN=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=sawtooth DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=sawtooth SPIN=1 DOUBLE_SPEED=1

# Triangle
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=triangle
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=triangle SPIN=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=triangle DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=triangle SPIN=1 DOUBLE_SPEED=1

# Figure-8 contraction
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_contraction
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_contraction FF=1

# Figure-8 heading contraction
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=fig8_heading_contraction

# Spiral contraction
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=spiral_contraction

# Trefoil contraction
make run_nmpc_cpp PLATFORM=hw TRAJECTORY=trefoil_contraction
```

## Feedforward Figure-8 (Python)

Use `make run_ff_f8 ...`. Add `LOG=1`, `LOG_FILE=...`, or `FLIGHT_PERIOD=...` as needed.

```bash
# 1x, default 2.0 s ramp
make run_ff_f8 PLATFORM=hw
make run_ff_f8 PLATFORM=hw P_FEEDBACK=1
make run_ff_f8 PLATFORM=hw RAMP_SECONDS=0
make run_ff_f8 PLATFORM=hw RAMP_SECONDS=4.0
make run_ff_f8 PLATFORM=hw P_FEEDBACK=1 RAMP_SECONDS=0
make run_ff_f8 PLATFORM=hw P_FEEDBACK=1 RAMP_SECONDS=4.0

# 2x
make run_ff_f8 PLATFORM=hw DOUBLE_SPEED=1
make run_ff_f8 PLATFORM=hw DOUBLE_SPEED=1 P_FEEDBACK=1
make run_ff_f8 PLATFORM=hw DOUBLE_SPEED=1 RAMP_SECONDS=0
make run_ff_f8 PLATFORM=hw DOUBLE_SPEED=1 RAMP_SECONDS=4.0
make run_ff_f8 PLATFORM=hw DOUBLE_SPEED=1 P_FEEDBACK=1 RAMP_SECONDS=0
make run_ff_f8 PLATFORM=hw DOUBLE_SPEED=1 P_FEEDBACK=1 RAMP_SECONDS=4.0
```
