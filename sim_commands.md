# Simulation Commands

## Docker / Workspace

```bash
make build
make run
make build_ros
make build_ros PACKAGES="pkg1 pkg2"
make clean_build_ros
make generate_nmpc_solver
make attach
make stop
make kill
```

## Shared Run Suffixes

All run commands below are unlogged as written. Add these suffixes when needed:

```bash
LOG=1
LOG=1 LOG_FILE=my_run
FLIGHT_PERIOD=45
```

Python NR / NMPC also support:

```bash
PYJOULES=1
LOG=1 PYJOULES=1
```

Contraction-only extras:

```bash
CONTROLLER_DIR=src/controller_params/new_params
NO_FEEDFORWARD=1
```

`ff_f8`-only extras:

```bash
P_FEEDBACK=1
RAMP_SECONDS=0
RAMP_SECONDS=4.0
```

## Actual Modifier Support

- `HOVER_MODE` is required for `hover` and `hover_contraction`; simulation allows `1..8`.
- `DOUBLE_SPEED=1` is meaningful for `yaw_only`, `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix`, `sawtooth`, and `triangle`.
- `SPIN=1` is meaningful for `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix`, `sawtooth`, and `triangle`.
- `SHORT=1` is only meaningful for `fig8_vert`.
- `FF=1` is only valid for `fig8_contraction`.
- `fig8_contraction` ignores `DOUBLE_SPEED=1`, so no 2x command is listed for it.
- `make run_ff_f8` accepts `P_FEEDBACK=1` and `RAMP_SECONDS=<seconds>` in addition to `DOUBLE_SPEED=1`.

## Contraction Controller (Python)

Use `make run_contraction ...`. Add `LOG=1`, `LOG_FILE=...`, `FLIGHT_PERIOD=...`, `CONTROLLER_DIR=...`, or `NO_FEEDFORWARD=1` as needed.

```bash
# Hover contraction
make run_contraction PLATFORM=sim TRAJECTORY=hover_contraction HOVER_MODE=1
make run_contraction PLATFORM=sim TRAJECTORY=hover_contraction HOVER_MODE=2
make run_contraction PLATFORM=sim TRAJECTORY=hover_contraction HOVER_MODE=3
make run_contraction PLATFORM=sim TRAJECTORY=hover_contraction HOVER_MODE=4
make run_contraction PLATFORM=sim TRAJECTORY=hover_contraction HOVER_MODE=5
make run_contraction PLATFORM=sim TRAJECTORY=hover_contraction HOVER_MODE=6
make run_contraction PLATFORM=sim TRAJECTORY=hover_contraction HOVER_MODE=7
make run_contraction PLATFORM=sim TRAJECTORY=hover_contraction HOVER_MODE=8

# Figure-8 contraction
make run_contraction PLATFORM=sim TRAJECTORY=fig8_contraction
make run_contraction PLATFORM=sim TRAJECTORY=fig8_contraction NO_FEEDFORWARD=1

# Figure-8 heading contraction
make run_contraction PLATFORM=sim TRAJECTORY=fig8_heading_contraction
make run_contraction PLATFORM=sim TRAJECTORY=fig8_heading_contraction NO_FEEDFORWARD=1

# Spiral contraction
make run_contraction PLATFORM=sim TRAJECTORY=spiral_contraction
make run_contraction PLATFORM=sim TRAJECTORY=spiral_contraction NO_FEEDFORWARD=1

# Trefoil contraction
make run_contraction PLATFORM=sim TRAJECTORY=trefoil_contraction
make run_contraction PLATFORM=sim TRAJECTORY=trefoil_contraction NO_FEEDFORWARD=1
```

## Newton-Raphson (Python)

Use `make run_newton_raphson ...`. Add `LOG=1`, `LOG_FILE=...`, `FLIGHT_PERIOD=...`, and optionally `PYJOULES=1`.

```bash
# Hover
make run_newton_raphson PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=hover HOVER_MODE=2
make run_newton_raphson PLATFORM=sim TRAJECTORY=hover HOVER_MODE=3
make run_newton_raphson PLATFORM=sim TRAJECTORY=hover HOVER_MODE=4
make run_newton_raphson PLATFORM=sim TRAJECTORY=hover HOVER_MODE=5
make run_newton_raphson PLATFORM=sim TRAJECTORY=hover HOVER_MODE=6
make run_newton_raphson PLATFORM=sim TRAJECTORY=hover HOVER_MODE=7
make run_newton_raphson PLATFORM=sim TRAJECTORY=hover HOVER_MODE=8

# Hover contraction
make run_newton_raphson PLATFORM=sim TRAJECTORY=hover_contraction

# Yaw only
make run_newton_raphson PLATFORM=sim TRAJECTORY=yaw_only
make run_newton_raphson PLATFORM=sim TRAJECTORY=yaw_only DOUBLE_SPEED=1

# Circle horizontal
make run_newton_raphson PLATFORM=sim TRAJECTORY=circle_horz
make run_newton_raphson PLATFORM=sim TRAJECTORY=circle_horz SPIN=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=circle_horz DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=circle_horz SPIN=1 DOUBLE_SPEED=1

# Circle vertical
make run_newton_raphson PLATFORM=sim TRAJECTORY=circle_vert
make run_newton_raphson PLATFORM=sim TRAJECTORY=circle_vert SPIN=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=circle_vert DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=circle_vert SPIN=1 DOUBLE_SPEED=1

# Figure-8 horizontal
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_horz
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_horz SPIN=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_horz DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_horz SPIN=1 DOUBLE_SPEED=1

# Figure-8 vertical
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_vert
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_vert SHORT=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_vert DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 SHORT=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_vert SHORT=1 DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 SHORT=1 DOUBLE_SPEED=1

# Helix
make run_newton_raphson PLATFORM=sim TRAJECTORY=helix
make run_newton_raphson PLATFORM=sim TRAJECTORY=helix SPIN=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=helix DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=helix SPIN=1 DOUBLE_SPEED=1

# Sawtooth
make run_newton_raphson PLATFORM=sim TRAJECTORY=sawtooth
make run_newton_raphson PLATFORM=sim TRAJECTORY=sawtooth SPIN=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=sawtooth DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=sawtooth SPIN=1 DOUBLE_SPEED=1

# Triangle
make run_newton_raphson PLATFORM=sim TRAJECTORY=triangle
make run_newton_raphson PLATFORM=sim TRAJECTORY=triangle SPIN=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=triangle DOUBLE_SPEED=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=triangle SPIN=1 DOUBLE_SPEED=1

# Figure-8 contraction
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_contraction
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_contraction FF=1

# Figure-8 heading contraction
make run_newton_raphson PLATFORM=sim TRAJECTORY=fig8_heading_contraction

# Spiral contraction
make run_newton_raphson PLATFORM=sim TRAJECTORY=spiral_contraction

# Trefoil contraction
make run_newton_raphson PLATFORM=sim TRAJECTORY=trefoil_contraction
```

## Newton-Raphson C++

Use `make run_newton_raphson_cpp ...`. Add `LOG=1`, `LOG_FILE=...`, or `FLIGHT_PERIOD=...` as needed.

```bash
# Hover
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=2
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=3
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=4
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=5
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=6
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=7
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=8

# Hover contraction
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=hover_contraction

# Yaw only
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=yaw_only
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=yaw_only DOUBLE_SPEED=1

# Circle horizontal
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=circle_horz
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=circle_horz SPIN=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=circle_horz DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=circle_horz SPIN=1 DOUBLE_SPEED=1

# Circle vertical
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=circle_vert
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=circle_vert SPIN=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=circle_vert DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=circle_vert SPIN=1 DOUBLE_SPEED=1

# Figure-8 horizontal
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_horz
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_horz SPIN=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_horz DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_horz SPIN=1 DOUBLE_SPEED=1

# Figure-8 vertical
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_vert
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_vert SHORT=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_vert DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 SHORT=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_vert SHORT=1 DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 SHORT=1 DOUBLE_SPEED=1

# Helix
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=helix
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=helix SPIN=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=helix DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=helix SPIN=1 DOUBLE_SPEED=1

# Sawtooth
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=sawtooth
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=sawtooth SPIN=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=sawtooth DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=sawtooth SPIN=1 DOUBLE_SPEED=1

# Triangle
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=triangle
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=triangle SPIN=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=triangle DOUBLE_SPEED=1
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=triangle SPIN=1 DOUBLE_SPEED=1

# Figure-8 contraction
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_contraction
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_contraction FF=1

# Figure-8 heading contraction
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=fig8_heading_contraction

# Spiral contraction
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=spiral_contraction

# Trefoil contraction
make run_newton_raphson_cpp PLATFORM=sim TRAJECTORY=trefoil_contraction
```

## NMPC Acados (Python)

Use `make run_nmpc ...`. Add `LOG=1`, `LOG_FILE=...`, `FLIGHT_PERIOD=...`, and optionally `PYJOULES=1`.

```bash
# Hover
make run_nmpc PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1
make run_nmpc PLATFORM=sim TRAJECTORY=hover HOVER_MODE=2
make run_nmpc PLATFORM=sim TRAJECTORY=hover HOVER_MODE=3
make run_nmpc PLATFORM=sim TRAJECTORY=hover HOVER_MODE=4
make run_nmpc PLATFORM=sim TRAJECTORY=hover HOVER_MODE=5
make run_nmpc PLATFORM=sim TRAJECTORY=hover HOVER_MODE=6
make run_nmpc PLATFORM=sim TRAJECTORY=hover HOVER_MODE=7
make run_nmpc PLATFORM=sim TRAJECTORY=hover HOVER_MODE=8

# Hover contraction
make run_nmpc PLATFORM=sim TRAJECTORY=hover_contraction

# Yaw only
make run_nmpc PLATFORM=sim TRAJECTORY=yaw_only
make run_nmpc PLATFORM=sim TRAJECTORY=yaw_only DOUBLE_SPEED=1

# Circle horizontal
make run_nmpc PLATFORM=sim TRAJECTORY=circle_horz
make run_nmpc PLATFORM=sim TRAJECTORY=circle_horz SPIN=1
make run_nmpc PLATFORM=sim TRAJECTORY=circle_horz DOUBLE_SPEED=1
make run_nmpc PLATFORM=sim TRAJECTORY=circle_horz SPIN=1 DOUBLE_SPEED=1

# Circle vertical
make run_nmpc PLATFORM=sim TRAJECTORY=circle_vert
make run_nmpc PLATFORM=sim TRAJECTORY=circle_vert SPIN=1
make run_nmpc PLATFORM=sim TRAJECTORY=circle_vert DOUBLE_SPEED=1
make run_nmpc PLATFORM=sim TRAJECTORY=circle_vert SPIN=1 DOUBLE_SPEED=1

# Figure-8 horizontal
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_horz
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_horz SPIN=1
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_horz DOUBLE_SPEED=1
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_horz SPIN=1 DOUBLE_SPEED=1

# Figure-8 vertical
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_vert
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_vert SHORT=1
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_vert DOUBLE_SPEED=1
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 SHORT=1
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 DOUBLE_SPEED=1
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_vert SHORT=1 DOUBLE_SPEED=1
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 SHORT=1 DOUBLE_SPEED=1

# Helix
make run_nmpc PLATFORM=sim TRAJECTORY=helix
make run_nmpc PLATFORM=sim TRAJECTORY=helix SPIN=1
make run_nmpc PLATFORM=sim TRAJECTORY=helix DOUBLE_SPEED=1
make run_nmpc PLATFORM=sim TRAJECTORY=helix SPIN=1 DOUBLE_SPEED=1

# Sawtooth
make run_nmpc PLATFORM=sim TRAJECTORY=sawtooth
make run_nmpc PLATFORM=sim TRAJECTORY=sawtooth SPIN=1
make run_nmpc PLATFORM=sim TRAJECTORY=sawtooth DOUBLE_SPEED=1
make run_nmpc PLATFORM=sim TRAJECTORY=sawtooth SPIN=1 DOUBLE_SPEED=1

# Triangle
make run_nmpc PLATFORM=sim TRAJECTORY=triangle
make run_nmpc PLATFORM=sim TRAJECTORY=triangle SPIN=1
make run_nmpc PLATFORM=sim TRAJECTORY=triangle DOUBLE_SPEED=1
make run_nmpc PLATFORM=sim TRAJECTORY=triangle SPIN=1 DOUBLE_SPEED=1

# Figure-8 contraction
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_contraction
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_contraction FF=1

# Figure-8 heading contraction
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_heading_contraction

# Spiral contraction
make run_nmpc PLATFORM=sim TRAJECTORY=spiral_contraction

# Trefoil contraction
make run_nmpc PLATFORM=sim TRAJECTORY=trefoil_contraction
```

## NMPC Acados C++

Run `make generate_nmpc_solver` before the first C++ NMPC build, then use `make run_nmpc_cpp ...`. Add `LOG=1`, `LOG_FILE=...`, or `FLIGHT_PERIOD=...` as needed.

```bash
# Hover
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=2
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=3
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=4
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=5
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=6
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=7
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=hover HOVER_MODE=8

# Hover contraction
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=hover_contraction

# Yaw only
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=yaw_only
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=yaw_only DOUBLE_SPEED=1

# Circle horizontal
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=circle_horz
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=circle_horz SPIN=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=circle_horz DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=circle_horz SPIN=1 DOUBLE_SPEED=1

# Circle vertical
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=circle_vert
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=circle_vert SPIN=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=circle_vert DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=circle_vert SPIN=1 DOUBLE_SPEED=1

# Figure-8 horizontal
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_horz
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_horz SPIN=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_horz DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_horz SPIN=1 DOUBLE_SPEED=1

# Figure-8 vertical
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_vert
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_vert SHORT=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_vert DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 SHORT=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_vert SHORT=1 DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_vert SPIN=1 SHORT=1 DOUBLE_SPEED=1

# Helix
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=helix
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=helix SPIN=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=helix DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=helix SPIN=1 DOUBLE_SPEED=1

# Sawtooth
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=sawtooth
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=sawtooth SPIN=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=sawtooth DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=sawtooth SPIN=1 DOUBLE_SPEED=1

# Triangle
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=triangle
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=triangle SPIN=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=triangle DOUBLE_SPEED=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=triangle SPIN=1 DOUBLE_SPEED=1

# Figure-8 contraction
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_contraction
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_contraction FF=1

# Figure-8 heading contraction
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=fig8_heading_contraction

# Spiral contraction
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=spiral_contraction

# Trefoil contraction
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=trefoil_contraction
```

## Feedforward Figure-8 (Python)

Use `make run_ff_f8 ...`. Add `LOG=1`, `LOG_FILE=...`, or `FLIGHT_PERIOD=...` as needed.

```bash
# 1x, default 2.0 s ramp
make run_ff_f8 PLATFORM=sim
make run_ff_f8 PLATFORM=sim P_FEEDBACK=1
make run_ff_f8 PLATFORM=sim RAMP_SECONDS=0
make run_ff_f8 PLATFORM=sim RAMP_SECONDS=4.0
make run_ff_f8 PLATFORM=sim P_FEEDBACK=1 RAMP_SECONDS=0
make run_ff_f8 PLATFORM=sim P_FEEDBACK=1 RAMP_SECONDS=4.0

# 2x
make run_ff_f8 PLATFORM=sim DOUBLE_SPEED=1
make run_ff_f8 PLATFORM=sim DOUBLE_SPEED=1 P_FEEDBACK=1
make run_ff_f8 PLATFORM=sim DOUBLE_SPEED=1 RAMP_SECONDS=0
make run_ff_f8 PLATFORM=sim DOUBLE_SPEED=1 RAMP_SECONDS=4.0
make run_ff_f8 PLATFORM=sim DOUBLE_SPEED=1 P_FEEDBACK=1 RAMP_SECONDS=0
make run_ff_f8 PLATFORM=sim DOUBLE_SPEED=1 P_FEEDBACK=1 RAMP_SECONDS=4.0
```
