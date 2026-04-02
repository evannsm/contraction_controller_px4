# PX4 Quadrotor Controllers

A ROS 2 workspace containing multiple offboard controllers for PX4-based quadrotors,
all sharing a common trajectory library and platform abstraction layer.

## Controllers

| Target | Package | Language | Key dependencies |
|---|---|---|---|
| `run_contraction` | `contraction_controller_px4` | Python | JAX, equinox, immrax, linrax |
| `run_newton_raphson` | `newton_raphson_px4` | Python | JAX |
| `run_nr_diff_flat` | `nr_diff_flat_px4` | Python | JAX or NumPy, differential flatness / feedback linearization |
| `run_newton_raphson_cpp` | `newton_raphson_px4_cpp` | C++ | Eigen, autodiff |
| `run_nmpc` | `nmpc_acados_px4` | Python | acados, casadi |
| `run_nmpc_cpp` | `nmpc_acados_px4_cpp` | C++ | acados C API |
| `run_ff_f8` | `ff_f8_px4` | Python | JAX |

---

## Prerequisites (host machine)

### 1. Docker Engine

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER   # log out and back in
```

### 2. PX4-Autopilot (simulation only)

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh
make px4_sitl gz_x500           # confirm it runs
```

### 3. MicroXRCE-DDS-Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install && sudo ldconfig
```

### 4. Controller weights (contraction controller only)

Place the trained network files at:

```
src/contraction_controller_px4/contraction_controller_px4/params/
├── arch.txt       # network architecture, e.g.  10 128 softplus 128 softplus 4
├── model.eqx      # equinox-serialised weights
└── ix_ut.npy      # equilibrium input
```

---

## Quickstart

```bash
git clone --recurse-submodules git@github.com:evannsm/contraction_controller_px4.git
cd contraction_controller_px4
make build      # build Docker image (~15 min, once)
make run        # start container
make build_ros  # build ROS 2 workspace inside container
```

### NMPC C++ additional step

The root make targets now auto-handle the NMPC solver stamp and the C++ rebuild
path. If you want to pre-generate the solver explicitly for a given platform, you
can still do that:

```bash
make generate_nmpc_solver PLATFORM=sim
make generate_nmpc_solver PLATFORM=hw
```

---

## Usage walkthrough

### Fully automated experiment run

For simulation experiments, `make fly` and `make fly_all` start host SITL,
start the Micro XRCE bridge, ensure the Docker container is running, build the
selected controller package, run the flight, and generate RMSE tables/plots in
`src/data_analysis/results/...`.

```bash
make fly CONTROLLER=nmpc_cpp TRAJECTORY=fig8_contraction HEADLESS=1
make fly_all TRAJECTORY=fig8_contraction HEADLESS=1
```

`HEADLESS=1` uses PX4 headless SITL. Omitting it starts Gazebo normally.
`PX4_MODEL=...`, `NMPC_HORIZON=...`, and `NMPC_NUM_STEPS=...` are also forwarded
through the automated run path when you need a different SITL model or NMPC
discretisation.

### Manual workflow

### Step 1 — Start the simulator and bridge (host terminals)

Use these only when you are not using `make fly` / `make fly_all`.

**Terminal 1** — PX4 SITL + Gazebo:
```bash
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

**Terminal 2** — Micro XRCE-DDS bridge:
```bash
MicroXRCEAgent udp4 -p 8888
```

### Step 2 — Start the container

```bash
make run
```

Mounts the repo root → `/workspace` so `build/`, `install/`, and `log/` persist
on the host. Uses `--net host` so the container sees all ROS 2 topics.

### Step 3 — Build the ROS 2 workspace (first time only)

```bash
make build_ros
```

### Step 4 — Run a controller

```bash
make run_contraction PLATFORM=sim TRAJECTORY=hover_contraction HOVER_MODE=1
make run_newton_raphson PLATFORM=sim TRAJECTORY=helix DOUBLE_SPEED=1 SPIN=1 LOG=1
make run_nr_diff_flat PLATFORM=sim TRAJECTORY=helix CTRL_TYPE=jax LOG=1
make run_nmpc PLATFORM=sim TRAJECTORY=fig8_contraction FF=1 LOG=1
make run_nmpc_cpp PLATFORM=sim TRAJECTORY=circle_horz LOG=1
make run_ff_f8 PLATFORM=sim LOG=1
```

Or drop into the container and run directly:

```bash
make attach
ros2 run newton_raphson_px4 run_node --platform sim --trajectory helix --double-speed --spin --log
```

---

## All make commands

### Image and container

| Command | What it does |
|---|---|
| `make build` | Build the Docker image (run once, or after `Dockerfile` changes) |
| `make run` | Start the container (mounts repo root → `/workspace`) |
| `make stop` | Gracefully stop the container |
| `make kill` | Force-kill the container |
| `make attach` | Open an interactive shell inside the running container |
| `make start_sitl` | Start PX4 SITL with Gazebo on the host |
| `make start_sitl_headless` | Start PX4 headless SITL on the host |
| `make start_bridge` | Start MicroXRCEAgent on the host |
| `make fly ...` | Run one fully automated simulation experiment |
| `make fly_all ...` | Run a multi-controller simulation sweep |

### Building the ROS 2 workspace

```bash
make build_ros                        # build all packages
make build_ros UP_TO=nmpc_acados_px4_cpp
make build_ros PACKAGES="pkg1 pkg2"   # build specific packages only
make clean_build_ros                  # wipe build/install/log and rebuild from scratch
```

With `--symlink-install`, edits to existing Python source files are picked up
immediately without rebuilding.

### NMPC solver generation

```bash
make generate_nmpc_solver PLATFORM=sim
make generate_nmpc_solver PLATFORM=hw
make generate_nmpc_solver PLATFORM=hw FORCE=1
```

`make run_nmpc` and `make run_nmpc_cpp` also call the solver guard
automatically, so switching `PLATFORM=sim` to `PLATFORM=hw` regenerates the
solver when the mass or formulation changed.

### Controller variables

| Variable | Default | Description |
|---|---|---|
| `PLATFORM` | `sim` | `sim` or `hw` |
| `TRAJECTORY` | `hover` | see trajectory table below |
| `HOVER_MODE` | `1` | 1–8 for sim / 1–4 for hw; required for `TRAJECTORY=hover`, and for `run_contraction` when `TRAJECTORY=hover_contraction` |
| `FLIGHT_PERIOD` | *(auto)* | flight duration in seconds (30 sim / 60 hw) |
| `PX4_MODEL` | `gz_x500` | PX4 SITL model for `start_sitl`, `start_sitl_headless`, `fly`, and `fly_all` |
| `NMPC_HORIZON` | `2.0` | NMPC horizon in seconds for solver generation / guard |
| `NMPC_NUM_STEPS` | `50` | NMPC discretisation steps for solver generation / guard |
| `DOUBLE_SPEED` | *(off)* | set to any non-empty value for 2x speed on trajectories that implement it |
| `SHORT` | *(off)* | short variant for fig8_vert |
| `SPIN` | *(off)* | enable yaw spin on trajectories that implement it (`circle_*`, `fig8_*`, `helix`, `sawtooth`, `triangle`) |
| `FF` | *(off)* | enable differential-flatness feedforward (fig8_contraction only) |
| `LOG` | *(off)* | enable CSV logging |
| `LOG_FILE` | *(auto)* | log filename stem (without `.csv`) |
| `PYJOULES` | *(off)* | enable PyJoules energy logging (Python NR / diff-flat / NMPC only) |
| `CTRL_TYPE` | *(package default)* | `jax` or `numpy` for `run_nr_diff_flat`, and forwarded through `fly` / `fly_all` when `CONTROLLER=nr_diff_flat` |
| `CONTROLLER_DIR` | *(auto)* | neural network weights dir (contraction only) |
| `NO_FEEDFORWARD` | `0` | set to `1` to disable u_ff (contraction only) |
| `P_FEEDBACK` | *(off)* | enable proportional feedback layer in `run_ff_f8` |
| `RAMP_SECONDS` | *(default node value)* | startup ramp override for `run_ff_f8` |
| `HEADLESS` | *(off)* | use headless PX4 SITL in `fly` / `fly_all` |
| `RESULTS_DIR` | `src/data_analysis/results/<auto>` | output directory for `fly` / `fly_all` (must stay under the workspace root so the container can write analysis artifacts) |
| `RUN_NAME` | *(auto)* | custom experiment name for `fly` / `fly_all` |
| `CONTROLLER` | `newton_raphson` | controller key for `fly` (`contraction`, `newton_raphson`, `nr_diff_flat`, `newton_raphson_cpp`, `nmpc`, `nmpc_cpp`, `ff_f8`) |
| `CONTROLLERS` | *(auto)* | comma-separated controller list override for `fly_all` using the same keys as `CONTROLLER` |

### Infrastructure and recovery variables

| Variable | Default | Description |
|---|---|---|
| `PX4_DIR` | `~/PX4-Autopilot` | host PX4 checkout used by `start_sitl`, `start_sitl_headless`, `fly`, and `fly_all` |
| `MICROXRCE_PORT` | `8888` | UDP port for `MicroXRCEAgent` |
| `ROS_DOMAIN_ID` | `31` | ROS 2 domain passed to the container and host-side automation |
| `IMAGE_NAME` | `px4_controllers_jazzy` | Docker image tag used by `build`, `run`, `fly`, and `fly_all` |
| `CONTAINER_NAME` | `px4_controllers` | Docker container name used by `run`, `attach`, controller targets, and automation |
| `PACKAGES` | *(empty)* | package list for `make build_ros PACKAGES="..."` |
| `UP_TO` | *(empty)* | package frontier for `make build_ros UP_TO=...` |
| `FORCE` | *(empty)* | set to a non-empty value to force `make generate_nmpc_solver` regeneration |

### Rare overrides and recovery commands

```bash
# Use a non-default PX4 checkout or SITL model
make start_sitl PX4_DIR=~/PX4-Autopilot PX4_MODEL=gz_x500

# Run the bridge on a different port
make start_bridge MICROXRCE_PORT=9999

# Force NMPC regeneration even if the solver stamp matches
make generate_nmpc_solver PLATFORM=hw FORCE=1

# Run automation in a different ROS domain
make fly CONTROLLER=nmpc_cpp TRAJECTORY=fig8_contraction ROS_DOMAIN_ID=42

# Run the differential-flatness controller with the NumPy variant
make fly CONTROLLER=nr_diff_flat TRAJECTORY=helix CTRL_TYPE=numpy HEADLESS=1

# Name a run explicitly or choose the output directory
make fly CONTROLLER=nmpc TRAJECTORY=helix RUN_NAME=helix_trial_01
make fly_all TRAJECTORY=fig8_contraction RESULTS_DIR=src/data_analysis/results/fig8_batch

# Override the default fly_all controller set
make fly_all TRAJECTORY=fig8_contraction CONTROLLERS=nr_diff_flat,nmpc,nmpc_cpp,ff_f8
```

---

## Available trajectories

| Name | Shape | Period |
|---|---|---|
| `hover` | Static point (8 modes) | — |
| `hover_contraction` | Contraction hover variant | — |
| `yaw_only` | Yaw rotation in place | 10 s |
| `circle_horz` | Horizontal circle | 10 s |
| `circle_vert` | Vertical circle | 10 s |
| `fig8_horz` | Horizontal figure-eight | 10 s |
| `fig8_vert` | Vertical figure-eight | 10 s |
| `helix` | Helical path | 10 s |
| `sawtooth` | Sawtooth wave | 10 s |
| `triangle` | Triangle wave | 10 s |
| `fig8_contraction` | Contraction figure-eight | 10 s |
| `fig8_heading_contraction` | Figure-eight with heading tracking | 15 s |
| `spiral_contraction` | Horizontal spiral | 7.5 s |
| `trefoil_contraction` | 3-D trefoil knot | 15 s |

Not all controllers support all trajectories. The contraction controller uses
`_contraction`-suffixed trajectories; other controllers accept the full set.

### Hover modes

| `HOVER_MODE` | Position (x, y) | Notes |
|---|---|---|
| 1 | (0.0, 0.0) | centre — default |
| 2 | (0.0, 0.8) | |
| 3 | (0.8, 0.0) | |
| 4 | (0.8, 0.8) | |
| 5 | (0.0, 0.0) | −10 m altitude, **sim only** |
| 6 | (1.0, 1.0) | −4 m altitude, **sim only** |
| 7 | (0.0, 10.0) | −5 m altitude, **sim only** |
| 8 | (1.0, 1.0) | −3 m altitude, **sim only** |

---

## Flight phases

Every run follows four automatic phases:

```
HOVER (10 s) → CUSTOM (flight-period) → RETURN (10 s) → LAND
```

| Phase | Mode | Notes |
|---|---|---|
| **HOVER** | PX4 position setpoint | JIT compilation runs during this window |
| **CUSTOM** | Offboard bodyrate control | Controller tracks the selected trajectory |
| **RETURN** | PX4 position setpoint | Vehicle returns to hover point |
| **LAND** | PX4 land command | Descends and disarms automatically |

---

## Data logging

Pass `LOG=1` to enable CSV logging. The file is written to
`src/data_analysis/log_files/<package_name>/` (on the host, since the workspace
is bind-mounted) when the node shuts down (Ctrl+C or end of flight).

Logging runs at 10 Hz (during CUSTOM phase only).

`make fly` and `make fly_all` always log and then run
`src/data_analysis/run_analysis.py`, which writes RMSE tables, per-log plots,
and comparison figures under `src/data_analysis/results/...`.

---

## Repository structure

```
contraction_controller_px4/               ← repo root = ROS 2 workspace root
├── src/
│   ├── contraction_controller_px4/       ← Contraction NN controller (Python)
│   ├── newton_raphson_px4/               ← Newton-Raphson controller (Python, submodule)
│   ├── newton_raphson_px4_cpp/           ← Newton-Raphson controller (C++, submodule)
│   ├── nmpc_acados_px4/                  ← NMPC Acados controller (Python, submodule)
│   ├── nmpc_acados_px4_cpp/              ← NMPC Acados controller (C++, submodule)
│   ├── ff_f8_px4/                        ← Feedforward fig8 controller (Python)
│   ├── quad_platforms/                   ← Platform configs (Python, submodule)
│   ├── quad_platforms_cpp/               ← Platform configs (C++, submodule)
│   ├── quad_trajectories/                ← Trajectory library (Python, submodule)
│   ├── quad_trajectories_cpp/            ← Trajectory library (C++, submodule)
│   ├── ros2_logger/                      ← CSV logging (Python, submodule)
│   ├── ros2_logger_cpp/                  ← CSV logging (C++, submodule)
│   ├── controller_params/                ← Neural network weights (contraction only)
│   ├── data_analysis/                    ← Log files, notebooks, and analysis CLI
│   └── workspace_tools/                  ← host-side experiment automation scripts
├── docs/
│   ├── *.pdf                             ← Rendered PDF documents
│   └── qmd/                              ← Quarto source documents
├── docker/
│   ├── Dockerfile
│   └── requirements.txt
├── Makefile
└── README.md
```

`build/`, `install/`, and `log/` are created at the repo root by colcon (gitignored).

---

## Docker image

| Component | Details |
|---|---|
| Base | `osrf/ros:jazzy-desktop-full` |
| ROS 2 | Jazzy |
| px4_msgs | pre-built overlay at `/opt/ws_px4_msgs` (branch `v1.16_minimal_msgs`) |
| acados | v0.5.1 at `/opt/acados` (with tera_renderer + Python interface) |
| Python | JAX, equinox, immrax, linrax, casadi, acados_template, python-control |
| C++ | Eigen3, OpenBLAS, LAPACK, autodiff (fetched by CMake) |

---

## Theory document

`docs/qmd/contraction_controller.qmd` contains the full derivation: system model,
control law, differential flatness, feedforward construction, and trajectory
definitions. Render to PDF with [Quarto](https://quarto.org/) >= 1.4:

```bash
./docs/qmd/render_pdfs.sh
```
