# contraction_controller_px4

A ROS 2 offboard controller for PX4-based quadrotors using a contraction-metric
neural-network policy for trajectory tracking.

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

### 4. Controller weights

Place the trained network files at:

```
src/controller_params/
├── arch.txt       # network architecture, e.g.  10 128 softplus 128 softplus 4
├── model.eqx      # equinox-serialised weights
└── ix_ut.npy      # equilibrium input
```

---

## Quickstart

```bash
git clone --recurse-submodules git@github.com:evannsm/contraction_controller_px4.git
cd contraction_controller_px4
make build      # build Docker image (~10 min, once)
make run        # start container
make build_ros  # build ROS 2 workspace inside container (once)
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

### Building the ROS 2 workspace

```bash
make build_ros                        # build all packages
make build_ros PACKAGES="pkg1 pkg2"   # build specific packages only
make clean_build_ros                  # wipe build/install/log and rebuild from scratch
```

Rebuild is required after:
- Adding new source files or packages
- Changing `package.xml` or `setup.py`

With `--symlink-install`, edits to existing Python source files are picked up
immediately without rebuilding.

### Running the controller

```bash
make run_controller PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1
```

All variables and their defaults:

| Variable | Default | Description |
|---|---|---|
| `PLATFORM` | `sim` | `sim` or `hw` |
| `TRAJECTORY` | `hover` | see trajectory table below |
| `HOVER_MODE` | `1` | 1–8, required when `TRAJECTORY=hover` |
| `FLIGHT_PERIOD` | *(auto)* | flight duration in seconds (30 sim / 60 hw) |
| `CONTROLLER_DIR` | *(auto)* | path to weights dir inside container (default: `src/controller_params`) |
| `LOG` | *(off)* | set to any non-empty value to enable CSV logging |
| `LOG_FILE` | *(auto)* | log filename stem (without `.csv`); auto-generated if omitted |
| `NO_FEEDFORWARD` | `0` | set to `1` to disable the `u_ff` term (suffix `_no_ff` in auto filename) |

#### Examples

```bash
# Simulation — hover at position 1
make run_controller PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1

# Simulation — hover at position 4, 45-second flight
make run_controller PLATFORM=sim TRAJECTORY=hover HOVER_MODE=4 FLIGHT_PERIOD=45

# Simulation — figure-eight
make run_controller PLATFORM=sim TRAJECTORY=fig8

# Simulation — trefoil knot
make run_controller PLATFORM=sim TRAJECTORY=trefoil

# Simulation — spiral
make run_controller PLATFORM=sim TRAJECTORY=spiral

# Simulation — figure-eight with heading
make run_controller PLATFORM=sim TRAJECTORY=fig8_heading

# Hardware — hover
make run_controller PLATFORM=hw TRAJECTORY=hover HOVER_MODE=1

# Hardware — figure-eight
make run_controller PLATFORM=hw TRAJECTORY=fig8

# Custom weights
make run_controller PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1 \
    CONTROLLER_DIR=/workspace/src/my_weights

# With data logging (auto-generated filename)
make run_controller PLATFORM=sim TRAJECTORY=fig8 LOG=1

# With data logging and custom filename
make run_controller PLATFORM=sim TRAJECTORY=trefoil LOG=1 LOG_FILE=my_run
```

Or drop into the container and run the node directly:

```bash
make attach
ros2 run contraction_controller_px4 run_node \
    --platform sim --trajectory figure_eight --flight-period 60  # or fig8
```

---

## Available trajectories

| `TRAJECTORY` | Shape | Period | Sim scale | HW scale |
|---|---|---|---|---|
| `hover` | Static point | — | — | — |
| `fig8` (or `figure_eight`) | Horizontal figure-eight | 10 s | R = 2.0 m | R = 0.5 m |
| `trefoil` | 3-D trefoil knot | 15 s | R = 1.0 m | R = 0.3 m |
| `spiral` | Horizontal circle | 7.5 s | R = 2.0 m | R = 0.5 m |
| `fig8_heading` | Figure-eight + yaw tracking | 15 s | R = 1.5 m | R = 0.4 m |

Heights are set automatically: −3.0 m NED (simulation) or −0.85 m NED (hardware).

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

## Usage walkthrough

### Step 1 — Start the simulator and bridge (host terminals)

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

### Step 4 — Run the controller

```bash
make run_controller PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1
```

---

## Flight phases

Every run follows four automatic phases:

```
HOVER (10 s) → CUSTOM (flight-period) → RETURN (10 s) → LAND
```

| Phase | Mode | Notes |
|---|---|---|
| **HOVER** | PX4 position setpoint | JIT compilation of control law and trajectory feedforward runs during this window — no latency on first control tick |
| **CUSTOM** | Offboard bodyrate control | Contraction NN controller tracks the selected trajectory |
| **RETURN** | PX4 position setpoint | Vehicle returns to hover point |
| **LAND** | PX4 land command | Descends and disarms automatically |

---

## Data logging

Pass `LOG=1` to enable CSV logging. The file is written to
`src/data_analysis/log_files/` (on the host, since the workspace is bind-mounted)
when the node shuts down (Ctrl+C or end of flight).

### Logged columns

| Group | Columns | Description |
|---|---|---|
| Timing | `time`, `traj_time`, `comp_time` | program time, trajectory time, controller compute time (s) |
| True state | `x`, `y`, `z`, `roll`, `pitch`, `yaw` | NED position and Euler angles (m, rad) |
| True velocity | `vx`, `vy`, `vz` | NED velocity (m/s) |
| True thrust | `f` | specific thrust = F/m (N/kg) |
| True rates | `p`, `q`, `r` | body angular rates from odometry (rad/s) |
| Reference state | `x_ref`, `y_ref`, `z_ref`, `roll_ref`, `pitch_ref`, `yaw_ref` | x_ff position and angles |
| Reference velocity | `vx_ref`, `vy_ref`, `vz_ref` | x_ff velocity |
| Reference thrust | `f_ref` | x_ff specific thrust (N/kg) |
| Reference control | `u_ff_df`, `u_ff_dphi`, `u_ff_dth`, `u_ff_dpsi` | feedforward rates (N/kg/s, rad/s) |
| Commands | `throttle`, `p_cmd`, `q_cmd`, `r_cmd` | normalized throttle and body-rate commands sent to PX4 |

Logging runs at 10 Hz (during CUSTOM phase only).

### Auto-generated filename format

```
{platform}_contraction_{trajectory}[_mode{N}][_no_ff].csv
# e.g.:  sim_contraction_figure_eight.csv
#        hw_contraction_hover_mode1.csv
#        sim_contraction_figure_eight_no_ff.csv   (NO_FEEDFORWARD=1)
```

The `_no_ff` suffix is appended automatically when `NO_FEEDFORWARD=1` is set.

---

## How it works

### Control law

```
u(t) = π(x(t)) − π(x_ff(t)) + u_ff(t)
```

`π` is the trained neural network.  `K(t)` is a time-varying LQR gain computed
by linearising the NED quadrotor dynamics around the current vehicle state via
`jax.jacfwd` and solving the continuous-time algebraic Riccati equation with
`python-control`'s `ctrl.lqr` (updates at 10 Hz).
`x_ff(t)` and `u_ff(t)` are the feedforward state and control derived from the
reference trajectory. The error `e = x − x_ff` converges exponentially under the
contraction metric certificate.

### State (10-D) and control (4-D)

```
x   = [px, py, pz, vx, vy, vz, f, φ, θ, ψ]
u   = [ḟ, φ̇, θ̇, ψ̇]
```

`f = F/m` is the mass-normalised specific thrust (≈ 9.81 N/kg at hover).
The control outputs are *rates*, not values.

### Thrust integration

The network outputs `ḟ` (specific force rate, N/kg/s).  Actual thrust (N) is
integrated each tick with the correct mass scaling:

```
F_new = F_last + dt × ḟ × mass
```

### Feedforward via differential flatness

Each trajectory is defined as a flat output `σ(t) = [px, py, pz, ψ]`.
The full `x_ff` and `u_ff` are computed automatically by applying `jax.jacfwd`
twice — no manual derivation needed:

```
velocity          : dσ/dt
acceleration      : d²σ/dt²
specific thrust   : f   = √(ax² + ay² + (az − g)²)
pitch / roll      : θ   = arcsin(−ax / f),  φ = arctan2(ay, g − az)
feedforward rates : u_ff = d/dt [f, φ, θ, ψ]
```

Both `_flat_to_x_u` and `contraction_control` are JIT-compiled with XLA during
the hover cushion before flight begins.

### Adding a new trajectory

Add a function to `src/contraction_controller_px4/contraction_controller_px4/trajectories.py`:

```python
def my_traj(t: float, ctx: TrajContext) -> jnp.ndarray:
    height = SIM_HEIGHT if ctx.sim else HW_HEIGHT
    px = ...   # smooth function of t
    py = ...
    pz = -height
    psi = 0.0
    return jnp.array([px, py, pz, psi], dtype=jnp.float32)
```

Add `MY_TRAJ = "my_traj"` to `TrajectoryType` and register it in `TRAJ_REGISTRY`.
Run `make build_ros` to pick it up, then use `TRAJECTORY=my_traj`.

---

## Repository structure

```
contraction_controller_px4/        ← repo root = ROS 2 workspace root
├── src/
│   ├── contraction_controller_px4/    ← main package
│   │   ├── contraction_controller_px4/
│   │   │   ├── ros2px4_node.py        ← ROS 2 node
│   │   │   ├── controller.py          ← contraction control law + NN loader
│   │   │   ├── trajectories.py        ← trajectory definitions + registry
│   │   │   └── run_node.py            ← entry point / argument parsing
│   │   └── new_project/               ← training artefacts (optional reference)
│   ├── controller_params/             ← network weights (arch.txt, model.eqx, ix_ut.npy)
│   ├── quad_platforms/                ← submodule: platform configs (mass, thrust curve)
│   └── contraction_controller_px4_utils/  ← PX4 helpers, yaw unwrapping
├── docs/
│   └── contraction_controller.qmd     ← theory document (renders to PDF via Quarto)
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
| Python venv | `/opt/px4-venv` — JAX, equinox, immrax, linrax (`jax-0.9-support` branches), python-control |

---

## Theory document

`docs/contraction_controller.qmd` contains the full derivation: system model,
control law, differential flatness, feedforward construction, and trajectory
definitions. Render to PDF with [Quarto](https://quarto.org/) ≥ 1.4:

```bash
cd docs && quarto render contraction_controller.qmd
```
