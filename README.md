# contraction_controller_px4

A ROS 2 offboard controller for the PX4-based quadrotor that uses a
contraction-metric neural-network policy for trajectory tracking.

---

## Prerequisites

Before using this package, the following must be in place on the **host machine**:

### 1. Docker Engine

Install Docker Engine (not Docker Desktop) for Linux:

```bash
# Official install script
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER   # log out and back in after this
```

Verify: `docker run hello-world`

### 2. PX4-Autopilot (cloned and built on the host)

The container mounts your existing PX4 build — it does **not** build PX4
from scratch. Clone and build it once on the host:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh          # installs system deps (run once)
make px4_sitl gz_x500               # confirm the sim builds/runs on the host
```

If your PX4-Autopilot is somewhere other than `~/PX4-Autopilot`, pass the path
when running make targets:

```bash
make run_gui PX4_ROOT=/path/to/PX4-Autopilot
```

### 3. X11 / display (for Gazebo GUI)

On a native Linux desktop this works out of the box.  The `make run_gui`
target calls `xhost +local:docker` automatically.

If running over SSH with X forwarding, make sure your SSH session was started
with `ssh -X` or `ssh -Y`.

### 4. `ws_px4_work` ROS 2 workspace

This package lives inside `ws_px4_work/src/` alongside its sibling packages
(`quad_platforms`, `quad_trajectories`, `newton_raphson_px4_utils`, `px4_msgs`,
etc.).  The workspace does **not** need to be built on the host — it is built
inside the container with `make build_ros`.

### 5. Controller weights

The trained neural-network weights must be present at:

```
ws_px4_work/src/new_project/Controller 1/Controller/
├── arch.txt       # 10 128 softplus 128 softplus 4
├── model.eqx      # equinox-serialised weights
└── ix_ut.npy      # auxiliary training artefact
```

This path is automatically mounted into the container via the workspace volume.

---

## Quick-start with Docker

All make commands are run from the **docker directory**:

```bash
cd ws_px4_work/src/contraction_controller_px4/docker
```

### 1. Build the Docker image

Downloads and installs ROS 2 Jazzy, Gazebo Harmonic, MicroXRCE-DDS-Agent,
immrax, linrax, and JAX inside a single image (~several GB, takes a few minutes):

```bash
make build
```

### 2. Start the container with GUI (for Gazebo)

```bash
make run_gui
```

This mounts:
- `ws_px4_work/` → `/workspace/ws_px4_work`
- `~/PX4-Autopilot/` → `/workspace/PX4-Autopilot`

Override the PX4 path if needed:

```bash
make run_gui PX4_ROOT=/path/to/PX4-Autopilot
```

### 3. Build the ROS 2 workspace (first time only)

```bash
make build_ros
```

### 4. Launch PX4 Gazebo simulation (terminal tab 1)

```bash
make sim
```

This runs `make px4_sitl gz_x500` inside the container.  A Gazebo window with
the x500 drone should appear on your display.

### 5. Start the MicroXRCE-DDS bridge (terminal tab 2)

```bash
make bridge
```

This runs `MicroXRCEAgent udp4 -p 8888`, connecting PX4 to ROS 2.

Or launch both together in the background:

```bash
make launch_sim_and_bridge
```

### 6. Run the contraction controller (terminal tab 3)

```bash
# Hover (sim)
make run_controller PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1

# Circle trajectory (sim)
make run_controller PLATFORM=sim TRAJECTORY=circle_horz

# Helix (sim)
make run_controller PLATFORM=sim TRAJECTORY=helix

# Hardware
make run_controller PLATFORM=hw TRAJECTORY=hover HOVER_MODE=1
```

### Other useful commands

```bash
make attach          # open a bash shell in the running container
make stop            # stop the container
make kill            # force-kill the container
```

---

## Manual usage (without Docker)

Requires ROS 2 Jazzy, immrax (jax-0.9-support), and linrax (jax-0.9-support)
installed on the host.

```bash
cd ws_px4_work
colcon build --symlink-install
source install/setup.bash
ros2 run contraction_controller_px4 run_node \
    --platform sim --trajectory circle_horz
```

Full argument reference:

```
--platform        sim | hw                          (required)
--trajectory      hover | circle_horz | fig8_vert | helix | ...  (required)
--hover-mode      1-8                               (required when --trajectory=hover)
--controller-dir  /path/to/Controller               (default: new_project/Controller 1/Controller)
--flight-period   <seconds>                         (default: 30 sim / 60 hw)
```

---

## Theory

See `docs/contraction_controller.qmd` (or the compiled PDF) for a full
derivation of the contraction tracking law:

```
u(t) = control(x(t), π) - control(x_ff(t), π) + u_ff(t)
```

To compile the PDF (requires Quarto ≥ 1.4):

```bash
cd docs && quarto render contraction_controller.qmd
```

---

## Dependencies

| Dependency | Source | Purpose |
|---|---|---|
| ROS 2 Jazzy | `osrf/ros:jazzy-desktop-full` | middleware + Gazebo Harmonic |
| px4_msgs | `ws_px4_work/src/px4_msgs` | PX4 message types |
| quad_platforms / quad_trajectories | `ws_px4_work/src/` | platform config & reference trajectories |
| newton_raphson_px4_utils | `ws_px4_work/src/` | PX4 flight-phase and yaw utilities |
| immrax (`jax-0.9-support`) | GitHub: Akash-Harapanahalli/immrax | `NeuralNetwork` loader |
| linrax (`jax-0.9-support`) | GitHub: gtfactslab/linrax | linear systems support |
| JAX / equinox | pip | neural-network inference |
| MicroXRCE-DDS-Agent | built from source in Docker | ROS 2 ↔ PX4 uXRCE bridge |
