# contraction_controller_px4

A ROS 2 offboard controller for the PX4-based quadrotor using a
contraction-metric neural-network policy for trajectory tracking.

---

## Prerequisites (host machine)

### 1. Docker Engine

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER   # log out and back in
```

### 2. PX4-Autopilot (built on host)

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh
make px4_sitl gz_x500           # confirm it runs
```

### 3. MicroXRCE-DDS-Agent (built on host)

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install && sudo ldconfig
```

### 4. Controller weights

Must be present at:

```
ws_px4_work/src/new_project/Controller 1/Controller/
├── arch.txt       # 10 128 softplus 128 softplus 4
├── model.eqx      # equinox-serialised weights
└── ix_ut.npy
```

---

## Usage

All make commands run from `ws_px4_work/src/`:

```bash
cd ~/ws_px4_work/src
```

### Step 1 — Build the Docker image (once)

```bash
make build
```

The image contains:
- ROS 2 Jazzy
- `px4_msgs` pre-built as an upstream overlay
- immrax + linrax (jax-0.9-support)
- JAX / equinox

### Step 2 — On the host: start sim and bridge

**Terminal 1:**
```bash
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

**Terminal 2:**
```bash
MicroXRCEAgent udp4 -p 8888
```

### Step 3 — Start the container

```bash
make run
```

Mounts `ws_px4_work/` → `/workspace` so `build/install/log` persist on host.
Uses `--net host` so the container sees all host ROS 2 topics.

### Step 4 — Build the ROS 2 workspace (first time only)

```bash
make build_ros
```

### Step 5 — Run the controller

Via make:
```bash
make run_controller PLATFORM=sim TRAJECTORY=hover HOVER_MODE=1
make run_controller PLATFORM=sim TRAJECTORY=circle_horz
make run_controller PLATFORM=sim TRAJECTORY=helix
make run_controller PLATFORM=hw  TRAJECTORY=hover HOVER_MODE=1
```

Or enter the container and run manually:
```bash
make attach
ros2 run contraction_controller_px4 run_node --platform sim --trajectory hover --hover-mode 1
```

### Other commands

```bash
make attach    # shell into the running container
make stop      # stop the container
make kill      # force-kill the container
```

---

## Controller arguments

```
--platform        sim | hw                                      (required)
--trajectory      hover | circle_horz | fig8_vert | helix | ...  (required)
--hover-mode      1-8                   (required when --trajectory=hover)
--controller-dir  /path/to/Controller   (default: new_project/Controller 1/Controller)
--flight-period   <seconds>             (default: 30 sim / 60 hw)
```

---

## Theory

See `docs/contraction_controller.qmd` for the full derivation.

```
u(t) = control(x(t), π) - control(x_ff(t), π) + u_ff(t)
```

Compile the PDF (requires Quarto ≥ 1.4):

```bash
cd docs && quarto render contraction_controller.qmd
```

---

## Dependencies

| Dependency | Where | Purpose |
|---|---|---|
| ROS 2 Jazzy | Docker image | middleware |
| px4_msgs | pre-built in image (`/opt/ws_px4_msgs`) | PX4 message types |
| quad_platforms / quad_trajectories | `ws_px4_work/src/` | platform config & trajectories |
| newton_raphson_px4_utils | `ws_px4_work/src/` | PX4 utilities |
| immrax (`jax-0.9-support`) | Docker image (pip) | `NeuralNetwork` loader |
| linrax (`jax-0.9-support`) | Docker image (pip) | linear systems support |
| JAX / equinox | Docker image (pip) | neural-network inference |
| MicroXRCE-DDS-Agent | host | ROS 2 ↔ PX4 bridge |
