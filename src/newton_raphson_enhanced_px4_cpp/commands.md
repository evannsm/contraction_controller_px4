# Newton-Raphson Standard Controller Commands (C++)

## Build Package

```bash
cd ~/ws_px4_work
colcon build --packages-select quad_platforms_cpp quad_trajectories_cpp newton_raphson_enhanced_px4_cpp
source install/setup.bash
```

## Run Commands

### Simulation

```bash
# Hover (requires --hover-mode)
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory hover --hover-mode 1

# Yaw Only
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory yaw_only
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory yaw_only --double-speed

# Circle Horizontal
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory circle_horz
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory circle_horz --double-speed
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory circle_horz --spin
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory circle_horz --double-speed --spin

# Circle Vertical
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory circle_vert
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory circle_vert --double-speed

# Figure-8 Horizontal
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_horz
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_horz --double-speed

# Figure-8 Vertical
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_vert
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_vert --double-speed
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_vert --short
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_vert --double-speed --short

# Helix
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory helix
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory helix --double-speed
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory helix --spin
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory helix --double-speed --spin

# Sawtooth
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory sawtooth
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory sawtooth --double-speed

# Triangle
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory triangle
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory triangle --double-speed

# Figure-8 Contraction (no feedforward)
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_contraction
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_contraction --double-speed

# Figure-8 Contraction with feedforward
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_contraction --ff
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_contraction --ff --double-speed
```

### Hardware

```bash
# Hover (modes 1-4 only on hardware)
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform hw --trajectory hover --hover-mode 1
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform hw --trajectory hover --hover-mode 2
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform hw --trajectory hover --hover-mode 3
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform hw --trajectory hover --hover-mode 4

# Circle Horizontal
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform hw --trajectory circle_horz
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform hw --trajectory circle_horz --double-speed

# Helix
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform hw --trajectory helix
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform hw --trajectory helix --double-speed

# Figure-8 Contraction with feedforward
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform hw --trajectory fig8_contraction --double-speed --ff
```

## With Logging

Add `--log` to auto-generate log filename based on configuration:

```bash
# Auto-generated filename: sim_nr_std_helix_2x_spin.csv
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory helix --double-speed --spin --log

# Auto-generated filename: sim_nr_std_fig8_contraction_1x.csv
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_contraction --log

# Auto-generated filename: sim_nr_std_fig8_contraction_ff_1x.csv
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_contraction --ff --log

# Auto-generated filename: sim_nr_std_fig8_contraction_ff_2x.csv
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory fig8_contraction --ff --double-speed --log

# Custom filename
ros2 run newton_raphson_enhanced_px4_cpp run_node --platform sim --trajectory helix --log --log-file my_experiment
```

## Arguments Reference

| Argument | Required | Values | Description |
|----------|----------|--------|-------------|
| `--platform` | Yes | `sim`, `hw` | Platform type |
| `--trajectory` | Yes | `hover`, `yaw_only`, `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix`, `sawtooth`, `triangle`, `fig8_contraction` | Trajectory type |
| `--hover-mode` | If hover | `1-8` (sim), `1-4` (hw) | Hover position |
| `--double-speed` | No | flag | 2x trajectory speed |
| `--short` | No | flag | Short fig8_vert variant |
| `--spin` | No | flag | Enable yaw rotation |
| `--ff` | No | flag | Enable feedforward injection and mark log filename with `_ff` (only valid with `fig8_contraction`) |
| `--log` | No | flag | Enable data logging |
| `--log-file` | No | string | Custom log filename |
| `--flight-period` | No | float | Override default flight duration (sim: 30s, hw: 60s) |
