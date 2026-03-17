# First all trajectories without feedforward, then f8_contraction with feedforward.

## Part 1: All Trajectories (No Feedforward)

```bash
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory hover         --double-speed --hover-mode 1 --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory yaw_only      --double-speed --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory circle_horz   --double-speed --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory circle_horz   --double-speed --spin --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory circle_vert   --double-speed --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory fig8_horz     --double-speed --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory fig8_vert     --double-speed --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory fig8_vert     --double-speed --short --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory helix         --double-speed --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory sawtooth      --double-speed --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory triangle      --double-speed --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory f8_contraction --double-speed --log
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory f8_contraction --double-speed --ff --log
```

## Part 2: Helix Spin

```bash
ros2 run newton_raphson_px4_cpp run_node --platform sim --trajectory helix --double-speed --spin --log
```
