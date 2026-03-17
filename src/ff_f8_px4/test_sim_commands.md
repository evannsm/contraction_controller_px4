# ff_f8_px4 Simulation Commands

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
