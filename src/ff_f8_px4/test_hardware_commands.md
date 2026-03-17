# ff_f8_px4 Hardware Commands

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
