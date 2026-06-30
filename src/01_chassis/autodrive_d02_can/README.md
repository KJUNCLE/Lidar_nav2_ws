# AutoDrive D02 CAN chassis control

This package provides D02 CAN feedback decoding, vehicle control encoding, and a
keyboard-friendly `/cmd_vel` bridge for low-speed chassis tests.

## Keyboard control test

Bring up the CAN interfaces before launching this package. Use the vehicle CAN
bitrate configured for the chassis:

```bash
sudo ip link set can0 up type can bitrate <bitrate>
sudo ip link set can1 up type can bitrate <bitrate>
```

Build the chassis packages:

```bash
colcon build --symlink-install --packages-select \
  ros2_socketcan_msgs ros2_socketcan \
  autodrive_d02_can_msgs autodrive_d02_can
```

Start feedback on `can0`, control output on `can1`, and the `/cmd_vel` bridge:

```bash
ros2 launch autodrive_d02_can d02_keyboard_control_test.launch.py \
  feedback_interface:=can0 \
  control_interface:=can1 \
  max_linear_mps:=0.2 \
  max_angular_radps:=0.4
```

Run keyboard teleop in another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

If keyboard teleop is unavailable, publish a small command directly:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

For the first vehicle test, keep `max_linear_mps` and `max_angular_radps` low,
keep emergency stop ready, and verify `/vehicle_control_cmd` and `/to_can_bus`
before allowing motion.
The bridge sends a brake command when `/cmd_vel` is zero, and the control sender
falls back to a safe neutral/brake command when commands go stale.
