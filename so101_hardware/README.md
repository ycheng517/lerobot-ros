# SO-101 Hardware Interface

C++ hardware interface plugin for the SO-101 robot with Feetech STS3215 servos.

## Overview

This package provides a pure C++ hardware interface plugin that integrates with ros2_control. It uses the [SCServo_Linux](https://github.com/adityakamath/SCServo_Linux) C++ SDK for direct communication with Feetech servos.

**Key Features:**
- ✅ Pure C++ implementation (no Python/GIL threading issues)
- ✅ Fast controller loading (< 1 second)
- ✅ 100 Hz real-time control loop
- ✅ Calibration-aware motor mapping to URDF limits
- ✅ Automatic torque management

## Architecture

- **C++ Plugin**: Implements `hardware_interface::SystemInterface` for ros2_control
- **SCServo SDK**: Uses C++ Feetech servo communication library
- **Motor Calibration**: Maps raw motor positions to URDF joint limits using LeRobot calibration data

### Joint Mapping

| ROS 2 Joint Name | Motor ID | URDF Limits (rad) |
|------------------|----------|-------------------|
| shoulder_pan     | 1        | [-1.92, 1.92]     |
| shoulder_lift    | 2        | [-1.75, 1.75]     |
| elbow_flex       | 3        | [-1.75, 1.57]     |
| wrist_flex       | 4        | [-1.66, 1.66]     |
| wrist_roll       | 5        | [-2.79, 2.79]     |
| gripper          | 6        | [0, 1] (normalized) |

## Setup

### 1. Calibrate Robot

First, calibrate your robot using LeRobot's calibration tool:

```bash
# Activate conda environment
source ~/miniforge3/bin/activate lerobot_ros

# Run calibration (follow on-screen instructions)
cd ~/le_ws
./calibrate_and_install.sh my_follower /dev/ttyACM1
```

This creates calibration data at:
- `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/my_follower.json`
- `~/le_ws/src/lerobot-ros/lerobot_robot_ros/so101_moveit/config/motor_calibration.yaml`

### 2. Build Hardware Interface

```bash
cd ~/le_ws
colcon build --packages-select so101_hardware so101_moveit
source install/setup.bash
```

### 3. Launch with MoveIt

```bash
# Check robot port
ls -la /dev/ttyACM*

# Launch MoveIt with real hardware
ros2 launch so101_moveit demo.launch.py port:=/dev/ttyACM1
```

## Usage

### Launch Parameters

```bash
ros2 launch so101_moveit demo.launch.py \
  port:=/dev/ttyACM1 \
  use_fake_hardware:=false \
  calibration_file:=/path/to/motor_calibration.yaml
```

Parameters:
- `port`: Serial port for Feetech servo bus (default: `/dev/ttyACM0`)
- `use_fake_hardware`: Set to `false` for real hardware (default: `false`)
- `calibration_file`: Path to motor calibration YAML (default: package config)

### Verify Controllers

```bash
# Check controller status
ros2 control list_controllers

# Expected output:
# arm_controller          [active]
# gripper_controller      [active]
# joint_state_broadcaster [active]

# Monitor joint states
ros2 topic echo /joint_states
```

## Calibration Formula

The hardware interface maps raw motor positions to radians using URDF-limit-aware calibration:

```cpp
// For regular joints:
progress = (raw_position - range_min) / (range_max - range_min)
radians = progress * (urdf_upper - urdf_lower) + urdf_lower

// For gripper:
normalized = (raw_position - range_min) / (range_max - range_min)  // [0, 1]
```

This ensures:
- ✅ Joint angles never exceed URDF limits
- ✅ Full motor range maps to full URDF range
- ✅ Calibration captures actual physical limits

## Implementation Details

### Lifecycle States

1. **on_init**: Validates joint interfaces, loads parameters
2. **on_configure**: Connects to servo bus, loads calibration
3. **on_activate**: Reads current positions, enables torque
4. **read**: Reads motor positions at 100 Hz via `SMS_STS::FeedBack()`
5. **write**: Writes motor commands via `SMS_STS::SyncWritePosEx()`
6. **on_deactivate**: Disables motor torque

### Calibration File Format

```yaml
shoulder_pan:
  id: 1
  drive_mode: 0
  homing_offset: 427
  range_min: 792
  range_max: 3251
shoulder_lift:
  id: 2
  drive_mode: 0
  homing_offset: 666
  range_min: 824
  range_max: 3202
# ... etc
```

## Troubleshooting

### Serial Port Permissions

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect

# Or temporarily:
sudo chmod 666 /dev/ttyACM1
```

### Find Robot Port

```bash
# List available serial ports
ls -la /dev/ttyACM*

# Use LeRobot's port finder
lerobot-find-port
```

### Motor Connection Issues

If motors aren't responding:
1. Check power supply is connected and ON
2. Verify port with `lsof /dev/ttyACM1`
3. Check motor IDs are 1-6
4. Try power cycling the robot

### Joint States Not Publishing

If `/joint_states` shows all zeros:
1. Verify port is correct: `ls /dev/ttyACM*`
2. Check hardware activated: `ros2 control list_hardware_interfaces`
3. Check controllers loaded: `ros2 control list_controllers`
4. View logs: `ros2 node list` and check for errors

### RViz Visualization Wrong

If RViz doesn't match physical robot:
1. Check calibration was run recently
2. Verify motor_calibration.yaml has correct values
3. Rebuild: `colcon build --packages-select so101_hardware so101_moveit`
4. Ensure you moved robot through FULL range during calibration

## Development

### Building from Source

```bash
cd ~/le_ws
colcon build --packages-select so101_hardware --cmake-clean-first
source install/setup.bash
```

### Dependencies

C++ Dependencies (via CMakeLists.txt):
- `hardware_interface`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`
- SCServo SDK (included in `scservo_sdk/`)

## License

Apache License 2.0 - Copyright 2024 The HuggingFace Inc. team
