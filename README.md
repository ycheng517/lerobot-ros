# LeRobot ROS

This repository provides a generic ROS 2 interface for the [LeRobot](https://github.com/huggingface/lerobot) framework. It acts as a lightweight wrapper to connect any [ros2_control](https://control.ros.org/rolling/index.html) or [MoveIt](https://moveit.ai/) compatible robot arm with the LeRobot ecosystem.

A gamepad teleoperator for 6-DoF end-effector control and a keyboard teleoperator for joint position control is also provided.

**Supported control modes:**

- Joint position with ros2_control
  - Using [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Using [position_controllers](https://control.ros.org/rolling/doc/ros2_controllers/position_controllers/doc/userdoc.html)
- End-effector velocity with MoveIt 2
  - Using [Moveit Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- Gripper control with ros2_control
  - Using [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Using [Gripper Action Controller](https://control.ros.org/jazzy/doc/ros2_controllers/gripper_controllers/doc/userdoc.html)

## Video Demo

[![lerobot-ros](https://markdown-videos-api.jorgenkh.no/url?url=https%3A%2F%2Fyoutu.be%2F8U8vDyi5IAs)](https://youtu.be/8U8vDyi5IAs)

## Prerequisites

### Software Requirements

Before getting started, ensure you have the following installed:

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) - This repo is only tested on Jazzy.
- [ros2_control](https://control.ros.org/rolling/index.html)
- If end-effector control is desired, then [MoveIt2](https://moveit.ai/install-moveit2/binary) needs to be installed

## Quickstart with Simulated SO-101

Below steps will allow you to perform keyboard teleoperation of a simulated SO-101 arm using Lerobot.

---

First, setup LeRobot and lerobot-ros in a virtual environment. Note that the Python version of the virtualenv must be compatible with your ROS version. For ROS 2 Jazzy, we use Python 3.12.

```bash
# Create and activate virtual env
conda create -y -n lerobot-ros python=3.12
conda activate lerobot-ros

# Install lerobot
git clone https://github.com/huggingface/lerobot
pip install -e lerobot

# Install lerobot-ros packages
git clone https://github.com/Neoxra/lerobot-ros
pip install -e lerobot-ros/lerobot_robot_ros
pip install -e lerobot-ros/lerobot_teleoperator_devices
```

**Note:** This fork includes:
- `lerobot-ros-teleoperate` command with state synchronization
- SO-101 hardware wrapper (`so101_follower_ros`) with home pose support
- Enhanced keyboard teleoperation with multi-key support
- MoveIt motion planning configuration (`lerobot_moveit_config`)


Then, setup the Simulated SO-101 by following instructions in: https://github.com/Pavankv92/lerobot_ws

Finally, to run all programs:

```bash
# In terminal 1, run the Gazebo simulation
ros2 launch lerobot_description so101_gazebo.launch.py

# In terminal 2, load the ros2 controllers and run MoveIt
ros2 launch lerobot_controller so101_controller.launch.py && \
  ros2 launch lerobot_moveit so101_moveit.launch.py

# In terminal 3, run lerobot-ros teleoperation with keyboard control
lerobot-ros-teleoperate \
  --robot.type=so101_ros \
  --robot.id=my_follower_arm \
  --teleop.type=keyboard_joint \
  --teleop.id=my_leader_arm \
  --display_data=true
```

**Keyboard Controls:**
- Q/A - Joint 1 (decrease/increase)
- W/S - Joint 2
- E/D - Joint 3
- R/F - Joint 4
- T/G - Joint 5
- O/L - Gripper (close/open)
- ESC - Exit

Once you have teleoperation working, you can use all standard LeRobot features as usual.

## Hardware Control with SO-101

For controlling real SO-101 hardware directly (bypassing ROS 2):

```bash
# Activate virtual environment
conda activate lerobot-ros

# Install Feetech servo SDK
pip install "feetech-servo-sdk>=1.0.0,<2.0.0"

# Run teleoperation with hardware
lerobot-ros-teleoperate \
  --robot.type=so101_follower_ros \
  --robot.port=/dev/ttyUSB0 \
  --robot.id=my_follower \
  --robot.go_home_on_connect=true \
  --robot.return_home_on_disconnect=true \
  --teleop.type=keyboard_joint_hardware \
  --teleop.id=my_leader \
  --teleop.action_increment=1.0 \
  --display_data=true
```

**Features:**
- Direct servo control via USB
- Multi-key support (hold multiple keys simultaneously)
- Configurable movement speed (`action_increment`)
- Home pose on startup/shutdown
- Joint clamping (-100 to 100 for joints, 0-100 for gripper)

**Custom Home Pose:**
```bash
--robot.home_pose="{'shoulder_pan': 0.0, 'shoulder_lift': -60.0, 'elbow_flex': 30.0, 'wrist_flex': 75.0, 'wrist_roll': 0.0, 'gripper': 10.0}"
```

## MoveIt Motion Planning

The `lerobot_moveit_config` package provides MoveIt configuration for motion planning with SO-101.

### Build MoveIt Package

```bash
cd /home/nick-bell/le_ws
colcon build --packages-select lerobot_moveit_config
source install/setup.bash
```

### Test with Demo (Fake Controllers)

```bash
ros2 launch lerobot_moveit_config demo.launch.py
```

This launches:
- move_group node (motion planning)
- RViz with MoveIt plugin
- Fake controllers for testing

### Planning Groups

- **arm**: Joints 1-5 (shoulder through wrist)
- **gripper**: Joint 6

### Named Poses

- **home**: Safe folded position
- **open** (gripper): Fully open
- **close** (gripper): Fully closed

For more details, see [lerobot_moveit_config/README.md](./lerobot_moveit_config/README.md)

## Robot Integration Guide

This section describes how to integrate other ROS-based robots with Lerobot.

### Arm Control Modes

Currently the repo supports the following arm control modes:

**Option 1: Joint Position Control**

This option uses [position_controllers](https://control.ros.org/rolling/doc/ros2_controllers/position_controllers/doc/userdoc.html) in `ros2_control`. It requires the robot to have:

- `position_controllers/JointGroupPositionController` for the robot arm joints
- `joint_state_broadcaster/JointStateBroadcaster` for joint state feedback

This option is enabled by setting `action_type` to `ActionType.JOINT_POSITION` in robot config.

**Option 2: Joint Trajectory Control**

This option uses [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) in `ros2_control`. It requires the robot to have:

- `joint_trajectory_controller/JointTrajectoryController` for the robot arm joints
- `joint_state_broadcaster/JointStateBroadcaster` for joint state feedback

This option is enabled by setting `action_type` to `ActionType.JOINT_TRAJECTORY` in robot config.

**Option 3: End-Effector Control**

This option uses [Moveit Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html) in MoveIt. It requires the robot to have:

- The `moveit_servo` node for real-time end-effector control
- `joint_trajectory_controller/JointTrajectoryController` for robot arm control
- `joint_state_broadcaster/JointStateBroadcaster` for joint state feedback

This option is enabled by setting `action_type` to `ActionType.CARTESIAN_VELOCITY` in robot config. See: [ar4_ros_driver](https://github.com/ycheng517/ar4_ros_driver) for an example of using `moveit_servo`.

### Gripper Control Modes

The repo supports two gripper control modes that can be configured via the `gripper_action_type` setting:

**Trajectory Control (`GripperActionType.TRAJECTORY`)**

- Uses `JointTrajectoryController` from ros2_control
- Publishes `JointTrajectory` messages to `/gripper_controller/joint_trajectory`

**Action Control (`GripperActionType.ACTION`)**

- Uses `GripperActionController` from ros2_control
- Sends action goals to `/gripper_controller/gripper_cmd`
- Provides feedback on whether the gripper reached its target position

### Code Changes to Lerobot-ros

Extend the `ROS2Robot` class in [robot.py](./lerobot_robot_ros/lerobot_robot_ros/robot.py).
This class can be a simple pass-through. It's just is needed to satisfy lerobot device discovery requirements.

```python
class MyRobot(ROS2Robot):
  pass
```

Then, create a config class for your robot by sub-classing `ROS2Config` in [config.py](./lerobot_robot_ros/lerobot_robot_ros/config.py).
The name of this class must be the same as your robot class, suffixed by `Config`.
You may override joint names, gripper configurations, and other parameters as needed.
An example config class for joint velocity control may look like this:

```python
from dataclasses import dataclass, field
from lerobot.common.robots.config import RobotConfig
from lerobot.common.robots.config import ROS2Config, ROS2InterfaceConfig

@RobotConfig.register_subclass("my_ros2_robot")
@dataclass
class MyRobotConfig(ROS2Config):
    action_type: ActionType = ActionType.JOINT_POSITION

    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            base_link="base_link",
            arm_joint_names=[
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
            gripper_joint_name="gripper_joint",
            gripper_open_position=0.0,
            gripper_close_position=1.0,
            max_linear_velocity=0.05,  # m/s
            max_angular_velocity=0.25,  # rad/s
        )
    )
```
