# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from dataclasses import dataclass, field
from enum import Enum

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


class ActionType(Enum):
    CARTESIAN_VELOCITY = "cartesian_velocity"
    JOINT_POSITION = "joint_position"
    JOINT_TRAJECTORY = "joint_trajectory"


class GripperActionType(Enum):
    TRAJECTORY = "trajectory"  # Use JointTrajectoryController for gripper
    ACTION = "action"  # Use GripperActionClient


@dataclass
class ROS2InterfaceConfig:
    # Namespace used by ros2_control / MoveIt2 nodes
    namespace: str = ""

    arm_joint_names: list[str] = field(
        default_factory=lambda: [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
    )
    gripper_joint_name: str = "gripper_joint"

    # Base link name for computing end effector pose / velocity
    # Only applicable for cartesian control
    base_link: str = "base_link"

    # Only applicable if velocity control is used.
    max_linear_velocity: float = 0.10
    max_angular_velocity: float = 0.25  # rad/s

    # Only applicable if position control is used.
    min_joint_positions: list[float] | None = None
    max_joint_positions: list[float] | None = None

    gripper_open_position: float = 0.0
    gripper_close_position: float = 1.0

    gripper_action_type: GripperActionType = GripperActionType.TRAJECTORY


@dataclass
class ROS2Config(RobotConfig):
    # Action type for controlling the robot. Can be 'cartesian_velocity' or 'joint_position'.
    action_type: ActionType = ActionType.JOINT_POSITION

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    # cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # ROS2 interface configuration
    ros2_interface: ROS2InterfaceConfig = field(default_factory=ROS2InterfaceConfig)

    # Home/initial pose position (in radians for joint positions, 0-1 for gripper)
    # This is a safe folded/home position where the arm starts and returns to
    home_pose: dict[str, float] | None = None

    # Move to home pose on connect
    go_home_on_connect: bool = False

    # Return to home pose on disconnect
    return_home_on_disconnect: bool = False


@RobotConfig.register_subclass("annin_ar4_mk1")
@dataclass
class AnninAR4Config(ROS2Config):
    """Annin Robotics AR4 robot configuration - extends ROS2Config with
    AR4-specific settings
    """

    action_type: ActionType = ActionType.CARTESIAN_VELOCITY

    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            gripper_joint_name="gripper_jaw1_joint",
            base_link="base_link",
            min_joint_positions=[-2.9671, -0.7330, -1.5533, -2.8798, -1.8326, -2.7053],
            max_joint_positions=[2.9671, 1.5708, 0.9076, 2.8798, 1.8326, 2.7053],
            gripper_open_position=0.014,
            gripper_close_position=0.0,
            gripper_action_type=GripperActionType.ACTION,
        ),
    )


@RobotConfig.register_subclass("so101_ros")
@dataclass
class SO101ROSConfig(ROS2Config):
    """Configuration for the ROS 2 version of SO101: https://github.com/Pavankv92/lerobot_ws."""

    action_type: ActionType = ActionType.JOINT_TRAJECTORY

    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            arm_joint_names=["1", "2", "3", "4", "5"],
            gripper_joint_name="6",
            base_link="base",
            min_joint_positions=[-1.91986, -1.74533, -1.74533, -1.65806, -2.79253],
            max_joint_positions=[1.91986, 1.74533, 1.5708, 1.65806, 2.79253],
            gripper_open_position=1.74533,
            gripper_close_position=0.0,
        ),
    )

    # Home/folded pose for SO-101 (in radians)
    # This is a safe compact position with arm folded up
    home_pose: dict[str, float] = field(default_factory=lambda: {
        "1": 0.0,      # Joint 1 (base) - centered
        "2": -0.5,     # Joint 2 (shoulder) - lowered
        "3": 1.0,      # Joint 3 (elbow) - bent inward
        "4": -0.5,     # Joint 4 (wrist pitch) - folded down
        "5": 0.0,      # Joint 5 (wrist roll) - centered
        "6": 0.87,     # Gripper - half open (50% between 0.0 and 1.74533)
    })

    # Enable home pose behavior by default for safety
    go_home_on_connect: bool = True
    return_home_on_disconnect: bool = True



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
