from dataclasses import dataclass, field

from lerobot.teleoperators import TeleoperatorConfig
from lerobot.teleoperators.keyboard import KeyboardTeleopConfig


@TeleoperatorConfig.register_subclass("keyboard_joint")
@dataclass
class KeyboardJointTeleopConfig(KeyboardTeleopConfig):
    # Default to ROS 2 joint names (numeric)
    arm_action_keys: list[str] = field(default_factory=lambda: [f"{i}.pos" for i in range(1, 6)])
    gripper_action_key: str = "gripper.pos"

    # The amount by which a joint action changes when a key is pressed.
    action_increment: float = 0.02


@TeleoperatorConfig.register_subclass("keyboard_joint_hardware")
@dataclass
class KeyboardJointHardwareTeleopConfig(KeyboardTeleopConfig):
    # For real SO101 hardware with descriptive motor names
    arm_action_keys: list[str] = field(default_factory=lambda: [
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos"
    ])
    gripper_action_key: str = "gripper.pos"

    # The amount by which a joint action changes when a key is pressed.
    # Higher values = faster movement. Range is -100 to 100 for joints at 60fps.
    # 1.0 = 1% of full range per frame = 60%/second
    action_increment: float = 1.0

    # Gripper increment (0-100 range, can be different from arm joints)
    gripper_increment: float = 2.0
