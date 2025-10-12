from dataclasses import dataclass, field

from lerobot.teleoperators import TeleoperatorConfig
from lerobot.teleoperators.keyboard import KeyboardTeleopConfig


@TeleoperatorConfig.register_subclass("keyboard_joint")
@dataclass
class KeyboardJointTeleopConfig(KeyboardTeleopConfig):
    arm_action_keys: list[str] = field(default_factory=lambda: [f"{i}.pos" for i in range(1, 6)])
    gripper_action_key: str = "gripper.pos"

    # The amount by which a joint action changes when a key is pressed.
    action_increment: float = 0.02
