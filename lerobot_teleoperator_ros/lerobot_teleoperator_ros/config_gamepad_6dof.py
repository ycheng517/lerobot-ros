from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("gamepad_6dof")
@dataclass
class Gamepad6DOFTeleopConfig(TeleoperatorConfig):
    use_gripper: bool = True
