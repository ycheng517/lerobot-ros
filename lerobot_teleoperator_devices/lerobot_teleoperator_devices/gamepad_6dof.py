from typing import Any

import numpy as np
from lerobot.teleoperators import Teleoperator

from .config_gamepad_6dof import Gamepad6DOFTeleopConfig
from .gamepad_6dof_utils import GamepadController6DOF


class Gamepad6DOFTeleop(Teleoperator):
    """
    Teleop class to use gamepad inputs for 6-DOF control.
    Mapping:
    - Left joystick: Linear X/Y movement
    - Right joystick: Angular X/Y rotation
    - LR bumpers: Angular Z rotation (yaw)
    - LR triggers: Linear Z movement (up/down)
    - Button: Gripper control (nominally open, close when pressed)
    """

    config_class = Gamepad6DOFTeleopConfig
    name = "gamepad_6dof"

    def __init__(self, config: Gamepad6DOFTeleopConfig):
        super().__init__(config)
        self.config = config
        self.robot_type = config.type

        self.gamepad: GamepadController6DOF | None = None

    @property
    def action_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {
                    "linear_x.vel": 0,
                    "linear_y.vel": 1,
                    "linear_z.vel": 2,
                    "angular_x.vel": 3,
                    "angular_y.vel": 4,
                    "angular_z.vel": 5,
                    "gripper.pos": 6,
                },
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {
                    "linear_x.vel": 0,
                    "linear_y.vel": 1,
                    "linear_z.vel": 2,
                    "angular_x.vel": 3,
                    "angular_y.vel": 4,
                    "angular_z.vel": 5,
                },
            }

    @property
    def feedback_features(self) -> dict:
        return {}

    def connect(self) -> None:
        self.gamepad = GamepadController6DOF()
        self.gamepad.start()

    def get_action(self) -> dict[str, Any]:
        # Update the controller to get fresh inputs
        if self.gamepad is None:
            raise RuntimeError("Gamepad is not connected. Please call connect() first.")

        self.gamepad.update()

        # Get movement deltas and rotations from the controller
        delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw = self.gamepad.get_6dof_deltas()

        # Create action from gamepad input
        gamepad_action = np.array(
            [delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw], dtype=np.float32
        )

        action_dict = {
            "linear_x.vel": gamepad_action[0],
            "linear_y.vel": gamepad_action[1],
            "linear_z.vel": gamepad_action[2],
            "angular_x.vel": gamepad_action[3],
            "angular_y.vel": gamepad_action[4],
            "angular_z.vel": gamepad_action[5],
        }

        # Default gripper action is to stay open
        if self.config.use_gripper:
            gripper_command = self.gamepad.gripper_command()
            action_dict["gripper.pos"] = gripper_command

        return action_dict

    def disconnect(self) -> None:
        """Disconnect from the gamepad."""
        if self.gamepad is not None:
            self.gamepad.stop()
            self.gamepad = None

    def is_connected(self) -> bool:
        """Check if gamepad is connected."""
        return self.gamepad is not None

    def calibrate(self) -> None:
        """Calibrate the gamepad."""
        # No calibration needed for gamepad
        pass

    def is_calibrated(self) -> bool:
        """Check if gamepad is calibrated."""
        # Gamepad doesn't require calibration
        return True

    def configure(self) -> None:
        """Configure the gamepad."""
        # No additional configuration needed
        pass

    def send_feedback(self, feedback: dict) -> None:
        """Send feedback to the gamepad."""
        # Gamepad doesn't support feedback
        pass
