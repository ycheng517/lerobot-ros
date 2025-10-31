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

"""
Wrapper for SO101Follower robot that adds home pose functionality without modifying the core lerobot library.
"""

import logging
import time
from dataclasses import dataclass, field

from lerobot.robots import RobotConfig
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

logger = logging.getLogger(__name__)


@RobotConfig.register_subclass("so101_follower_ros")
@dataclass
class SO101FollowerROSConfig(SO101FollowerConfig):
    """Extended SO101 configuration with home pose support."""

    # Home/folded pose position (in normalized coordinates -100 to 100 for joints, 0-100 for gripper)
    # This is a safe folded position where the arm is compact
    home_pose: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 0.0,  # Centered
            "shoulder_lift": -60.0,  # Lowered
            "elbow_flex": 30.0,  # Bent inward
            "wrist_flex": 75.0,  # Folded down
            "wrist_roll": 0.0,  # Centered
            "gripper": 10.0,  # Half open
        }
    )

    # Move to home pose on connect
    go_home_on_connect: bool = True

    # Return to home pose on disconnect
    return_home_on_disconnect: bool = True

    # Disable max_relative_target for smoother, faster movements
    # Set to None to allow full range movements, or a value like 10.0 to limit speed
    max_relative_target: float | dict[str, float] | None = None


class SO101FollowerROS(SO101Follower):
    """
    Wrapper for SO101Follower with home pose functionality.
    This extends the base SO101Follower without modifying the core lerobot library.
    """

    config_class = SO101FollowerROSConfig
    name = "so101_follower_ros"

    def __init__(self, config: SO101FollowerROSConfig):
        super().__init__(config)
        self.config = config

    def _go_to_home_pose(self) -> None:
        """Move robot to the configured home pose."""
        home_action = {f"{motor}.pos": pos for motor, pos in self.config.home_pose.items()}
        self.send_action(home_action)

    def connect(self, calibrate: bool = True) -> None:
        """
        Connect to the robot and optionally move to home pose.
        Extends the base connect() with home pose functionality.
        """
        # Call parent connect (which handles calibration, cameras, configuration, torque)
        super().connect(calibrate=calibrate)

        # Move to home pose if configured
        if self.config.go_home_on_connect:
            logger.info(f"Moving {self} to home pose...")
            self._go_to_home_pose()
            time.sleep(1)  # Wait for movement to start

    def disconnect(self):
        """
        Disconnect from the robot and optionally return to home pose.
        Extends the base disconnect() with home pose functionality.
        """
        if not self.is_connected:
            from lerobot.utils.errors import DeviceNotConnectedError

            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Return to home pose if configured
        if self.config.return_home_on_disconnect:
            logger.info(f"Returning {self} to home pose...")
            try:
                self._go_to_home_pose()
                time.sleep(2)  # Wait for movement to complete
            except Exception as e:
                logger.warning(f"Failed to return to home pose: {e}")

        # Call parent disconnect
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
