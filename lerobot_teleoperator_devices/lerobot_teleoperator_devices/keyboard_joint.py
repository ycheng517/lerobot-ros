from queue import Queue
from typing import Any

from lerobot.teleoperators.keyboard import KeyboardTeleop
from lerobot.utils.errors import DeviceNotConnectedError

from .config_keyboard_joint import KeyboardJointTeleopConfig, KeyboardJointHardwareTeleopConfig


class KeyboardJointTeleop(KeyboardTeleop):
    """
    Teleop class to use keyboard inputs for end effector control.
    Designed to be used with the `So100FollowerEndEffector` robot.
    """

    config_class = KeyboardJointTeleopConfig
    name = "keyboard_ee"

    def __init__(self, config: KeyboardJointTeleopConfig):
        super().__init__(config)
        self.config = config
        self.misc_keys_queue: Queue[Any] = Queue()
        # Initialize all joint actions to 0.0
        self.curr_joint_actions = dict.fromkeys(self.action_features["names"].keys(), 0.0)
        # Initialize gripper to middle position (50 for RANGE_0_100 mode which expects 0-100)
        if self.config.gripper_action_key:
            self.curr_joint_actions[self.config.gripper_action_key] = 50.0

    @property
    def action_features(self) -> dict:
        n_joints = len(self.config.arm_action_keys)
        action_names = self.config.arm_action_keys.copy()
        if self.config.gripper_action_key:
            n_joints += 1
            action_names.append(self.config.gripper_action_key)
        action_features = dict.fromkeys(action_names, float)
        return {
            "dtype": "float32",
            "shape": (n_joints,),
            "names": action_features,
        }

    def _on_press(self, key):
        if hasattr(key, "char"):
            key = key.char
        self.event_queue.put((key, True))

    def _on_release(self, key):
        if hasattr(key, "char"):
            key = key.char
        self.event_queue.put((key, False))

    def get_action(self) -> dict[str, Any]:
        """Returns the current joint actions based on the pressed keys.
        Supports up to 7 joints and a gripper.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "KeyboardTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        self._drain_pressed_keys()

        # Key mappings: key -> (joint_index, direction)
        # direction: -1 for decrease, +1 for increase
        key_mappings = {
            "q": (0, -1),
            "a": (0, 1),
            "w": (1, -1),
            "s": (1, 1),
            "e": (2, -1),
            "d": (2, 1),
            "r": (3, -1),
            "f": (3, 1),
            "t": (4, -1),
            "g": (4, 1),
            "y": (5, -1),
            "h": (5, 1),
        }

        # Add 7th joint keys if available
        if len(self.config.arm_action_keys) > 6:
            key_mappings.update({"u": (6, -1), "j": (6, 1)})

        # Add gripper keys if available
        gripper_keys = {}
        if self.config.gripper_action_key:
            gripper_keys = {"o": -1, "l": 1}

        # Generate action based on currently held keys (only process keys that are pressed, val=True)
        for key, is_pressed in list(self.current_pressed.items()):
            if not is_pressed:
                # Key was released, remove it from tracking
                del self.current_pressed[key]
                continue

            # Handle arm joint keys
            if key in key_mappings:
                joint_index, direction = key_mappings[key]
                if joint_index < len(self.config.arm_action_keys):
                    joint_key = self.config.arm_action_keys[joint_index]
                    self.curr_joint_actions[joint_key] += direction * self.config.action_increment
                    # Clamp arm joints to -100 to 100 (RANGE_M100_100 mode)
                    self.curr_joint_actions[joint_key] = max(-100.0, min(100.0, self.curr_joint_actions[joint_key]))

            # Handle gripper keys
            elif key in gripper_keys and self.config.gripper_action_key:
                direction = gripper_keys[key]
                # Use gripper_increment if available, otherwise fall back to action_increment
                gripper_inc = getattr(self.config, 'gripper_increment', self.config.action_increment)
                self.curr_joint_actions[self.config.gripper_action_key] += (
                    direction * gripper_inc
                )
                # Clamp gripper to 0.0 - 100.0 (RANGE_0_100 mode expects 0-100)
                self.curr_joint_actions[self.config.gripper_action_key] = max(
                    0.0, min(100.0, self.curr_joint_actions[self.config.gripper_action_key])
                )

            else:
                # If the key is pressed but doesn't belong to any action, add it to the misc_keys_queue
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        return self.curr_joint_actions


# Alias for hardware version - same implementation, just different default config
class KeyboardJointHardwareTeleop(KeyboardJointTeleop):
    """
    Teleop class for hardware robots with descriptive motor names.
    Uses the same implementation as KeyboardJointTeleop but with KeyboardJointHardwareTeleopConfig.
    """
    config_class = KeyboardJointHardwareTeleopConfig
    name = "keyboard_joint_hardware"
