from lerobot import replay as lr_rec
from lerobot.robots import Robot, RobotConfig

from lerobot_robot_ros import ROS2Config, ROS2Robot

# Override the default robot creation functions to use ROS2-specific
#  implementations. This allows us to create our own robots.
orig_make_robot_from_config = lr_rec.make_robot_from_config


def make_my_robot_from_config(config: RobotConfig) -> Robot:
    """Create a robot instance based on the provided configuration."""
    if not isinstance(config, ROS2Config):
        return orig_make_robot_from_config(config)

    return ROS2Robot(config)


# Replace the default creation function with our ROS2-specific ones
lr_rec.make_robot_from_config = make_my_robot_from_config


if __name__ == "__main__":
    lr_rec.replay()  # type: ignore
