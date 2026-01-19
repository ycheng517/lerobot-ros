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

import logging
import threading
import time
import asyncio
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from typing import Optional, List, Dict, Any

import rclpy
from control_msgs.action import GripperCommand
from lerobot.errors import DeviceNotConnectedError
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import Executor, SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


from .config_ros import ActionType, GripperActionType, ROS2InterfaceConfig
from .moveit_servo import MoveIt2Servo

logger = logging.getLogger(__name__)


class FastTrajectoryClient:
    """Optimized trajectory client for high-frequency non-blocking execution"""

    def __init__(self, node: Node, action_client: ActionClient, joint_names: List[str]):
        self.node = node
        self.action_client = action_client
        self.joint_names = joint_names

        # For 60 Hz execution
        self.target_period = 1.0 / 60.0  # 16.67 ms
        self.execution_queue = deque(maxlen=10)  # Buffer for commands
        self.last_goal_handle = None
        self.is_executing = False
        self.pending_futures = []

        # Performance monitoring
        self.timings = deque(maxlen=100)
        self.publish_times = deque(maxlen=100)

        # Thread pool for async processing
        self.thread_pool = ThreadPoolExecutor(max_workers=2)

        # QoS for trajectory messages
        self.trajectory_pub = node.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",  # Direct topic for faster publishing
            qos_profile=QoSProfile(
                depth=5,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            )
        )

    def send_position_non_blocking(self, positions: List[float], time_from_start: float = 0.1):
        """Send joint position as a single-point trajectory without blocking"""
        start_time = time.time()

        # Method 1: Direct publish to trajectory topic (fastest)
        try:
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
            point.velocities = [0.0] * len(positions)  # Optional: add velocities for smoother motion

            trajectory.points = [point]

            # Publish directly - this bypasses action server for speed
            self.trajectory_pub.publish(trajectory)

            elapsed = time.time() - start_time
            self.publish_times.append(elapsed)

            # Monitor performance
            if len(self.publish_times) % 50 == 0:
                avg_time = sum(self.publish_times) / len(self.publish_times)
                logger.debug(f"Direct trajectory publish avg: {avg_time*1000:.2f}ms")

            return True

        except Exception as e:
            logger.warning(f"Direct publish failed: {e}, falling back to action")

        # Method 2: Non-blocking action call (slower but more reliable)
        try:
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(self.target_period * 1e9)  # ~16.67ms for 60Hz

            goal_msg.trajectory.points = [point]

            # Send without waiting for result
            future = self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback
            )

            # Store future to prevent garbage collection
            self.pending_futures.append(future)
            # Clean up old futures
            self.pending_futures = [f for f in self.pending_futures if not f.done()]

            elapsed = time.time() - start_time
            self.timings.append(elapsed)

            # Monitor performance
            if len(self.timings) % 50 == 0:
                avg_time = sum(self.timings) / len(self.timings)
                current_hz = 1.0 / self.target_period if self.timings else 0
                logger.debug(f"Action send avg: {avg_time*1000:.2f}ms, Target Hz: {current_hz:.1f}")

            return True

        except Exception as e:
            logger.error(f"Non-blocking action send failed: {e}")
            return False

    def _feedback_callback(self, feedback_msg):
        """Handle feedback from trajectory execution"""
        # Optional: Process feedback if needed
        pass

    def cleanup(self):
        """Clean up resources"""
        if self.trajectory_pub:
            self.trajectory_pub.destroy()
        self.thread_pool.shutdown(wait=False)


class ROS2Interface:
    """Class to interface with a MoveIt2 manipulator.

    Optimized for 60 Hz operation with trajectory control.
    """

    def __init__(self, config: ROS2InterfaceConfig, action_type: ActionType):
        self.config = config
        self.action_type = action_type
        self.robot_node: Node | None = None
        self.pos_cmd_pub: Publisher | None = None
        self.traj_cmd_pub: Publisher | None = None
        self.gripper_action_client: ActionClient | None = None
        self.gripper_traj_pub: Publisher | None = None
        self.executor: Executor | None = None
        self.moveit2_servo: MoveIt2Servo | None = None
        self.executor_thread: threading.Thread | None = None
        self.is_connected = False
        self._last_joint_state: dict[str, dict[str, float]] | None = None

        # For 60 Hz operation
        self.fast_trajectory_client: Optional[FastTrajectoryClient] = None
        self.last_command_time = 0.0
        self.command_interval = 1.0 / 60.0  # 60 Hz = 16.67ms
        self.performance_stats = {
            "command_count": 0,
            "avg_latency": 0.0,
            "min_latency": float('inf'),
            "max_latency": 0.0
        }

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        # Use multi-threaded executor for better performance
        self.robot_node = Node(
            "moveit2_interface_node",
            namespace=self.config.namespace,
            parameter_overrides=[]
        )

        # **FIXED: Use standard QoS profiles that match robot_state_publisher**
        # Default QoS for command topics (fast)
        command_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,  # Changed from BEST_EFFORT
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(seconds=0, nanoseconds=int(1e9/60))
        )

        # **CRITICAL: Use SYSTEM_DEFAULT for joint_states to match robot_state_publisher**
        # Or use the exact QoS that robot_state_publisher uses
        joint_state_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,  # robot_state_publisher uses RELIABLE
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(seconds=0, nanoseconds=0)  # No deadline
        )

        if self.action_type == ActionType.JOINT_POSITION:
            self.pos_cmd_pub = self.robot_node.create_publisher(
                Float64MultiArray,
                "/position_controller/commands",
                qos_profile=command_qos
            )

        elif self.action_type == ActionType.JOINT_TRAJECTORY:
            # Create action client with optimized settings
            self.traj_action_client = ActionClient(
                self.robot_node,
                FollowJointTrajectory,
                "/arm_controller/follow_joint_trajectory",
                callback_group=ReentrantCallbackGroup(),
                # Use default QoS for action servers
                goal_service_qos_profile=QoSProfile(depth=10),
                result_service_qos_profile=QoSProfile(depth=10),
                feedback_sub_qos_profile=QoSProfile(depth=10),
                status_sub_qos_profile=QoSProfile(depth=10)
            )

            # Initialize fast trajectory client
            self.fast_trajectory_client = FastTrajectoryClient(
                self.robot_node,
                self.traj_action_client,
                self.config.arm_joint_names
            )

            # Also create direct publisher for even faster control
            self.traj_cmd_pub = self.robot_node.create_publisher(
                JointTrajectory,
                "/arm_controller/joint_trajectory",
                qos_profile=command_qos
            )

        elif self.action_type == ActionType.CARTESIAN_VELOCITY:
            self.moveit2_servo = MoveIt2Servo(
                node=self.robot_node,
                frame_id=self.config.base_link,
                callback_group=ReentrantCallbackGroup(),
            )

        # Gripper control - use trajectory for speed
        if self.config.gripper_action_type == GripperActionType.TRAJECTORY:
            self.gripper_traj_pub = self.robot_node.create_publisher(
                JointTrajectory,
                "/gripper_controller/joint_trajectory",
                qos_profile=command_qos
            )
        else:
            self.gripper_action_client = ActionClient(
                self.robot_node,
                GripperCommand,
                "/gripper_controller/gripper_cmd",
                callback_group=ReentrantCallbackGroup(),
                goal_service_qos_profile=QoSProfile(depth=10)
            )
            self._goal_msg = GripperCommand.Goal()

        # **FIXED: Joint state subscription with compatible QoS**
        self.joint_state_sub = self.robot_node.create_subscription(
            JointState,
            "joint_states",
            self._joint_state_callback,
            qos_profile=joint_state_qos  # Use compatible QoS
        )

        # Alternative: Use default QoS (even simpler)
        # self.joint_state_sub = self.robot_node.create_subscription(
        #     JointState,
        #     "joint_states",
        #     self._joint_state_callback,
        #     10  # Just queue size, uses default QoS
        # )

        # Create and start the executor with more threads
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.robot_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # Wait for connections with timeout
        self._wait_for_connections()
        time.sleep(1)  # Give some time to connect to services

        self.is_connected = True
        logger.info("ROS2Interface connected with 60Hz optimization")

    def _wait_for_connections(self):
        """Wait for required connections with timeout"""
        timeout = 5.0
        start_time = time.time()

        if self.action_type == ActionType.JOINT_TRAJECTORY:
            logger.info("Waiting for trajectory action server...")
            if self.traj_action_client.wait_for_server(timeout_sec=timeout):
                logger.info("Trajectory action server connected")
            else:
                logger.warning("Trajectory action server not available, using direct publishing")

        if self.config.gripper_action_type != GripperActionType.TRAJECTORY:
            if self.gripper_action_client:
                logger.info("Waiting for gripper action server...")
                if not self.gripper_action_client.wait_for_server(timeout_sec=2.0):
                    logger.warning("Gripper action server not available")

    def send_joint_position_command(self, joint_positions: list[float],
                                   unnormalize: bool = True,
                                   non_blocking: bool = True,
                                   time_from_start: float = 0.1) -> bool:
        """
        Send a command to the robot's joints optimized for 60 Hz.

        Args:
            joint_positions: Target positions for joints
            unnormalize: Whether to unnormalize positions
            non_blocking: If True, doesn't wait for execution (FAST, for 60Hz)
            time_from_start: Time to reach position (for trajectories)

        Returns:
            bool: True if command was sent successfully
        """
        if not self.robot_node:
            raise DeviceNotConnectedError("ROS2Interface is not connected.")

        # Rate limiting for 60 Hz
        current_time = time.time()
        time_since_last = current_time - self.last_command_time
        if time_since_last < self.command_interval:
            # Skip if sending too fast
            return False

        self.last_command_time = current_time

        # Start timing
        start_time = time.time()

        if unnormalize:
            if self.config.min_joint_positions is None or self.config.max_joint_positions is None:
                raise ValueError("Joint position normalization requires min and max.")
            joint_positions = [
                min(max(pos, min_pos), max_pos)
                for pos, min_pos, max_pos in zip(
                    joint_positions,
                    self.config.min_joint_positions,
                    self.config.max_joint_positions,
                    strict=True,
                )
            ]

        if len(joint_positions) != len(self.config.arm_joint_names):
            raise ValueError(
                f"Expected {len(self.config.arm_joint_names)} joint positions, got {len(joint_positions)}."
            )

        if self.action_type == ActionType.JOINT_TRAJECTORY:
            if non_blocking:
                # FAST PATH: Non-blocking trajectory for 60 Hz
                return self._send_trajectory_fast(joint_positions, time_from_start)
            else:
                # SLOW PATH: Blocking trajectory with feedback
                return self._send_trajectory_blocking(joint_positions, time_from_start)

        else:  # JOINT_POSITION
            if self.pos_cmd_pub is None:
                raise DeviceNotConnectedError("Position command publisher not initialized.")

            msg = Float64MultiArray()
            msg.data = joint_positions
            self.pos_cmd_pub.publish(msg)

            # Update performance stats
            self._update_performance_stats(time.time() - start_time)
            return True

    def _send_trajectory_fast(self, joint_positions: list[float],
                             time_from_start: float = 0.1) -> bool:
        """
        Send trajectory command using fastest available method.
        """
        start_time = time.time()
        success = False

        # Method 1: Direct publish to trajectory topic (fastest, ~0.1ms)
        if self.traj_cmd_pub:
            try:
                trajectory = JointTrajectory()
                trajectory.joint_names = self.config.arm_joint_names

                point = JointTrajectoryPoint()
                point.positions = joint_positions
                point.time_from_start.sec = int(time_from_start)
                point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)

                # Add velocities for smoother motion (optional)
                if self._last_joint_state and 'velocity' in self._last_joint_state:
                    current_velocities = [self._last_joint_state['velocity'].get(name, 0.0)
                                         for name in self.config.arm_joint_names]
                    point.velocities = current_velocities

                trajectory.points = [point]
                self.traj_cmd_pub.publish(trajectory)
                success = True

            except Exception as e:
                logger.debug(f"Direct trajectory publish failed: {e}")

        # Method 2: Use fast trajectory client
        if not success and self.fast_trajectory_client:
            try:
                success = self.fast_trajectory_client.send_position_non_blocking(
                    joint_positions,
                    time_from_start
                )
            except Exception as e:
                logger.debug(f"Fast trajectory client failed: {e}")

        # Method 3: Non-blocking action call (fallback)
        if not success and self.traj_action_client:
            try:
                goal_msg = FollowJointTrajectory.Goal()
                goal_msg.trajectory.joint_names = self.config.arm_joint_names

                point = JointTrajectoryPoint()
                point.positions = joint_positions
                point.time_from_start.sec = 0
                point.time_from_start.nanosec = int(self.command_interval * 1e9)  # 16.67ms

                goal_msg.trajectory.points = [point]

                # Send without waiting
                future = self.traj_action_client.send_goal_async(goal_msg)
                success = True

            except Exception as e:
                logger.error(f"All trajectory methods failed: {e}")
                success = False

        # Update performance stats
        elapsed = time.time() - start_time
        self._update_performance_stats(elapsed)

        return success

    def _send_trajectory_blocking(self, joint_positions: list[float],
                                 time_from_start: float = 1.0) -> bool:
        """
        Send trajectory command and wait for result (slower, not for 60Hz).
        """
        if not self.traj_action_client:
            return False

        if not self.traj_action_client.wait_for_server(timeout_sec=0.5):
            logger.warning("Trajectory action server not available")
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.config.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
        goal_msg.trajectory.points.append(point)

        # Send and wait (blocking)
        send_goal_future = self.traj_action_client.send_goal_async(goal_msg)

        # Use a short timeout to prevent blocking too long
        try:
            rclpy.spin_until_future_complete(self.robot_node, send_goal_future, timeout_sec=0.1)
            if send_goal_future.done():
                goal_handle = send_goal_future.result()
                return goal_handle.accepted if goal_handle else False
        except Exception as e:
            logger.debug(f"Trajectory send interrupted: {e}")

        return False

    def _update_performance_stats(self, latency: float):
        """Update performance monitoring statistics"""
        self.performance_stats["command_count"] += 1
        self.performance_stats["avg_latency"] = (
            (self.performance_stats["avg_latency"] * (self.performance_stats["command_count"] - 1) + latency)
            / self.performance_stats["command_count"]
        )
        self.performance_stats["min_latency"] = min(self.performance_stats["min_latency"], latency)
        self.performance_stats["max_latency"] = max(self.performance_stats["max_latency"], latency)

        # Log performance periodically
        if self.performance_stats["command_count"] % 100 == 0:
            current_hz = 1.0 / self.command_interval
            achieved_hz = 1.0 / self.performance_stats["avg_latency"] if self.performance_stats["avg_latency"] > 0 else 0
            logger.info(
                f"Performance: Commands={self.performance_stats['command_count']}, "
                f"Latency: avg={self.performance_stats['avg_latency']*1000:.2f}ms, "
                f"min={self.performance_stats['min_latency']*1000:.2f}ms, "
                f"max={self.performance_stats['max_latency']*1000:.2f}ms, "
                f"Target={current_hz:.1f}Hz, Achieved={achieved_hz:.1f}Hz"
            )

    def servo(self, linear, angular, normalize: bool = True) -> None:
        if not self.moveit2_servo:
            raise DeviceNotConnectedError("MoveIt2Servo not available.")

        if normalize:
            linear = [v * self.config.max_linear_velocity for v in linear]
            angular = [v * self.config.max_angular_velocity for v in angular]

        # Use non-blocking servo if available
        self.moveit2_servo.servo(linear=linear, angular=angular)

    def send_gripper_command(self, position: float, unnormalize: bool = True) -> bool:
        """
        Send gripper command optimized for speed.
        """
        if not self.robot_node:
            raise DeviceNotConnectedError("ROS2Interface not connected.")

        if unnormalize:
            open_pos = self.config.gripper_open_position
            closed_pos = self.config.gripper_close_position
            gripper_goal = open_pos + position * (closed_pos - open_pos)
        else:
            gripper_goal = position

        if self.config.gripper_action_type == GripperActionType.TRAJECTORY:
            if self.gripper_traj_pub is None:
                raise DeviceNotConnectedError("Gripper publisher not initialized.")

            # Fast publish for gripper
            msg = JointTrajectory()
            msg.joint_names = [self.config.gripper_joint_name]
            point = JointTrajectoryPoint()
            point.positions = [float(gripper_goal)]
            msg.points = [point]
            self.gripper_traj_pub.publish(msg)
            return True

        else:
            if not self.gripper_action_client:
                raise DeviceNotConnectedError("Gripper action client not initialized.")

            # Non-blocking gripper action
            self._goal_msg.command.position = float(gripper_goal)
            future = self.gripper_action_client.send_goal_async(self._goal_msg)
            return future is not None

    @property
    def joint_state(self) -> dict[str, dict[str, float]] | None:
        return self._last_joint_state

    def _joint_state_callback(self, msg: "JointState") -> None:
        self._last_joint_state = self._last_joint_state or {}
        positions = {}
        velocities = {}
        name_to_index = {name: i for i, name in enumerate(msg.name)}

        for joint_name in self.config.arm_joint_names:
            idx = name_to_index.get(joint_name)
            if idx is None:
                continue  # Don't raise error, just skip
            positions[joint_name] = msg.position[idx]
            velocities[joint_name] = msg.velocity[idx]

        if self.config.gripper_joint_name:
            idx = name_to_index.get(self.config.gripper_joint_name)
            if idx is not None:
                positions[self.config.gripper_joint_name] = msg.position[idx]
                velocities[self.config.gripper_joint_name] = msg.velocity[idx]

        self._last_joint_state["position"] = positions
        self._last_joint_state["velocity"] = velocities

    def disconnect(self):
        # Clean up fast trajectory client
        if self.fast_trajectory_client:
            self.fast_trajectory_client.cleanup()
            self.fast_trajectory_client = None

        # Clean up ROS2 resources
        if hasattr(self, 'joint_state_sub') and self.joint_state_sub:
            self.joint_state_sub.destroy()
        if self.pos_cmd_pub:
            self.pos_cmd_pub.destroy()
        if hasattr(self, "traj_action_client") and self.traj_action_client:
            self.traj_action_client.destroy()
        if self.traj_cmd_pub:
            self.traj_cmd_pub.destroy()
        if self.gripper_action_client:
            self.gripper_action_client.destroy()
        if self.gripper_traj_pub:
            self.gripper_traj_pub.destroy()
        if self.robot_node:
            self.robot_node.destroy_node()
            self.robot_node = None
        if self.moveit2_servo:
            self.moveit2_servo = None

        if self.executor:
            self.executor.shutdown()
            self.executor = None
        if self.executor_thread:
            self.executor_thread.join(timeout=1.0)
            self.executor_thread = None

        self.is_connected = False
        logger.info("ROS2Interface disconnected")


# Example usage for 60 Hz control loop:
def example_60hz_control():
    """Example of how to use the interface for 60 Hz control"""
    import numpy as np

    # Create interface
    interface = ROS2Interface(config, ActionType.JOINT_TRAJECTORY)
    interface.connect()

    # 60 Hz control loop
    target_hz = 60.0
    period = 1.0 / target_hz
    next_time = time.time()

    try:
        while True:
            start_time = time.time()

            # 1. Get current state
            joint_state = interface.joint_state

            # 2. Compute next command (your control logic here)
            # Example: sine wave motion for testing
            t = time.time()
            joint_positions = [np.sin(t + i) * 0.5 for i in range(6)]

            # 3. Send command NON-BLOCKING for 60 Hz
            success = interface.send_joint_position_command(
                joint_positions,
                non_blocking=True,  # CRITICAL for 60 Hz
                time_from_start=period  # Match control rate
            )

            if not success:
                logger.warning("Command send failed or rate limited")

            # 4. Maintain 60 Hz timing
            next_time += period
            sleep_time = max(0, next_time - time.time())

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # We're falling behind
                logger.warning(f"Control loop behind by {-sleep_time*1000:.1f}ms")
                next_time = time.time() + period

    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        interface.disconnect()
