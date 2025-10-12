import logging

import pygame
from lerobot.teleoperators.gamepad.gamepad_utils import InputController


class GamepadController6DOF(InputController):
    """Generate 6DOF motion deltas from gamepad input using pygame."""

    def __init__(self, x_step_size=1.0, y_step_size=1.0, z_step_size=1.0, rot_step_size=1.0, deadzone=0.1):
        super().__init__(x_step_size, y_step_size, z_step_size)
        self.rot_step_size = rot_step_size
        self.deadzone = deadzone
        self.joystick = None
        self.intervention_flag = False

    def start(self):
        """Initialize pygame and the gamepad."""
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            logging.error("No gamepad detected. Please connect a gamepad and try again.")
            self.running = False
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        logging.info(f"Initialized gamepad: {self.joystick.get_name()}")

        print("6DOF Gamepad controls:")
        print("  Left analog stick: Linear X/Y movement")
        print("  Right analog stick: Angular X/Y rotation (roll/pitch)")
        print("  LB/RB bumpers: Angular Z rotation (yaw)")
        print("  LT/RT triggers: Linear Z movement (up/down)")
        print("  A/X button: Gripper control (open by default, close when pressed)")

    def stop(self):
        """Clean up pygame resources."""
        import pygame

        if pygame.joystick.get_init():
            if self.joystick:
                self.joystick.quit()
            pygame.joystick.quit()
        pygame.quit()

    def update(self):
        """Process pygame events to get fresh gamepad readings."""
        import pygame

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                # Y button (3) for success
                if event.button == 3:
                    self.episode_end_status = "success"
                # B button (1) for exit
                elif event.button == 1:
                    self.running = False
                    self.episode_end_status = "quit"
                # A button (0) for gripper close
                elif event.button == 0:
                    self.close_gripper_command = True

            elif event.type == pygame.JOYBUTTONUP and event.button in [0, 1, 3]:
                if event.button == 0:
                    self.close_gripper_command = False
                if event.button != 1:  # Don't reset episode status for exit button
                    self.episode_end_status = None

    def get_6dof_deltas(self):
        """Get the current 6DOF movement deltas from gamepad state."""
        if not self.joystick:
            raise RuntimeError("Gamepad not initialized. Call start() first.")

        # Left stick: Linear X/Y movement (axes 0 and 1)
        left_x = self.joystick.get_axis(0)  # Left/Right
        left_y = self.joystick.get_axis(1)  # Up/Down (often inverted)

        # Right stick: Angular X/Y rotation (axes 2 and 3)
        right_x = self.joystick.get_axis(3)  # Roll
        right_y = self.joystick.get_axis(4)  # Pitch

        # Triggers: Linear Z movement (axes 4 and 5, or buttons)
        # LT (left trigger) - up movement
        # RT (right trigger) - down movement
        left_trigger = self.joystick.get_axis(2)  # LT
        right_trigger = self.joystick.get_axis(5)  # RT
        # Convert trigger values from [-1, 1] to [0, 1] range
        left_trigger = (left_trigger + 1) / 2
        right_trigger = (right_trigger + 1) / 2

        # Bumpers: Angular Z rotation (yaw) - buttons 4 and 5
        left_bumper = 1.0 if self.joystick.get_button(4) else 0.0  # LB
        right_bumper = 1.0 if self.joystick.get_button(5) else 0.0  # RB

        # Apply deadzone to avoid drift
        left_x = 0 if abs(left_x) < self.deadzone else left_x
        left_y = 0 if abs(left_y) < self.deadzone else left_y
        right_x = 0 if abs(right_x) < self.deadzone else right_x
        right_y = 0 if abs(right_y) < self.deadzone else right_y

        # Calculate deltas
        # Linear movement
        linear_x = -left_x * self.x_step_size  # Forward/backward
        linear_y = left_y * self.y_step_size  # Left/right
        linear_z = (right_trigger - left_trigger) * self.z_step_size  # Up/down

        # Angular movement
        angular_x = right_y * self.rot_step_size  # Roll
        angular_y = right_x * self.rot_step_size  # Pitch
        angular_z = (left_bumper - right_bumper) * self.rot_step_size  # Yaw

        return linear_x, linear_y, linear_z, angular_x, angular_y, angular_z

    def gripper_command(self):
        """Return the current gripper command - open by default, close when button pressed."""
        if self.close_gripper_command:
            return 1.0
        else:
            return 0.0  # Default to open
