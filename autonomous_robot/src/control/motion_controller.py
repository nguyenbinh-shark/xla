"""
Motion Controller Module.
Simple controller for direct velocity and yaw rate commands.
"""

import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class MotionCommand:
    """Represents a motion command for the robot."""
    velocity: float  # Linear velocity in m/s
    yaw_rate: float  # Angular velocity in rad/s (steering)
    is_emergency_stop: bool  # Emergency stop flag


class MotionController:
    """
    Simplified motion controller for the autonomous robot.
    
    Directly passes through velocity and yaw_rate commands
    without PID control or complex processing.
    """

    def __init__(self):
        """Initialize the motion controller."""
        logger.info("Motion controller initialized (direct control mode)")

    def create_motion_command(
        self,
        velocity: float,
        yaw_rate: float,
        is_emergency_stop: bool = False
    ) -> MotionCommand:
        """
        Create motion command directly from input parameters.

        Args:
            velocity: Desired linear velocity (m/s)
            yaw_rate: Desired angular velocity (rad/s) - positive = turn left
            is_emergency_stop: Emergency stop flag

        Returns:
            MotionCommand with specified values
        """
        if is_emergency_stop:
            return self._emergency_stop()
        
        return MotionCommand(
            velocity=velocity,
            yaw_rate=yaw_rate,
            is_emergency_stop=False
        )

    def _emergency_stop(self) -> MotionCommand:
        """
        Generate emergency stop command.

        Returns:
            MotionCommand for emergency stop
        """
        return MotionCommand(
            velocity=0.0,
            yaw_rate=0.0,
            is_emergency_stop=True
        )

    def reset(self) -> None:
        """Reset controller state (minimal in direct mode)."""
        logger.info("Motion controller reset")

    def get_controller_info(self) -> dict:
        """Get current controller state for debugging."""
        return {
            "mode": "direct_control",
            "status": "ready"
        }
