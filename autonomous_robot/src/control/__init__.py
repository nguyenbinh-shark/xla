"""
Control Module for Autonomous Robot.
Contains motion controller.
"""

from .motion_controller import MotionController, MotionCommand

__all__ = [
    'MotionController', 'MotionCommand',
]
