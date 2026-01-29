"""
Base Mode - Abstract base class for all robot operation modes.
Defines common interface and shared functionality.
"""

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple, Any
import numpy as np

logger = logging.getLogger(__name__)


class ModeState(Enum):
    """States that any mode can be in."""
    IDLE = auto()           # Mode initialized but not running
    RUNNING = auto()        # Normal operation
    SEARCHING = auto()      # Lost target, searching
    PAUSED = auto()         # Temporarily paused
    ERROR = auto()          # Error state
    COMPLETED = auto()      # Task completed (for finite tasks)


@dataclass
class ModeOutput:
    """
    Standard output from any mode's process() method.
    Contains velocity commands and status information.
    """
    # Motion commands
    velocity: float = 0.0       # Linear velocity (m/s)
    yaw_rate: float = 0.0       # Angular velocity (rad/s)
    
    # State
    state: ModeState = ModeState.IDLE
    
    # Confidence/quality of the control decision (0-1)
    confidence: float = 0.0
    
    # Optional message for logging/debug
    message: str = ""
    
    # Optional visualization frame
    viz_frame: Optional[np.ndarray] = None
    
    # Should robot stop immediately?
    emergency_stop: bool = False


class BaseMode(ABC):
    """
    Abstract base class for all robot operation modes.
    
    Each mode implements:
    - process(): Main processing loop, returns ModeOutput
    - reset(): Reset mode state
    - get_name(): Return mode name
    """
    
    def __init__(self):
        """Initialize base mode."""
        self._state = ModeState.IDLE
        self._frame_count = 0
        self._enabled = False
        
        # Control parameters (can be overridden by subclasses)
        self.max_velocity = 2.0      # m/s
        self.min_velocity = 0.0      # m/s
        self.max_yaw_rate = 1.5      # rad/s
        
    @property
    def state(self) -> ModeState:
        """Get current mode state."""
        return self._state
    
    @property
    def is_enabled(self) -> bool:
        """Check if mode is enabled."""
        return self._enabled
    
    def enable(self) -> None:
        """Enable the mode."""
        self._enabled = True
        self._state = ModeState.RUNNING
        logger.info(f"{self.get_name()} enabled")
    
    def disable(self) -> None:
        """Disable the mode."""
        self._enabled = False
        self._state = ModeState.IDLE
        logger.info(f"{self.get_name()} disabled")
    
    def pause(self) -> None:
        """Pause the mode."""
        if self._state == ModeState.RUNNING:
            self._state = ModeState.PAUSED
            logger.info(f"{self.get_name()} paused")
    
    def resume(self) -> None:
        """Resume the mode from pause."""
        if self._state == ModeState.PAUSED:
            self._state = ModeState.RUNNING
            logger.info(f"{self.get_name()} resumed")
    
    @abstractmethod
    def process(
        self, 
        color_frame: np.ndarray,
        depth_frame: Optional[np.ndarray] = None,
        feedback: Optional[Any] = None
    ) -> ModeOutput:
        """
        Process one frame and return control output.
        
        Args:
            color_frame: BGR image from camera
            depth_frame: Depth image (optional)
            feedback: Odometry feedback from robot (optional)
            
        Returns:
            ModeOutput with velocity commands and status
        """
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """Reset mode to initial state."""
        pass
    
    @abstractmethod
    def get_name(self) -> str:
        """Get human-readable mode name."""
        pass
    
    def _clamp_velocity(self, v: float) -> float:
        """Clamp velocity to valid range."""
        return max(self.min_velocity, min(self.max_velocity, v))
    
    def _clamp_yaw_rate(self, yaw: float) -> float:
        """Clamp yaw rate to valid range."""
        return max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw))
    
    def _create_stop_output(self, message: str = "") -> ModeOutput:
        """Create a stop output (zero velocity)."""
        return ModeOutput(
            velocity=0.0,
            yaw_rate=0.0,
            state=self._state,
            confidence=0.0,
            message=message
        )
    
    def _create_emergency_stop(self, message: str = "") -> ModeOutput:
        """Create an emergency stop output."""
        return ModeOutput(
            velocity=0.0,
            yaw_rate=0.0,
            state=ModeState.ERROR,
            confidence=0.0,
            message=message,
            emergency_stop=True
        )
