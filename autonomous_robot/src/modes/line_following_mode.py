"""
Line Following Mode - Follow a line on the ground.
Uses camera to detect line and calculates steering commands.
"""

import cv2
import numpy as np
import logging
from typing import Optional, Any
from dataclasses import dataclass

from .base_mode import BaseMode, ModeOutput, ModeState
from src.perception import SimpleLineDetector, LineDetectionResult

logger = logging.getLogger(__name__)


@dataclass
class LineFollowingConfig:
    """Configuration for line following mode."""
    # Speed control
    base_speed: float = 1.2          # Base forward speed (m/s)
    max_speed: float = 2.0           # Maximum speed when line is straight
    min_speed: float = 0.5           # Minimum speed during sharp turns
    
    # Steering control
    steering_gain: float = 3.0       # Position error -> yaw rate (rad/s per unit)
    heading_gain: float = 0.8        # Heading error -> yaw rate
    
    # Search behavior
    search_yaw_rate: float = 0.6     # Yaw rate when searching (rad/s)
    max_frames_lost: int = 30        # Stop after this many frames without line
    
    # Velocity modulation based on curvature
    curvature_slowdown: float = 0.8  # How much to slow down in curves (0-1)


class LineFollowingMode(BaseMode):
    """
    Line following mode using camera and centerline detection.
    
    Pipeline:
        Camera Frame → Line Detection → Error Calculation → Control Output
    
    Features:
    - Position and heading error calculation
    - Adaptive speed control (slow in curves)
    - Line search/recovery when lost
    - Smooth control transitions
    """
    
    def __init__(self, config: Optional[LineFollowingConfig] = None):
        """
        Initialize line following mode.
        
        Args:
            config: Configuration parameters (uses defaults if None)
        """
        super().__init__()
        
        self.config = config or LineFollowingConfig()
        self.line_detector = SimpleLineDetector()
        
        # State tracking
        self._frames_lost = 0
        self._last_line_position = 0.0
        self._search_direction = 0
        
        # Statistics
        self._total_frames = 0
        self._line_detected_frames = 0
        
        logger.info("LineFollowingMode initialized")
    
    def get_name(self) -> str:
        """Get mode name."""
        return "Line Following"
    
    def reset(self) -> None:
        """Reset mode state."""
        self._state = ModeState.IDLE
        self._frames_lost = 0
        self._last_line_position = 0.0
        self._search_direction = 0
        self._total_frames = 0
        self._line_detected_frames = 0
        self.line_detector.reset()
        logger.info("LineFollowingMode reset")
    
    def process(
        self,
        color_frame: np.ndarray,
        depth_frame: Optional[np.ndarray] = None,
        feedback: Optional[Any] = None
    ) -> ModeOutput:
        """
        Process frame and compute control commands.
        
        Args:
            color_frame: BGR image from camera
            depth_frame: Not used in this mode
            feedback: Robot feedback (velocity, position, yaw)
            
        Returns:
            ModeOutput with velocity and yaw_rate commands
        """
        if not self._enabled:
            return self._create_stop_output("Mode not enabled")
        
        if color_frame is None or color_frame.size == 0:
            return self._create_stop_output("Invalid frame")
        
        self._total_frames += 1
        self._frame_count += 1
        
        # Detect line
        result = self.line_detector.detect(color_frame)
        
        # Create visualization
        viz_frame = self.line_detector.visualize(color_frame, result)
        
        # Process detection result
        if result.line_detected:
            output = self._process_line_detected(result, feedback)
        else:
            output = self._process_line_lost(result)
        
        # Add visualization
        output.viz_frame = self._add_status_overlay(viz_frame, output, result)
        
        return output
    
    def _process_line_detected(
        self, 
        result: LineDetectionResult,
        feedback: Optional[Any] = None
    ) -> ModeOutput:
        """
        Process when line is detected.
        Calculate velocity and yaw_rate based on errors.
        """
        self._line_detected_frames += 1
        self._frames_lost = 0
        self._state = ModeState.RUNNING
        
        # Save last position for recovery
        self._last_line_position = result.position_error
        
        # Calculate yaw rate from errors
        yaw_rate = self._calculate_yaw_rate(result)
        
        # Calculate velocity (adaptive based on curvature)
        velocity = self._calculate_velocity(result, feedback)
        
        return ModeOutput(
            velocity=velocity,
            yaw_rate=yaw_rate,
            state=ModeState.RUNNING,
            confidence=result.confidence,
            message=f"Following: err={result.position_error:.2f}, heading={result.heading_error_degrees:.1f}°"
        )
    
    def _process_line_lost(self, result: LineDetectionResult) -> ModeOutput:
        """
        Process when line is lost.
        Enter search mode or stop.
        """
        self._frames_lost += 1
        
        # Check if we should stop
        if self._frames_lost > self.config.max_frames_lost:
            self._state = ModeState.ERROR
            return ModeOutput(
                velocity=0.0,
                yaw_rate=0.0,
                state=ModeState.ERROR,
                confidence=0.0,
                message=f"Line lost for {self._frames_lost} frames - STOPPED"
            )
        
        # Search mode - rotate in place
        self._state = ModeState.SEARCHING
        
        # Determine search direction based on last known position
        if self._frames_lost == 1:
            if self._last_line_position > 0.1:
                self._search_direction = 1  # Line was on right
            elif self._last_line_position < -0.1:
                self._search_direction = -1  # Line was on left
            else:
                self._search_direction = 1  # Default: search right
        
        # Search: rotate in place (velocity = 0)
        search_yaw = self._search_direction * self.config.search_yaw_rate
        
        return ModeOutput(
            velocity=0.0,  # Stop moving forward
            yaw_rate=search_yaw,
            state=ModeState.SEARCHING,
            confidence=0.0,
            message=f"Searching: direction={'right' if self._search_direction > 0 else 'left'}, frames={self._frames_lost}"
        )
    
    def _calculate_yaw_rate(self, result: LineDetectionResult) -> float:
        """
        Calculate yaw rate from position and heading errors.
        
        Combines:
        - Position error: How far line is from center
        - Heading error: Angle of line vs robot heading
        """
        # Position error contribution (main steering)
        pos_contribution = self.config.steering_gain * result.position_error
        
        # Heading error contribution (anticipatory steering)
        heading_contribution = self.config.heading_gain * result.heading_error
        
        # Combine
        yaw_rate = pos_contribution + heading_contribution
        
        # Clamp to valid range
        return self._clamp_yaw_rate(yaw_rate)
    
    def _calculate_velocity(
        self, 
        result: LineDetectionResult,
        feedback: Optional[Any] = None
    ) -> float:
        """
        Calculate forward velocity based on line curvature and errors.
        Slow down in curves, speed up on straight sections.
        """
        # Error magnitude (higher error = more curve = slower)
        error_magnitude = abs(result.position_error) + abs(result.heading_error) / 1.57
        
        # Speed factor: 1.0 for straight, lower for curves
        # Keep at least (1 - curvature_slowdown) speed
        min_factor = 1.0 - self.config.curvature_slowdown
        speed_factor = max(min_factor, 1.0 - error_magnitude)
        
        # Scale by confidence
        speed_factor *= result.confidence
        
        # Calculate velocity
        velocity = self.config.min_speed + \
                   (self.config.max_speed - self.config.min_speed) * speed_factor
        
        # Apply closed-loop correction if feedback available
        if feedback is not None and hasattr(feedback, 'velocity'):
            # Simple P control to match target velocity
            v_error = velocity - feedback.velocity
            velocity += 0.3 * v_error
        
        return self._clamp_velocity(velocity)
    
    def _add_status_overlay(
        self, 
        frame: np.ndarray, 
        output: ModeOutput,
        result: LineDetectionResult
    ) -> np.ndarray:
        """Add status information overlay to visualization frame."""
        if frame is None:
            return None
        
        h, w = frame.shape[:2]
        
        # Status bar at bottom
        status_bar_height = 40
        frame_with_bar = cv2.copyMakeBorder(
            frame, 0, status_bar_height, 0, 0,
            cv2.BORDER_CONSTANT, value=(40, 40, 40)
        )
        
        # Mode and state
        state_color = {
            ModeState.RUNNING: (0, 255, 0),
            ModeState.SEARCHING: (0, 255, 255),
            ModeState.ERROR: (0, 0, 255),
            ModeState.PAUSED: (255, 255, 0),
        }.get(output.state, (255, 255, 255))
        
        cv2.putText(
            frame_with_bar, 
            f"LINE FOLLOW | {output.state.name}", 
            (10, h + 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, state_color, 1
        )
        
        # Control commands
        cv2.putText(
            frame_with_bar,
            f"V:{output.velocity:.2f} Y:{output.yaw_rate:+.2f}",
            (250, h + 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )
        
        # Confidence
        cv2.putText(
            frame_with_bar,
            f"Conf:{output.confidence:.0%}",
            (450, h + 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )
        
        # Detection rate
        if self._total_frames > 0:
            rate = self._line_detected_frames / self._total_frames
            cv2.putText(
                frame_with_bar,
                f"Det:{rate:.0%}",
                (550, h + 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1
            )
        
        return frame_with_bar
    
    def get_statistics(self) -> dict:
        """Get mode statistics."""
        return {
            'total_frames': self._total_frames,
            'line_detected_frames': self._line_detected_frames,
            'detection_rate': self._line_detected_frames / max(1, self._total_frames),
            'current_state': self._state.name,
            'frames_lost': self._frames_lost
        }
    
    def set_speeds(
        self, 
        base_speed: Optional[float] = None,
        max_speed: Optional[float] = None,
        min_speed: Optional[float] = None
    ) -> None:
        """Update speed parameters at runtime."""
        if base_speed is not None:
            self.config.base_speed = base_speed
        if max_speed is not None:
            self.config.max_speed = max_speed
            self.max_velocity = max_speed
        if min_speed is not None:
            self.config.min_speed = min_speed
            self.min_velocity = min_speed
        logger.info(f"Speeds updated: base={self.config.base_speed}, "
                   f"max={self.config.max_speed}, min={self.config.min_speed}")
    
    def set_steering_gains(
        self,
        steering_gain: Optional[float] = None,
        heading_gain: Optional[float] = None
    ) -> None:
        """Update steering gain parameters at runtime."""
        if steering_gain is not None:
            self.config.steering_gain = steering_gain
        if heading_gain is not None:
            self.config.heading_gain = heading_gain
        logger.info(f"Steering gains updated: steering={self.config.steering_gain}, "
                   f"heading={self.config.heading_gain}")
