"""
Patrol Mode - Autonomous patrol with AI intruder detection.
Robot patrols along waypoints and alerts when detecting people.
"""

import cv2
import numpy as np
import logging
import time
from typing import Optional, Any, List, Tuple
from dataclasses import dataclass, field
from enum import Enum, auto

from .base_mode import BaseMode, ModeOutput, ModeState
from src.perception import ObjectDetector, DepthEstimator

logger = logging.getLogger(__name__)


class PatrolState(Enum):
    """Sub-states for patrol mode."""
    PATROLLING = auto()      # Normal patrol movement
    ROTATING = auto()        # Rotating to next direction
    ALERT = auto()           # Intruder detected - alerting
    TRACKING = auto()        # Following detected intruder
    RETURNING = auto()       # Returning to patrol route


@dataclass
class PatrolConfig:
    """Configuration for patrol mode."""
    # Patrol movement
    patrol_velocity: float = 0.3        # Forward speed while patrolling (m/s)
    rotate_yaw_rate: float = 0.5        # Yaw rate when rotating (rad/s)
    
    # Waypoint timing (simple timed patrol)
    patrol_forward_time: float = 5.0    # Seconds to move forward
    patrol_rotate_time: float = 3.0     # Seconds to rotate
    
    # Detection settings
    detect_class: str = "person"        # Class to detect as intruder
    min_confidence: float = 0.5         # Minimum detection confidence
    min_box_area: int = 3000            # Minimum box area (pixels^2)
    
    # Alert settings
    alert_distance: float = 3.0         # Maximum distance to trigger alert (m)
    alert_duration: float = 3.0         # How long to stay in alert state (s)
    
    # Tracking settings (optional follow intruder)
    track_intruder: bool = True         # Follow intruder when detected
    tracking_distance: float = 2.0      # Distance to maintain from intruder (m)
    tracking_gain: float = 1.5          # Steering gain when tracking
    max_track_time: float = 10.0        # Maximum time to track before returning
    
    # Sound alert (placeholder - needs actual implementation)
    enable_sound_alert: bool = True
    
    # Visual alert
    alert_color: Tuple[int, int, int] = (0, 0, 255)  # Red for alert
    normal_color: Tuple[int, int, int] = (0, 255, 0)  # Green for normal


@dataclass
class Intruder:
    """Detected intruder information."""
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    center_x: float                   # Normalized center x (-1 to 1)
    center_y: float                   # Normalized center y (-1 to 1)
    distance: float                   # Distance in meters
    confidence: float                 # Detection confidence
    timestamp: float                  # Detection time


class PatrolMode(BaseMode):
    """
    Patrol mode - autonomous patrol with intruder detection.
    
    Behavior:
    1. Patrol: Move forward for X seconds, rotate, repeat
    2. Detect: Continuously scan for people using YOLO
    3. Alert: When person detected, trigger alert
    4. Track (optional): Follow the intruder while alerting
    5. Return: After alert, return to patrol
    
    Features:
    - Timed waypoint patrol (no odometry needed)
    - Real-time person detection with YOLO
    - Distance-based alert threshold
    - Optional intruder tracking
    - Visual overlay with detection status
    """
    
    def __init__(self, config: Optional[PatrolConfig] = None):
        """
        Initialize patrol mode.
        
        Args:
            config: Patrol configuration parameters
        """
        super().__init__()
        
        self.config = config or PatrolConfig()
        
        # Initialize perception
        self.object_detector = ObjectDetector()
        self.depth_estimator = DepthEstimator()
        
        # Patrol state
        self._patrol_state = PatrolState.PATROLLING
        self._state_start_time = time.time()
        self._patrol_cycle = 0
        
        # Detection state
        self._current_intruder: Optional[Intruder] = None
        self._intruders_detected: List[Intruder] = []
        self._alert_start_time: Optional[float] = None
        self._track_start_time: Optional[float] = None
        
        # Callback for alerts (can be set by user)
        self._alert_callback = None
        
        logger.info("PatrolMode initialized")
    
    def get_name(self) -> str:
        """Get mode name."""
        return "Patrol"
    
    def reset(self) -> None:
        """Reset mode state."""
        self._state = ModeState.IDLE
        self._patrol_state = PatrolState.PATROLLING
        self._state_start_time = time.time()
        self._patrol_cycle = 0
        self._current_intruder = None
        self._intruders_detected = []
        self._alert_start_time = None
        self._track_start_time = None
        logger.info("PatrolMode reset")
    
    def set_alert_callback(self, callback) -> None:
        """
        Set callback function for alerts.
        
        Args:
            callback: Function(intruder: Intruder) -> None
        """
        self._alert_callback = callback
    
    def get_intruder_history(self) -> List[Intruder]:
        """Get list of all detected intruders."""
        return self._intruders_detected.copy()
    
    def process(
        self,
        color_frame: np.ndarray,
        depth_frame: Optional[np.ndarray] = None,
        feedback: Optional[Any] = None
    ) -> ModeOutput:
        """
        Process frame and compute patrol/tracking commands.
        
        Args:
            color_frame: BGR image from camera
            depth_frame: Depth image from camera (optional)
            feedback: Robot feedback (unused currently)
            
        Returns:
            ModeOutput with velocity and yaw_rate commands
        """
        if not self._enabled:
            return self._create_stop_output("Mode not enabled")
        
        if color_frame is None or color_frame.size == 0:
            return self._create_stop_output("Invalid frame")
        
        self._frame_count += 1
        current_time = time.time()
        
        # Always run detection
        intruder = self._detect_intruder(color_frame, depth_frame)
        
        # State machine
        if intruder is not None:
            # Intruder detected!
            if self._patrol_state != PatrolState.ALERT and \
               self._patrol_state != PatrolState.TRACKING:
                self._trigger_alert(intruder)
        
        # Process based on current state
        if self._patrol_state == PatrolState.ALERT:
            output = self._process_alert(intruder, current_time)
        elif self._patrol_state == PatrolState.TRACKING:
            output = self._process_tracking(intruder, current_time)
        elif self._patrol_state == PatrolState.RETURNING:
            output = self._process_returning(current_time)
        elif self._patrol_state == PatrolState.ROTATING:
            output = self._process_rotating(current_time)
        else:  # PATROLLING
            output = self._process_patrolling(current_time)
        
        # Add visualization
        output.viz_frame = self._create_visualization(
            color_frame, intruder, output
        )
        
        return output
    
    def _detect_intruder(
        self,
        color_frame: np.ndarray,
        depth_frame: Optional[np.ndarray]
    ) -> Optional[Intruder]:
        """
        Detect intruder in frame.
        
        Returns:
            Intruder object if detected, None otherwise
        """
        # Run object detection
        result = self.object_detector.detect(color_frame, depth_frame)
        
        if result is None or result.objects is None:
            return None
        
        frame_h, frame_w = color_frame.shape[:2]
        best_intruder = None
        best_score = 0
        
        for det in result.objects:
            # Check class
            if det.class_name != self.config.detect_class:
                continue
            
            # Check confidence
            if det.confidence < self.config.min_confidence:
                continue
            
            # Get bbox - DetectedObject uses bbox tuple (x1, y1, x2, y2)
            x1, y1, x2, y2 = det.bbox
            
            # Check box area
            box_area = (x2 - x1) * (y2 - y1)
            if box_area < self.config.min_box_area:
                continue
            
            # Get distance - DetectedObject uses 'depth' attribute
            distance = det.depth if det.depth > 0 else float('inf')
            
            # Check distance threshold
            if distance > self.config.alert_distance:
                continue
            
            # Calculate score (closer + more confident = better)
            score = det.confidence * (1.0 / (distance + 0.1))
            
            if score > best_score:
                best_score = score
                
                # Calculate normalized center
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                norm_x = (center_x - frame_w / 2) / (frame_w / 2)  # -1 to 1
                norm_y = (center_y - frame_h / 2) / (frame_h / 2)  # -1 to 1
                
                best_intruder = Intruder(
                    bbox=(x1, y1, x2, y2),
                    center_x=norm_x,
                    center_y=norm_y,
                    distance=distance,
                    confidence=det.confidence,
                    timestamp=time.time()
                )
        
        return best_intruder
    
    def _trigger_alert(self, intruder: Intruder) -> None:
        """Trigger alert when intruder detected."""
        logger.warning(f"🚨 INTRUDER DETECTED! Distance: {intruder.distance:.1f}m")
        
        self._current_intruder = intruder
        self._intruders_detected.append(intruder)
        self._patrol_state = PatrolState.ALERT
        self._alert_start_time = time.time()
        
        # Call alert callback if set
        if self._alert_callback:
            try:
                self._alert_callback(intruder)
            except Exception as e:
                logger.error(f"Alert callback error: {e}")
        
        # Sound alert (placeholder)
        if self.config.enable_sound_alert:
            self._play_alert_sound()
    
    def _play_alert_sound(self) -> None:
        """Play alert sound (placeholder - implement with actual audio)."""
        # Could use: pygame, playsound, or system beep
        try:
            print('\a')  # Terminal bell
        except:
            pass
    
    def _process_alert(
        self,
        intruder: Optional[Intruder],
        current_time: float
    ) -> ModeOutput:
        """Process alert state - stop and alert."""
        elapsed = current_time - self._alert_start_time
        
        # Check if should transition to tracking
        if self.config.track_intruder and intruder is not None:
            if elapsed > 1.0:  # Brief pause before tracking
                self._patrol_state = PatrolState.TRACKING
                self._track_start_time = current_time
                return self._process_tracking(intruder, current_time)
        
        # Check if alert duration expired
        if elapsed > self.config.alert_duration:
            if intruder is None:
                # Lost intruder, return to patrol
                self._patrol_state = PatrolState.RETURNING
                self._state_start_time = current_time
            elif self.config.track_intruder:
                # Still see intruder, start tracking
                self._patrol_state = PatrolState.TRACKING
                self._track_start_time = current_time
        
        # Stay stopped during alert
        return ModeOutput(
            velocity=0.0,
            yaw_rate=0.0,
            state=ModeState.RUNNING,
            confidence=1.0,
            message=f"🚨 ALERT! Intruder at {intruder.distance:.1f}m" if intruder else "🚨 ALERT!"
        )
    
    def _process_tracking(
        self,
        intruder: Optional[Intruder],
        current_time: float
    ) -> ModeOutput:
        """Process tracking state - follow intruder."""
        # Check tracking timeout
        elapsed = current_time - self._track_start_time
        if elapsed > self.config.max_track_time:
            logger.info("Tracking timeout, returning to patrol")
            self._patrol_state = PatrolState.RETURNING
            self._state_start_time = current_time
            return self._create_stop_output("Tracking timeout")
        
        # Lost intruder
        if intruder is None:
            logger.info("Lost intruder, returning to patrol")
            self._patrol_state = PatrolState.RETURNING
            self._state_start_time = current_time
            return self._create_stop_output("Intruder lost")
        
        self._current_intruder = intruder
        
        # Calculate steering to center intruder
        yaw_rate = -intruder.center_x * self.config.tracking_gain
        yaw_rate = np.clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)
        
        # Calculate velocity based on distance
        distance_error = intruder.distance - self.config.tracking_distance
        
        if distance_error > 0.3:
            # Too far, move closer
            velocity = min(self.config.patrol_velocity, distance_error * 0.5)
        elif distance_error < -0.3:
            # Too close, back up slowly
            velocity = max(-0.2, distance_error * 0.3)
        else:
            # Good distance, just rotate to keep centered
            velocity = 0.0
        
        return ModeOutput(
            velocity=velocity,
            yaw_rate=yaw_rate,
            state=ModeState.RUNNING,
            confidence=intruder.confidence,
            message=f"Tracking at {intruder.distance:.1f}m"
        )
    
    def _process_returning(self, current_time: float) -> ModeOutput:
        """Process returning state - go back to patrol."""
        elapsed = current_time - self._state_start_time
        
        # Simple: rotate for a bit then resume patrol
        if elapsed < 2.0:
            return ModeOutput(
                velocity=0.0,
                yaw_rate=self.config.rotate_yaw_rate,
                state=ModeState.RUNNING,
                confidence=0.8,
                message="Returning to patrol..."
            )
        
        # Resume patrol
        self._patrol_state = PatrolState.PATROLLING
        self._state_start_time = current_time
        self._current_intruder = None
        logger.info("Resumed patrol")
        
        return self._process_patrolling(current_time)
    
    def _process_patrolling(self, current_time: float) -> ModeOutput:
        """Process patrolling state - move forward."""
        elapsed = current_time - self._state_start_time
        
        # Check if should rotate
        if elapsed > self.config.patrol_forward_time:
            self._patrol_state = PatrolState.ROTATING
            self._state_start_time = current_time
            self._patrol_cycle += 1
            return self._process_rotating(current_time)
        
        return ModeOutput(
            velocity=self.config.patrol_velocity,
            yaw_rate=0.0,
            state=ModeState.RUNNING,
            confidence=1.0,
            message=f"Patrolling... ({elapsed:.1f}s / {self.config.patrol_forward_time}s)"
        )
    
    def _process_rotating(self, current_time: float) -> ModeOutput:
        """Process rotating state - turn to new direction."""
        elapsed = current_time - self._state_start_time
        
        # Check if rotation complete
        if elapsed > self.config.patrol_rotate_time:
            self._patrol_state = PatrolState.PATROLLING
            self._state_start_time = current_time
            return self._process_patrolling(current_time)
        
        # Alternate rotation direction each cycle
        direction = 1 if self._patrol_cycle % 2 == 0 else -1
        
        return ModeOutput(
            velocity=0.0,
            yaw_rate=self.config.rotate_yaw_rate * direction,
            state=ModeState.RUNNING,
            confidence=1.0,
            message=f"Rotating... ({elapsed:.1f}s / {self.config.patrol_rotate_time}s)"
        )
    
    def _create_visualization(
        self,
        frame: np.ndarray,
        intruder: Optional[Intruder],
        output: ModeOutput
    ) -> np.ndarray:
        """Create visualization overlay."""
        viz = frame.copy()
        h, w = viz.shape[:2]
        
        # Determine color based on state
        if self._patrol_state in (PatrolState.ALERT, PatrolState.TRACKING):
            color = self.config.alert_color
            status = "🚨 INTRUDER DETECTED!"
        else:
            color = self.config.normal_color
            status = "Patrolling..."
        
        # Draw status bar
        cv2.rectangle(viz, (0, 0), (w, 40), color, -1)
        cv2.putText(
            viz, status,
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            (255, 255, 255), 2
        )
        
        # Draw intruder bounding box
        if intruder is not None:
            x1, y1, x2, y2 = intruder.bbox
            cv2.rectangle(viz, (x1, y1), (x2, y2), self.config.alert_color, 3)
            
            # Draw distance label
            label = f"INTRUDER {intruder.distance:.1f}m"
            cv2.putText(
                viz, label,
                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                self.config.alert_color, 2
            )
        
        # Draw patrol info
        info_y = h - 80
        cv2.putText(
            viz, f"State: {self._patrol_state.name}",
            (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (255, 255, 255), 1
        )
        cv2.putText(
            viz, f"Cycle: {self._patrol_cycle}",
            (10, info_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (255, 255, 255), 1
        )
        cv2.putText(
            viz, f"Intruders: {len(self._intruders_detected)}",
            (10, info_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (255, 255, 255), 1
        )
        cv2.putText(
            viz, output.message,
            (10, info_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (200, 200, 200), 1
        )
        
        return viz
    
    def _create_stop_output(self, message: str) -> ModeOutput:
        """Create stop output with message."""
        return ModeOutput(
            velocity=0.0,
            yaw_rate=0.0,
            state=self._state,
            confidence=0.0,
            message=message
        )
