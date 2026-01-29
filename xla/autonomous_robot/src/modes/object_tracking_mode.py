"""
Object Tracking Mode - Track and follow a detected object.
Uses YOLO for detection and depth camera for distance.
"""

import cv2
import numpy as np
import logging
from typing import Optional, Any, Tuple
from dataclasses import dataclass

from .base_mode import BaseMode, ModeOutput, ModeState
from src.perception import ObjectDetector, DepthEstimator

logger = logging.getLogger(__name__)


@dataclass
class ObjectTrackingConfig:
    """Configuration for object tracking mode."""
    # Target settings
    target_class: str = "person"     # COCO class name to track
    target_distance: float = 1.5     # Desired distance from target (m)
    
    # Speed control
    max_speed: float = 1.0           # Maximum forward speed (m/s)
    min_speed: float = 0.0           # Minimum speed
    approach_speed: float = 0.5      # Speed when approaching target
    
    # Steering control (Gimbal-style)
    steering_gain: float = 2.0       # Position error -> yaw rate
    max_yaw_rate: float = 1.0        # Maximum yaw rate (rad/s)
    steering_deadband: float = 0.05  # Ignore small position errors
    
    # Distance control
    distance_deadband: float = 0.2   # Acceptable distance error (m)
    distance_gain: float = 0.5       # Distance error -> velocity
    min_safe_distance: float = 0.5   # Minimum distance to maintain (m)
    max_tracking_distance: float = 5.0  # Maximum distance to track (m)
    backup_on_too_close: bool = True  # Lùi lại nếu quá gần
    
    # Search behavior
    search_yaw_rate: float = 0.4     # Yaw rate when searching
    max_frames_lost: int = 60        # Frames before giving up
    
    # Smoothing (for gimbal-like behavior)
    yaw_smoothing: float = 0.3       # 0-1, higher = smoother but slower response
    velocity_smoothing: float = 0.3  # 0-1, higher = smoother acceleration
    
    # Detection thresholds
    min_confidence: float = 0.5      # Minimum detection confidence
    min_box_area: int = 2000         # Minimum bounding box area (pixels^2)


class ObjectTrackingMode(BaseMode):
    """
    Object tracking mode - follow a specific object class.
    
    Pipeline:
        Camera Frame → Object Detection → Target Selection → Control Output
    
    Features:
    - Track specific object class (person, car, etc.)
    - Maintain target distance using depth
    - Center object in frame with steering
    - Search behavior when target is lost
    - Smooth approach/retreat control
    """
    
    def __init__(self, config: Optional[ObjectTrackingConfig] = None):
        """
        Initialize object tracking mode.
        
        Args:
            config: Configuration parameters
        """
        super().__init__()
        
        self.config = config or ObjectTrackingConfig()
        
        # Initialize detector and depth estimator
        self.object_detector = ObjectDetector()
        self.depth_estimator = DepthEstimator()
        
        # Tracking state
        self._target = None               # Current tracked object
        self._frames_lost = 0             # Frames since target last seen
        self._last_target_position = 0.0  # Last known x position (-1 to 1)
        self._search_direction = 1        # Search rotation direction
        
        # Smoothing state (for gimbal-like behavior)
        self._smoothed_yaw_rate = 0.0
        self._smoothed_velocity = 0.0
        
        # Statistics
        self._total_frames = 0
        self._tracking_frames = 0
        
        logger.info(f"ObjectTrackingMode initialized - tracking '{self.config.target_class}'")
    
    def get_name(self) -> str:
        """Get mode name."""
        return "Object Tracking"
    
    def reset(self) -> None:
        """Reset mode state."""
        self._state = ModeState.IDLE
        self._target = None
        self._frames_lost = 0
        self._last_target_position = 0.0
        self._search_direction = 1
        self._smoothed_yaw_rate = 0.0
        self._smoothed_velocity = 0.0
        self._total_frames = 0
        self._tracking_frames = 0
        logger.info("ObjectTrackingMode reset")
    
    def set_target_class(self, class_name: str) -> None:
        """
        Change target class to track.
        
        Args:
            class_name: COCO class name (person, car, bicycle, etc.)
        """
        self.config.target_class = class_name
        self.reset()
        logger.info(f"Target class changed to '{class_name}'")
    
    def process(
        self,
        color_frame: np.ndarray,
        depth_frame: Optional[np.ndarray] = None,
        feedback: Optional[Any] = None
    ) -> ModeOutput:
        """
        Process frame and compute tracking commands.
        
        Args:
            color_frame: BGR image from camera
            depth_frame: Depth image from camera
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
        
        # Run object detection
        detection_result = self.object_detector.detect(color_frame, depth_frame)
        
        # Find target object
        target = self._find_target(detection_result, color_frame.shape)
        
        # Process based on whether target was found
        if target is not None:
            self._target = target
            output = self._process_target_found(target, depth_frame, feedback)
        else:
            output = self._process_target_lost()
        
        # Create visualization
        output.viz_frame = self._create_visualization(
            color_frame, detection_result, target, output
        )
        
        return output
    
    def _find_target(self, detection_result, frame_shape) -> Optional[Any]:
        """
        Find the best target object from detections.
        
        Prioritizes:
        1. Same object as before (tracking continuity)
        2. Largest bounding box (closest/most prominent)
        3. Highest confidence
        """
        if detection_result is None or not detection_result.objects:
            return None
        
        h, w = frame_shape[:2]
        candidates = []
        
        for obj in detection_result.objects:
            # Filter by class
            if obj.class_name.lower() != self.config.target_class.lower():
                continue
            
            # Filter by confidence
            if obj.confidence < self.config.min_confidence:
                continue
            
            # Filter by size
            bbox = obj.bbox  # (x1, y1, x2, y2)
            box_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
            if box_area < self.config.min_box_area:
                continue
            
            # Filter by distance
            if obj.depth > 0:
                if obj.depth > self.config.max_tracking_distance:
                    continue
            
            candidates.append(obj)
        
        if not candidates:
            return None
        
        # Sort by size (larger = better) then confidence
        candidates.sort(
            key=lambda o: (
                (o.bbox[2] - o.bbox[0]) * (o.bbox[3] - o.bbox[1]),
                o.confidence
            ),
            reverse=True
        )
        
        return candidates[0]
    
    def _process_target_found(
        self,
        target,
        depth_frame: Optional[np.ndarray],
        feedback: Optional[Any]
    ) -> ModeOutput:
        """Process when target is found."""
        self._tracking_frames += 1
        self._frames_lost = 0
        self._state = ModeState.RUNNING
        
        # Calculate target center position (normalized -1 to 1)
        frame_center_x = 320  # Assuming 640 width
        target_x = target.center[0]
        position_error = (target_x - frame_center_x) / frame_center_x
        
        # Save for recovery
        self._last_target_position = position_error
        
        # Calculate yaw rate to center target
        yaw_rate = self._calculate_yaw_rate(position_error)
        
        # Calculate velocity based on distance
        velocity = self._calculate_velocity(target.depth, feedback)
        
        return ModeOutput(
            velocity=velocity,
            yaw_rate=yaw_rate,
            state=ModeState.RUNNING,
            confidence=target.confidence,
            message=f"Tracking {self.config.target_class}: d={target.depth:.2f}m, err={position_error:+.2f}"
        )
    
    def _process_target_lost(self) -> ModeOutput:
        """Process when target is lost - enter search mode."""
        self._frames_lost += 1
        
        # Give up after too many frames
        if self._frames_lost > self.config.max_frames_lost:
            self._state = ModeState.ERROR
            return ModeOutput(
                velocity=0.0,
                yaw_rate=0.0,
                state=ModeState.ERROR,
                confidence=0.0,
                message=f"Target lost for {self._frames_lost} frames - STOPPED"
            )
        
        # Search mode
        self._state = ModeState.SEARCHING
        
        # Determine search direction
        if self._frames_lost == 1:
            self._search_direction = 1 if self._last_target_position > 0 else -1
        
        search_yaw = self._search_direction * self.config.search_yaw_rate
        
        return ModeOutput(
            velocity=0.0,  # Stop while searching
            yaw_rate=search_yaw,
            state=ModeState.SEARCHING,
            confidence=0.0,
            message=f"Searching for {self.config.target_class}... ({self._frames_lost})"
        )
    
    def _calculate_yaw_rate(self, position_error: float) -> float:
        """
        Calculate yaw rate to center target in frame (gimbal-style).
        Uses deadband and smoothing for stable tracking.
        """
        # Deadband - ignore small errors
        if abs(position_error) < self.config.steering_deadband:
            target_yaw = 0.0
        else:
            # Negative because positive x (right) needs negative yaw (turn right)
            target_yaw = -self.config.steering_gain * position_error
        
        # Clamp to max
        target_yaw = self._clamp_yaw_rate(target_yaw)
        
        # Apply smoothing for gimbal-like behavior
        alpha = 1.0 - self.config.yaw_smoothing
        self._smoothed_yaw_rate = (alpha * target_yaw + 
                                   self.config.yaw_smoothing * self._smoothed_yaw_rate)
        
        return self._smoothed_yaw_rate
    
    def _calculate_velocity(
        self,
        distance: float,
        feedback: Optional[Any]
    ) -> float:
        """
        Calculate velocity based on distance to target.
        
        - Too far: Move forward
        - Good distance: Stop or slow
        - Too close: Move backward
        """
        if distance <= 0:
            # No valid depth, slowly approach
            return self.config.approach_speed * 0.5
        
        # Distance error (positive = too far, negative = too close)
        distance_error = distance - self.config.target_distance
        
        # Deadband - don't move if within acceptable range
        if abs(distance_error) < self.config.distance_deadband:
            target_velocity = 0.0
        elif distance < self.config.min_safe_distance:
            # Too close - backup if enabled, else stop
            if self.config.backup_on_too_close:
                target_velocity = -0.2  # Slow backup
            else:
                target_velocity = 0.0
        else:
            # Calculate velocity proportional to distance error
            target_velocity = distance_error * self.config.distance_gain
            
            # Clamp velocity
            target_velocity = max(-self.config.max_speed * 0.3, 
                                  min(self.config.max_speed, target_velocity))
        
        # Apply smoothing for smooth acceleration/deceleration
        alpha = 1.0 - self.config.velocity_smoothing
        self._smoothed_velocity = (alpha * target_velocity + 
                                   self.config.velocity_smoothing * self._smoothed_velocity)
        
        return self._smoothed_velocity
    
    def _create_visualization(
        self,
        frame: np.ndarray,
        detection_result,
        target,
        output: ModeOutput
    ) -> np.ndarray:
        """Create visualization with detection boxes and status."""
        vis = frame.copy()
        h, w = vis.shape[:2]
        
        # Draw all detections
        if detection_result is not None:
            for obj in detection_result.objects:
                x1, y1, x2, y2 = obj.bbox
                
                # Color based on whether it's the target
                if target is not None and obj == target:
                    color = (0, 255, 0)  # Green for target
                    thickness = 3
                elif obj.class_name.lower() == self.config.target_class.lower():
                    color = (0, 255, 255)  # Yellow for target class
                    thickness = 2
                else:
                    color = (128, 128, 128)  # Gray for others
                    thickness = 1
                
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)
                
                # Label
                label = f"{obj.class_name} {obj.confidence:.0%}"
                if obj.depth > 0:
                    label += f" {obj.depth:.1f}m"
                cv2.putText(vis, label, (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Draw target indicator
        if target is not None:
            cx, cy = target.center
            # Crosshair
            cv2.line(vis, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            cv2.line(vis, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
            cv2.circle(vis, (cx, cy), 25, (0, 255, 0), 2)
        
        # Draw center line (target position)
        cv2.line(vis, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        
        # Status bar
        status_bar_height = 50
        vis_with_bar = cv2.copyMakeBorder(
            vis, 0, status_bar_height, 0, 0,
            cv2.BORDER_CONSTANT, value=(40, 40, 40)
        )
        
        # State color
        state_color = {
            ModeState.RUNNING: (0, 255, 0),
            ModeState.SEARCHING: (0, 255, 255),
            ModeState.ERROR: (0, 0, 255),
        }.get(output.state, (255, 255, 255))
        
        # Mode and target
        cv2.putText(
            vis_with_bar,
            f"TRACKING: {self.config.target_class} | {output.state.name}",
            (10, h + 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, state_color, 1
        )
        
        # Control values
        cv2.putText(
            vis_with_bar,
            f"V:{output.velocity:.2f} Y:{output.yaw_rate:+.2f}",
            (10, h + 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )
        
        # Target distance
        if target is not None and target.depth > 0:
            cv2.putText(
                vis_with_bar,
                f"Dist: {target.depth:.2f}m (target: {self.config.target_distance:.1f}m)",
                (300, h + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
            )
        
        return vis_with_bar
    
    def get_statistics(self) -> dict:
        """Get mode statistics."""
        return {
            'total_frames': self._total_frames,
            'tracking_frames': self._tracking_frames,
            'tracking_rate': self._tracking_frames / max(1, self._total_frames),
            'current_state': self._state.name,
            'frames_lost': self._frames_lost,
            'target_class': self.config.target_class
        }
    
    def set_target_distance(self, distance: float) -> None:
        """Set desired distance from target."""
        self.config.target_distance = max(0.5, min(5.0, distance))
        logger.info(f"Target distance set to {self.config.target_distance:.1f}m")
