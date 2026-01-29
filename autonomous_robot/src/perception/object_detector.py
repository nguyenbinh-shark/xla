"""
Object Detection Module using YOLO.
Handles detection of obstacles with depth estimation.
Simplified version - no lane classification.
"""

import cv2
import numpy as np
import logging
from typing import List, Optional, Tuple
from dataclasses import dataclass

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    logging.warning("ultralytics not installed. Object detection will be disabled.")

from src.core import config

logger = logging.getLogger(__name__)


@dataclass
class DetectedObject:
    """Represents a detected object with all relevant information."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[int, int]  # (cx, cy)
    depth: float  # Distance in meters
    is_obstacle: bool  # Whether it's a valid obstacle (close enough)


@dataclass
class ObjectDetectionResult:
    """Complete result of object detection."""
    objects: List[DetectedObject]
    obstacles: List[DetectedObject]  # Objects within safe distance
    closest_obstacle: Optional[DetectedObject]
    emergency_stop: bool  # True if any obstacle is too close


class ObjectDetector:
    """
    YOLO-based object detector with depth estimation.
    Simplified version without lane classification.
    """

    def __init__(self, model_path: str = config.YOLO_MODEL_PATH):
        """
        Initialize the object detector.

        Args:
            model_path: Path to YOLO model weights
        """
        self.model: Optional[YOLO] = None
        self.model_path = model_path

        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(model_path)
                logger.info(f"YOLO model loaded: {model_path}")
            except Exception as e:
                logger.error(f"Failed to load YOLO model: {e}")
                self.model = None
        else:
            logger.warning("YOLO not available - object detection disabled")

    def detect(
        self,
        color_frame: np.ndarray,
        depth_frame: Optional[np.ndarray] = None
    ) -> ObjectDetectionResult:
        """
        Detect objects in the frame.

        Args:
            color_frame: BGR image from camera
            depth_frame: Depth image in meters

        Returns:
            ObjectDetectionResult containing all detected objects
        """
        if self.model is None or color_frame is None:
            return self._create_empty_result()

        detected_objects = []

        try:
            # Run YOLO inference
            results = self.model(
                color_frame,
                conf=config.YOLO_CONFIDENCE_THRESHOLD,
                iou=config.YOLO_NMS_THRESHOLD,
                verbose=False
            )

            # Process detections
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue

                for box in boxes:
                    # Get class ID
                    class_id = int(box.cls[0])
                    
                    # If DETECT_CLASSES is None, detect all; otherwise filter
                    if config.DETECT_CLASSES is not None and class_id not in config.DETECT_CLASSES:
                        continue
                    
                    # Get class name
                    if config.DETECT_CLASSES is not None:
                        class_name = config.DETECT_CLASSES[class_id]
                    else:
                        class_name = self.model.names[class_id]

                    # Extract detection info
                    confidence = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                    # Calculate center
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    # Get depth with multi-point sampling for more robust estimation
                    depth = self._get_depth_at_point(depth_frame, cx, cy, bbox=(x1, y1, x2, y2))

                    # Determine if it's a valid obstacle
                    is_obstacle = (
                        confidence >= config.YOLO_CONFIDENCE_THRESHOLD and
                        0 < depth < config.D_SAFE
                    )

                    detected_obj = DetectedObject(
                        class_id=class_id,
                        class_name=class_name,
                        confidence=confidence,
                        bbox=(x1, y1, x2, y2),
                        center=(cx, cy),
                        depth=depth,
                        is_obstacle=is_obstacle
                    )

                    detected_objects.append(detected_obj)

        except Exception as e:
            logger.error(f"Object detection error: {e}")

        return self._create_result(detected_objects)

    def _get_depth_at_point(
        self, 
        depth_frame: np.ndarray, 
        x: int, 
        y: int,
        bbox: Tuple[int, int, int, int] = None
    ) -> float:
        """
        Get depth value at point with median filtering.
        If bbox is provided, sample multiple points within the box for more robust estimation.

        Args:
            depth_frame: Depth frame in meters
            x: X coordinate (center)
            y: Y coordinate (center)
            bbox: Optional bounding box (x1, y1, x2, y2) for multi-point sampling

        Returns:
            Depth in meters, or -1 if invalid
        """
        if depth_frame is None:
            return -1.0

        h, w = depth_frame.shape[:2]
        half_size = config.DEPTH_MEDIAN_FILTER_SIZE // 2

        # If bbox provided, sample multiple points for more robust depth
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            # Clamp to valid range
            x1, x2 = max(0, x1), min(w-1, x2)
            y1, y2 = max(0, y1), min(h-1, y2)
            
            # Sample from center and lower-center region (feet area for person)
            sample_points = [
                (x, y),                          # Center
                (x, min(y + (y2 - y)//2, y2)),   # Lower center
                (x - (x2-x1)//4, y),             # Left of center
                (x + (x2-x1)//4, y),             # Right of center
                (x, y - (y-y1)//3),              # Upper center
            ]
            
            all_depths = []
            for px, py in sample_points:
                px = max(half_size, min(w - half_size - 1, int(px)))
                py = max(half_size, min(h - half_size - 1, int(py)))
                
                region = depth_frame[
                    py - half_size:py + half_size + 1,
                    px - half_size:px + half_size + 1
                ]
                
                valid_depths = region[
                    (region > config.DEPTH_MIN_VALID) & 
                    (region < config.DEPTH_MAX_VALID)
                ]
                
                if valid_depths.size > 0:
                    all_depths.extend(valid_depths.flatten().tolist())
            
            if len(all_depths) > 0:
                # Return minimum valid depth (closest point)
                return float(np.percentile(all_depths, 25))  # 25th percentile to ignore outliers
            return -1.0

        # Single point sampling (original behavior)
        # Ensure coordinates are within bounds
        x = max(half_size, min(w - half_size - 1, x))
        y = max(half_size, min(h - half_size - 1, y))

        # Extract region for median filtering
        region = depth_frame[
            y - half_size:y + half_size + 1,
            x - half_size:x + half_size + 1
        ]

        # Filter valid depth values
        valid_depths = region[
            (region > config.DEPTH_MIN_VALID) & 
            (region < config.DEPTH_MAX_VALID)
        ]

        if valid_depths.size == 0:
            return -1.0

        return float(np.median(valid_depths))

    def _create_result(
        self, 
        objects: List[DetectedObject]
    ) -> ObjectDetectionResult:
        """
        Create detection result from list of detected objects.

        Args:
            objects: List of detected objects

        Returns:
            ObjectDetectionResult
        """
        # Filter to only obstacles (valid detections within safe distance)
        obstacles = [obj for obj in objects if obj.is_obstacle]

        # Find closest obstacle
        closest_obstacle = None
        if obstacles:
            closest_obstacle = min(obstacles, key=lambda o: o.depth)

        # Check for emergency stop condition
        emergency_stop = any(
            obj.depth < config.D_EMERGENCY and obj.depth > 0
            for obj in obstacles
        )

        return ObjectDetectionResult(
            objects=objects,
            obstacles=obstacles,
            closest_obstacle=closest_obstacle,
            emergency_stop=emergency_stop
        )

    def _create_empty_result(self) -> ObjectDetectionResult:
        """Create empty result when detection is not possible."""
        return ObjectDetectionResult(
            objects=[],
            obstacles=[],
            closest_obstacle=None,
            emergency_stop=False
        )

    def visualize(
        self, 
        frame: np.ndarray, 
        result: ObjectDetectionResult
    ) -> np.ndarray:
        """
        Draw object detection visualization on frame.

        Args:
            frame: Original BGR frame
            result: Object detection result

        Returns:
            Frame with visualization overlay
        """
        vis_frame = frame.copy()

        for obj in result.objects:
            x1, y1, x2, y2 = obj.bbox
            cx, cy = obj.center

            # Choose color based on obstacle status and distance
            if obj.is_obstacle:
                if obj.depth < config.D_EMERGENCY:
                    color = (0, 0, 255)  # Red - emergency
                elif obj.depth < config.D_SAFE:
                    color = (0, 165, 255)  # Orange - warning
                else:
                    color = (0, 255, 0)  # Green - safe
                thickness = 3
            else:
                color = (128, 128, 128)  # Gray - not obstacle
                thickness = 1

            # Draw bounding box
            cv2.rectangle(vis_frame, (x1, y1), (x2, y2), color, thickness)

            # Draw center point
            cv2.circle(vis_frame, (cx, cy), 5, color, -1)

            # Create label
            if obj.depth > 0:
                label = f"{obj.class_name}: {obj.depth:.2f}m"
            else:
                label = f"{obj.class_name}: N/A"

            # Draw label background
            label_size, _ = cv2.getTextSize(
                label, 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                1
            )
            cv2.rectangle(
                vis_frame,
                (x1, y1 - label_size[1] - 5),
                (x1 + label_size[0], y1),
                color,
                -1
            )

            # Draw label text
            cv2.putText(
                vis_frame,
                label,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

        # Add obstacle count
        cv2.putText(
            vis_frame,
            f"Objects: {len(result.objects)} | Obstacles: {len(result.obstacles)}",
            (10, frame.shape[0] - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        # Emergency warning
        if result.emergency_stop:
            cv2.putText(
                vis_frame,
                "EMERGENCY STOP!",
                (frame.shape[1] // 2 - 100, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                3
            )

        # Closest obstacle distance
        if result.closest_obstacle:
            cv2.putText(
                vis_frame,
                f"Closest: {result.closest_obstacle.depth:.2f}m",
                (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )

        return vis_frame
