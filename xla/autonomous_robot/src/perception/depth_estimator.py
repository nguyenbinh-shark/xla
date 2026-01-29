"""
Depth Estimation Module for Bounding Box Distance Measurement.
Uses Intel RealSense depth data to measure distance at detected object boxes.
"""

import cv2
import numpy as np
import logging
from typing import Tuple, Optional, List
from dataclasses import dataclass

from src.core import config

logger = logging.getLogger(__name__)


@dataclass
class BoxDepthResult:
    """Result of depth measurement for a bounding box."""
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[int, int]  # (cx, cy)
    center_depth: float  # Depth at center point (meters)
    avg_depth: float  # Average depth in box region (meters)
    min_depth: float  # Minimum depth in box (closest point)
    max_depth: float  # Maximum depth in box (farthest point)
    valid: bool  # Whether depth measurement is valid


class DepthEstimator:
    """
    Measures distance from RealSense depth data at bounding box locations.
    Provides multiple measurement methods: center point, average, min/max.
    """

    def __init__(
        self,
        median_filter_size: int = config.DEPTH_MEDIAN_FILTER_SIZE,
        min_valid_depth: float = config.DEPTH_MIN_VALID,
        max_valid_depth: float = config.DEPTH_MAX_VALID
    ):
        """
        Initialize the depth estimator.

        Args:
            median_filter_size: Size of median filter for noise reduction
            min_valid_depth: Minimum valid depth value (meters)
            max_valid_depth: Maximum valid depth value (meters)
        """
        self.median_filter_size = median_filter_size
        self.min_valid_depth = min_valid_depth
        self.max_valid_depth = max_valid_depth
        
        # Depth calibration parameters from config
        self.calibration_enabled = config.DEPTH_CALIBRATION_ENABLED
        self.correction_factor = config.DEPTH_CORRECTION_FACTOR  
        self.depth_offset = config.DEPTH_OFFSET
    
    def apply_calibration(self, depth: float) -> float:
        """
        Apply depth calibration correction if enabled.
        
        Args:
            depth: Raw depth value in meters
            
        Returns:
            Calibrated depth value
        """
        if not self.calibration_enabled or depth <= 0:
            return depth
            
        # Apply multiplicative correction and additive offset
        corrected_depth = depth * self.correction_factor + self.depth_offset
        return max(0, corrected_depth)

    def get_depth_at_center(
        self,
        depth_frame: np.ndarray,
        bbox: Tuple[int, int, int, int]
    ) -> BoxDepthResult:
        """
        Measure depth at the center of a bounding box.

        Args:
            depth_frame: Depth frame in meters (from RealSense)
            bbox: Bounding box as (x1, y1, x2, y2)

        Returns:
            BoxDepthResult with depth measurements
        """
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        # Get depth at center with median filtering
        center_depth = self._get_filtered_depth_at_point(depth_frame, cx, cy)

        # Get depth statistics in the box region
        avg_depth, min_depth, max_depth = self._get_box_depth_stats(
            depth_frame, bbox
        )

        valid = center_depth > 0 or avg_depth > 0

        return BoxDepthResult(
            bbox=bbox,
            center=(cx, cy),
            center_depth=center_depth,
            avg_depth=avg_depth,
            min_depth=min_depth,
            max_depth=max_depth,
            valid=valid
        )

    def get_depth_at_point(
        self,
        depth_frame: np.ndarray,
        x: int,
        y: int
    ) -> float:
        """
        Get depth value at a specific point.

        Args:
            depth_frame: Depth frame in meters
            x: X coordinate
            y: Y coordinate

        Returns:
            Depth in meters, or -1.0 if invalid
        """
        return self._get_filtered_depth_at_point(depth_frame, x, y)

    def measure_boxes(
        self,
        depth_frame: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]]
    ) -> List[BoxDepthResult]:
        """
        Measure depth for multiple bounding boxes.

        Args:
            depth_frame: Depth frame in meters
            bboxes: List of bounding boxes as (x1, y1, x2, y2)

        Returns:
            List of BoxDepthResult for each box
        """
        results = []
        for bbox in bboxes:
            result = self.get_depth_at_center(depth_frame, bbox)
            results.append(result)
        return results

    def _get_filtered_depth_at_point(
        self,
        depth_frame: np.ndarray,
        x: int,
        y: int
    ) -> float:
        """
        Get depth at point with median filtering for noise reduction.

        Args:
            depth_frame: Depth frame in meters
            x: X coordinate
            y: Y coordinate

        Returns:
            Filtered depth value in meters, or -1.0 if invalid
        """
        if depth_frame is None:
            return -1.0

        h, w = depth_frame.shape[:2]
        half_size = self.median_filter_size // 2

        # Clamp coordinates to valid range
        x = max(half_size, min(w - half_size - 1, x))
        y = max(half_size, min(h - half_size - 1, y))

        # Extract region around point
        region = depth_frame[
            y - half_size:y + half_size + 1,
            x - half_size:x + half_size + 1
        ]

        # Filter valid depth values
        valid_depths = region[
            (region > self.min_valid_depth) &
            (region < self.max_valid_depth)
        ]

        if valid_depths.size == 0:
            return -1.0

        depth = float(np.median(valid_depths))
        
        # Apply calibration correction
        depth = self.apply_calibration(depth)
        
        return depth

    def _get_box_depth_stats(
        self,
        depth_frame: np.ndarray,
        bbox: Tuple[int, int, int, int]
    ) -> Tuple[float, float, float]:
        """
        Get depth statistics within a bounding box region.

        Args:
            depth_frame: Depth frame in meters
            bbox: Bounding box as (x1, y1, x2, y2)

        Returns:
            Tuple of (average_depth, min_depth, max_depth)
            Returns (-1, -1, -1) if no valid depths found
        """
        if depth_frame is None:
            return -1.0, -1.0, -1.0

        h, w = depth_frame.shape[:2]
        x1, y1, x2, y2 = bbox

        # Clamp to frame bounds
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))

        # Extract box region
        region = depth_frame[y1:y2, x1:x2]

        if region.size == 0:
            return -1.0, -1.0, -1.0

        # Filter valid depths
        valid_depths = region[
            (region > self.min_valid_depth) &
            (region < self.max_valid_depth)
        ]

        if valid_depths.size == 0:
            return -1.0, -1.0, -1.0

        avg_depth = float(np.mean(valid_depths))
        min_depth = float(np.min(valid_depths))
        max_depth = float(np.max(valid_depths))

        return avg_depth, min_depth, max_depth

    def visualize_depth_box(
        self,
        frame: np.ndarray,
        result: BoxDepthResult,
        color: Tuple[int, int, int] = (0, 255, 0),
        show_center: bool = True,
        show_stats: bool = True
    ) -> np.ndarray:
        """
        Visualize depth measurement on a frame.

        Args:
            frame: BGR image to draw on
            result: BoxDepthResult to visualize
            color: Color for drawing (BGR)
            show_center: Whether to show center point
            show_stats: Whether to show all depth statistics

        Returns:
            Frame with visualization overlay
        """
        vis_frame = frame.copy()
        x1, y1, x2, y2 = result.bbox
        cx, cy = result.center

        # Draw bounding box
        thickness = 2 if result.valid else 1
        cv2.rectangle(vis_frame, (x1, y1), (x2, y2), color, thickness)

        # Draw center point
        if show_center:
            cv2.circle(vis_frame, (cx, cy), 6, color, -1)
            cv2.circle(vis_frame, (cx, cy), 8, (255, 255, 255), 2)

        # Draw crosshair at center
        crosshair_size = 15
        cv2.line(vis_frame, (cx - crosshair_size, cy), (cx + crosshair_size, cy), (255, 255, 255), 1)
        cv2.line(vis_frame, (cx, cy - crosshair_size), (cx, cy + crosshair_size), (255, 255, 255), 1)

        # Create depth label
        if result.center_depth > 0:
            depth_text = f"{result.center_depth:.2f}m"
        else:
            depth_text = "N/A"

        # Draw main depth label at center
        label_size, _ = cv2.getTextSize(depth_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        label_x = cx - label_size[0] // 2
        label_y = cy + label_size[1] + 15

        # Background for text
        cv2.rectangle(
            vis_frame,
            (label_x - 5, label_y - label_size[1] - 5),
            (label_x + label_size[0] + 5, label_y + 5),
            (0, 0, 0),
            -1
        )
        cv2.putText(
            vis_frame,
            depth_text,
            (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        # Show detailed statistics
        if show_stats and result.valid:
            stats_y = y1 - 10
            stats_text = f"Avg:{result.avg_depth:.2f}m Min:{result.min_depth:.2f}m"

            stats_size, _ = cv2.getTextSize(stats_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(
                vis_frame,
                (x1, stats_y - stats_size[1] - 5),
                (x1 + stats_size[0] + 5, stats_y + 5),
                color,
                -1
            )
            cv2.putText(
                vis_frame,
                stats_text,
                (x1 + 2, stats_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

        return vis_frame

    def visualize_multiple_boxes(
        self,
        frame: np.ndarray,
        results: List[BoxDepthResult],
        colors: Optional[List[Tuple[int, int, int]]] = None
    ) -> np.ndarray:
        """
        Visualize multiple depth measurements on a frame.

        Args:
            frame: BGR image to draw on
            results: List of BoxDepthResult to visualize
            colors: Optional list of colors for each box

        Returns:
            Frame with visualization overlay
        """
        vis_frame = frame.copy()

        if colors is None:
            # Generate distinct colors
            colors = [
                (0, 255, 0),    # Green
                (255, 0, 0),    # Blue
                (0, 0, 255),    # Red
                (255, 255, 0),  # Cyan
                (255, 0, 255),  # Magenta
                (0, 255, 255),  # Yellow
            ]

        for i, result in enumerate(results):
            color = colors[i % len(colors)]
            vis_frame = self.visualize_depth_box(
                vis_frame, result, color,
                show_center=True, show_stats=True
            )

        return vis_frame


def measure_distance_from_detection(
    depth_frame: np.ndarray,
    bbox: Tuple[int, int, int, int]
) -> float:
    """
    Convenient function to measure distance at center of a detection box.

    Args:
        depth_frame: Depth frame in meters (from RealSense camera)
        bbox: Bounding box as (x1, y1, x2, y2)

    Returns:
        Distance in meters at center of box, or -1.0 if invalid
    """
    estimator = DepthEstimator()
    result = estimator.get_depth_at_center(depth_frame, bbox)
    return result.center_depth


# Example usage and testing
if __name__ == "__main__":
    import sys
    sys.path.insert(0, '/home/nguyenbinh/Desktop/xla/autonomous_robot')
    from camera import RealSenseCamera

    logging.basicConfig(level=logging.INFO)

    print("Testing Depth Estimator with RealSense camera...")
    print("Press 'q' to quit, click to measure distance at point")

    camera = RealSenseCamera()
    estimator = DepthEstimator()

    # Mouse callback for clicking to measure distance
    clicked_point = None

    def mouse_callback(event, x, y, flags, param):
        global clicked_point
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked_point = (x, y)

    cv2.namedWindow("Depth Measurement")
    cv2.setMouseCallback("Depth Measurement", mouse_callback)

    if camera.start():
        try:
            while True:
                color_frame, depth_frame = camera.get_frames()

                if color_frame is None or depth_frame is None:
                    continue

                vis_frame = color_frame.copy()

                # Draw example boxes for testing
                h, w = color_frame.shape[:2]
                test_boxes = [
                    (w // 4, h // 4, w // 2, h // 2),  # Top-left region
                    (w // 2, h // 4, 3 * w // 4, h // 2),  # Top-right region
                ]

                # Measure depth for test boxes
                results = estimator.measure_boxes(depth_frame, test_boxes)
                vis_frame = estimator.visualize_multiple_boxes(vis_frame, results)

                # Measure at clicked point
                if clicked_point:
                    px, py = clicked_point
                    depth = estimator.get_depth_at_point(depth_frame, px, py)
                    cv2.circle(vis_frame, (px, py), 10, (0, 0, 255), -1)
                    cv2.putText(
                        vis_frame,
                        f"Click: {depth:.2f}m" if depth > 0 else "Click: N/A",
                        (px + 15, py),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2
                    )

                # Show center depth reading
                center_depth = estimator.get_depth_at_point(depth_frame, w // 2, h // 2)
                cv2.putText(
                    vis_frame,
                    f"Center: {center_depth:.2f}m",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                cv2.imshow("Depth Measurement", vis_frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

        finally:
            camera.stop()
            cv2.destroyAllWindows()
    else:
        print("Failed to start camera")
