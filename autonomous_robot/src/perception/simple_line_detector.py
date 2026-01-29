"""
Simple Single Line Detector Module - Centerline Method.
Detects one line using contour centroid method.
Calculates:
- Position error: Distance from image center to line center
- Heading error: Angle between line direction and robot heading
"""

import cv2
import numpy as np
import logging
from typing import Optional, Tuple, List
from dataclasses import dataclass

from src.core import config

logger = logging.getLogger(__name__)


@dataclass
class LineDetectionResult:
    """Result of single line detection."""
    line_detected: bool
    
    # Position error: positive = line is to the right of center
    position_error: float  # normalized [-1, 1]
    position_error_pixels: float  # raw pixels
    
    # Heading error: positive = line tilts to the right
    heading_error: float  # radians
    heading_error_degrees: float  # degrees
    
    # Line center at look-ahead point
    line_center_x: float
    line_center_y: float
    
    # Confidence
    confidence: float
    
    # Centerline points for visualization
    centerline_points: Optional[List[Tuple[int, int]]] = None
    
    # Line search/recovery info
    search_direction: int = 0  # -1=left, 0=none, 1=right
    frames_lost: int = 0  # Số frame liên tiếp mất line


class SimpleLineDetector:
    """
    Line detector using centerline/centroid method.
    Better for following a single line.
    Includes line recovery mode when line is lost.
    """

    def __init__(self):
        """Initialize detector."""
        # Ensure kernel size is valid (must be odd and >= 1)
        kernel_size = max(1, config.MORPH_KERNEL_SIZE)
        if kernel_size % 2 == 0:
            kernel_size += 1
        
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (kernel_size, kernel_size)
        )
        
        self._roi_mask: Optional[np.ndarray] = None
        self._frame_size: Optional[Tuple[int, int]] = None
        
        # Smoothing - adaptive based on confidence
        self._prev_result: Optional[LineDetectionResult] = None
        self._base_smoothing_factor = 0.4
        
        # Number of horizontal slices for centerline detection
        self._num_slices = 10
        
        # Line recovery state
        self._frames_lost = 0
        self._max_frames_before_search = 3  # Số frame mất line trước khi bắt đầu search
        self._search_direction = 0  # -1=left, 0=none, 1=right
        self._last_known_position = 0.0  # Vị trí cuối cùng của line (normalized)
        self._search_amplitude = 0.3  # Biên độ search ban đầu
        self._search_increment = 0.1  # Tăng biên độ mỗi lần không tìm thấy
        
        logger.info(f"LineDetector initialized: kernel={kernel_size}, slices={self._num_slices}")

    def detect(self, frame: np.ndarray) -> LineDetectionResult:
        """Detect single line in frame with recovery mode."""
        if frame is None or frame.size == 0:
            return self._handle_line_lost()

        height, width = frame.shape[:2]

        # Step 1: Preprocess
        binary = self._preprocess(frame)

        # Step 2: Find centerline points using horizontal slicing
        centerline_points = self._find_centerline(binary, height, width)

        if len(centerline_points) < 3:
            # Line not found - enter recovery mode
            return self._handle_line_lost()

        # Line found - reset recovery state
        self._frames_lost = 0
        self._search_direction = 0

        # Step 3: Fit line and calculate errors
        result = self._calculate_errors(centerline_points, width, height)
        
        # Save last known position for recovery
        self._last_known_position = result.position_error

        # Step 4: Adaptive smoothing based on confidence and curvature
        smoothing_factor = self._compute_adaptive_smoothing(result)
        if self._prev_result is not None and self._prev_result.line_detected:
            result = self._smooth_result(result, smoothing_factor)

        self._prev_result = result
        return result

    def _handle_line_lost(self) -> LineDetectionResult:
        """
        Handle case when line is lost.
        Returns a search command to help robot find line again.
        """
        self._frames_lost += 1
        
        # Determine search direction based on last known position
        if self._frames_lost == 1:
            # First frame lost - decide search direction
            if self._last_known_position > 0.1:
                self._search_direction = 1  # Line was on right, search right
            elif self._last_known_position < -0.1:
                self._search_direction = -1  # Line was on left, search left
            else:
                # Line was centered - alternate search
                self._search_direction = 1
            logger.warning(f"Line lost! Starting search: direction={self._search_direction}")
        
        # Calculate search error to send to robot
        search_error = 0.0
        if self._frames_lost > self._max_frames_before_search:
            # Oscillating search with increasing amplitude
            cycles = (self._frames_lost - self._max_frames_before_search) // 10
            amplitude = min(0.8, self._search_amplitude + cycles * self._search_increment)
            
            # Alternate direction every ~10 frames
            phase = (self._frames_lost - self._max_frames_before_search) % 20
            if phase < 10:
                search_error = amplitude * self._search_direction
            else:
                search_error = -amplitude * self._search_direction
                
            logger.debug(f"Searching: frames_lost={self._frames_lost}, error={search_error:.2f}")
        
        return LineDetectionResult(
            line_detected=False,
            position_error=search_error,
            position_error_pixels=search_error * (config.CAMERA_WIDTH / 2),
            heading_error=0.0,
            heading_error_degrees=0.0,
            line_center_x=config.CAMERA_WIDTH / 2,
            line_center_y=config.CAMERA_HEIGHT / 2,
            confidence=0.0,
            centerline_points=None,
            search_direction=self._search_direction,
            frames_lost=self._frames_lost
        )

    def _compute_adaptive_smoothing(self, result: LineDetectionResult) -> float:
        """
        Compute adaptive smoothing factor based on conditions.
        Lower smoothing (more responsive) when:
        - Low confidence
        - High heading error (sharp turn)
        - Recently recovered from lost line
        """
        alpha = self._base_smoothing_factor
        
        # Reduce smoothing for low confidence
        if result.confidence < 0.5:
            alpha *= 0.5
        
        # Reduce smoothing for sharp turns (high heading error)
        if abs(result.heading_error_degrees) > 15:
            alpha *= 0.6
        elif abs(result.heading_error_degrees) > 30:
            alpha *= 0.3
        
        # Reduce smoothing if just recovered from lost line
        if self._frames_lost > 0:
            alpha *= 0.3
        
        return max(0.1, min(0.6, alpha))

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess image with adaptive thresholding.
        Handles varying lighting conditions automatically.
        """
        height, width = frame.shape[:2]
        
        # Create ROI mask
        if self._roi_mask is None or self._frame_size != (height, width):
            self._frame_size = (height, width)
            self._roi_mask = np.zeros((height, width), dtype=np.uint8)
            
            vertices = np.array([[
                (int(width * config.ROI_BOTTOM_LEFT_X), int(height * config.ROI_BOTTOM_Y)),
                (int(width * config.ROI_BOTTOM_RIGHT_X), int(height * config.ROI_BOTTOM_Y)),
                (int(width * config.ROI_TOP_RIGHT_X), int(height * config.ROI_TOP_Y)),
                (int(width * config.ROI_TOP_LEFT_X), int(height * config.ROI_TOP_Y)),
            ]], dtype=np.int32)
            cv2.fillPoly(self._roi_mask, vertices, 255)

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Use adaptive thresholding for varying lighting conditions
        # This automatically adjusts threshold based on local neighborhood
        binary_adaptive = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            blockSize=51,  # Size of neighborhood (must be odd)
            C=10  # Constant subtracted from mean
        )
        
        # Also use Otsu's method as fallback/combination
        _, binary_otsu = cv2.threshold(
            gray, 0, 255, 
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        
        # Combine both methods - use intersection for robustness
        # This helps filter noise while keeping strong line signals
        binary = cv2.bitwise_and(binary_adaptive, binary_otsu)
        
        # If combination is too sparse, fall back to config threshold
        roi_pixels = cv2.countNonZero(cv2.bitwise_and(binary, self._roi_mask))
        if roi_pixels < 100:  # Not enough pixels detected
            _, binary = cv2.threshold(gray, config.BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

        # Apply ROI
        binary = cv2.bitwise_and(binary, self._roi_mask)

        # Morphological operations
        if config.MORPH_CLOSE_ITERATIONS > 0:
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.morph_kernel,
                                     iterations=config.MORPH_CLOSE_ITERATIONS)
        if config.MORPH_OPEN_ITERATIONS > 0:
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.morph_kernel,
                                     iterations=config.MORPH_OPEN_ITERATIONS)

        return binary

    def _find_centerline(
        self, 
        binary: np.ndarray, 
        height: int, 
        width: int
    ) -> List[Tuple[int, int]]:
        """
        Find centerline by scanning horizontal slices.
        For each slice, find the centroid of white pixels (the line).
        """
        centerline_points = []
        
        # Define ROI boundaries
        roi_top = int(height * config.ROI_TOP_Y)
        roi_bottom = int(height * config.ROI_BOTTOM_Y)
        
        # Create slices
        slice_height = (roi_bottom - roi_top) // self._num_slices
        
        for i in range(self._num_slices):
            y_start = roi_top + i * slice_height
            y_end = y_start + slice_height
            y_center = (y_start + y_end) // 2
            
            # Get slice
            slice_img = binary[y_start:y_end, :]
            
            # Find white pixels (line)
            white_pixels = np.where(slice_img > 0)
            
            if len(white_pixels[1]) > 10:  # Need enough pixels
                # Calculate centroid (average x position)
                x_center = int(np.mean(white_pixels[1]))
                centerline_points.append((x_center, y_center))
        
        return centerline_points

    def _calculate_errors(
        self, 
        points: List[Tuple[int, int]], 
        width: int, 
        height: int
    ) -> LineDetectionResult:
        """Calculate position and heading errors from centerline points."""
        points_array = np.array(points)
        
        # Use the bottom point (closest to robot) for position error
        # Sort by y (descending) to get bottom first
        sorted_points = sorted(points, key=lambda p: p[1], reverse=True)
        bottom_point = sorted_points[0]
        
        # Position error at bottom (look-ahead point)
        # Adjust for camera offset from robot center
        image_center_x = width / 2 + config.CAMERA_OFFSET_X
        position_error_pixels = bottom_point[0] - image_center_x
        position_error = position_error_pixels / (width / 2)
        position_error = np.clip(position_error, -1, 1)
        
        # Heading error - fit a line through points
        if len(points) >= 2:
            # Fit line: x = m*y + b
            y_vals = points_array[:, 1]
            x_vals = points_array[:, 0]
            
            try:
                # Linear fit
                coeffs = np.polyfit(y_vals, x_vals, 1)
                slope = coeffs[0]  # dx/dy
                
                # Heading error = arctan(slope)
                # slope > 0 means line tilts right as we go up
                heading_error = np.arctan(slope)
            except:
                heading_error = 0.0
        else:
            heading_error = 0.0
        
        # Confidence based on number of detected points
        confidence = len(points) / self._num_slices
        
        return LineDetectionResult(
            line_detected=True,
            position_error=float(position_error),
            position_error_pixels=float(position_error_pixels),
            heading_error=float(heading_error),
            heading_error_degrees=float(np.degrees(heading_error)),
            line_center_x=float(bottom_point[0]),
            line_center_y=float(bottom_point[1]),
            confidence=float(confidence),
            centerline_points=points
        )

    def _smooth_result(self, result: LineDetectionResult, alpha: float = None) -> LineDetectionResult:
        """Apply low-pass filter with adaptive smoothing."""
        if self._prev_result is None:
            return result
        
        if alpha is None:
            alpha = self._base_smoothing_factor
        
        smoothed_pos = alpha * result.position_error + (1 - alpha) * self._prev_result.position_error
        smoothed_pos_px = alpha * result.position_error_pixels + (1 - alpha) * self._prev_result.position_error_pixels
        smoothed_heading = alpha * result.heading_error + (1 - alpha) * self._prev_result.heading_error
        
        return LineDetectionResult(
            line_detected=True,
            position_error=smoothed_pos,
            position_error_pixels=smoothed_pos_px,
            heading_error=smoothed_heading,
            heading_error_degrees=np.degrees(smoothed_heading),
            line_center_x=result.line_center_x,
            line_center_y=result.line_center_y,
            confidence=result.confidence,
            centerline_points=result.centerline_points,
            search_direction=0,
            frames_lost=0
        )

    def _create_invalid_result(self) -> LineDetectionResult:
        """Create invalid result."""
        return LineDetectionResult(
            line_detected=False,
            position_error=0.0,
            position_error_pixels=0.0,
            heading_error=0.0,
            heading_error_degrees=0.0,
            line_center_x=0.0,
            line_center_y=0.0,
            confidence=0.0,
            centerline_points=None
        )

    def visualize(self, frame: np.ndarray, result: LineDetectionResult) -> np.ndarray:
        """Draw visualization."""
        vis = frame.copy()
        height, width = frame.shape[:2]
        
        # Draw ROI
        vertices = np.array([[
            (int(width * config.ROI_BOTTOM_LEFT_X), int(height * config.ROI_BOTTOM_Y)),
            (int(width * config.ROI_BOTTOM_RIGHT_X), int(height * config.ROI_BOTTOM_Y)),
            (int(width * config.ROI_TOP_RIGHT_X), int(height * config.ROI_TOP_Y)),
            (int(width * config.ROI_TOP_LEFT_X), int(height * config.ROI_TOP_Y)),
        ]], dtype=np.int32)
        cv2.polylines(vis, vertices, True, (0, 255, 255), 2)

        # Draw center reference lines
        # Image center (no offset) - gray line
        cv2.line(vis, (width // 2, 0), (width // 2, height), (128, 128, 128), 1)
        # Robot center (with offset) - magenta line - this is the actual reference point
        center_x = int(width / 2 + config.CAMERA_OFFSET_X)
        cv2.line(vis, (center_x, 0), (center_x, height), (255, 0, 255), 2)

        if not result.line_detected:
            # Show search mode info
            if result.frames_lost > 0:
                search_text = f"SEARCHING... ({result.frames_lost} frames)"
                color = (0, 165, 255)  # Orange
                
                # Draw search direction arrow
                if result.search_direction != 0:
                    arrow_x = center_x + int(result.position_error * width / 2)
                    cv2.arrowedLine(vis, (center_x, height // 2), 
                                   (arrow_x, height // 2), (0, 165, 255), 3)
            else:
                search_text = "NO LINE DETECTED"
                color = (0, 0, 255)  # Red
                
            cv2.putText(vis, search_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            return vis

        # Draw centerline points
        if result.centerline_points:
            for i, (x, y) in enumerate(result.centerline_points):
                cv2.circle(vis, (x, y), 5, (0, 255, 0), -1)
                
            # Connect points with line
            for i in range(len(result.centerline_points) - 1):
                pt1 = result.centerline_points[i]
                pt2 = result.centerline_points[i + 1]
                cv2.line(vis, pt1, pt2, (255, 0, 255), 2)

        # Draw bottom center point (target)
        cv2.circle(vis, (int(result.line_center_x), int(result.line_center_y)),
                  10, (0, 0, 255), -1)

        # Draw error line (from robot center to detected line)
        cv2.line(vis, 
                (center_x, int(result.line_center_y)),
                (int(result.line_center_x), int(result.line_center_y)),
                (255, 0, 0), 3)

        # Draw info
        info = [
            f"Position Error: {result.position_error:+.3f} ({result.position_error_pixels:+.0f}px)",
            f"Heading Error: {result.heading_error_degrees:+.1f} deg",
            f"Confidence: {result.confidence:.0%}",
        ]
        
        for i, text in enumerate(info):
            cv2.putText(vis, text, (10, 30 + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return vis

    def reset(self):
        """Reset all state including recovery mode."""
        self._prev_result = None
        self._frames_lost = 0
        self._search_direction = 0
        self._last_known_position = 0.0
        logger.info("Line detector reset")
