"""
Intel RealSense D435i Camera Interface Module.
Handles RGB and Depth stream acquisition with proper alignment.
"""

import numpy as np
import pyrealsense2 as rs
import logging
from typing import Optional, Tuple

from src.core import config

logger = logging.getLogger(__name__)


class RealSenseCamera:
    """
    Interface for Intel RealSense D435i camera.
    Provides synchronized RGB and depth frames with alignment.
    """

    def __init__(
        self,
        width: int = None,
        height: int = None,
        fps: int = None
    ):
        """
        Initialize the RealSense camera.

        Args:
            width: Frame width in pixels (default from config)
            height: Frame height in pixels (default from config)
            fps: Frames per second (default from config)
        """
        self.width = width or config.CAMERA_WIDTH
        self.height = height or config.CAMERA_HEIGHT
        self.fps = fps or config.CAMERA_FPS

        self.pipeline: Optional[rs.pipeline] = None
        self.rs_config: Optional[rs.config] = None
        self.align: Optional[rs.align] = None
        self.depth_scale: float = 0.001  # Default depth scale
        self.intrinsics = None  # Camera intrinsics

        self._is_running = False
        self._consecutive_failures = 0
        self._max_failures = getattr(config, 'CAMERA_MAX_FAILURES', 5)
        self._frame_timeout_ms = getattr(config, 'CAMERA_FRAME_TIMEOUT_MS', 1500)
        self._warmup_frames = getattr(config, 'CAMERA_WARMUP_FRAMES', 10)
        
        # Post-processing filters to reduce dead zones
        self._depth_filters_enabled = getattr(config, 'DEPTH_FILTERS_ENABLED', True)
        self._decimation_filter: Optional[rs.decimation_filter] = None
        self._depth_to_disparity: Optional[rs.disparity_transform] = None
        self._disparity_to_depth: Optional[rs.disparity_transform] = None
        self._spatial_filter: Optional[rs.spatial_filter] = None
        self._temporal_filter: Optional[rs.temporal_filter] = None
        self._hole_filling_filter: Optional[rs.hole_filling_filter] = None
        self._threshold_filter: Optional[rs.threshold_filter] = None

    def start(self) -> bool:
        """
        Start the camera streams.

        Returns:
            True if successful, False otherwise
        """
        try:
            self.pipeline = rs.pipeline()
            self.rs_config = rs.config()

            # Configure RGB stream - use 30fps max for stability
            actual_fps = min(self.fps, 30)
            
            self.rs_config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.bgr8,
                actual_fps
            )

            # Configure Depth stream
            self.rs_config.enable_stream(
                rs.stream.depth,
                self.width,
                self.height,
                rs.format.z16,
                actual_fps
            )

            # Start pipeline
            profile = self.pipeline.start(self.rs_config)
            
            # Get depth sensor and scale
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            logger.info(f"Depth scale: {self.depth_scale}")

            # Get camera intrinsics
            color_profile = profile.get_stream(rs.stream.color)
            self.intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
            logger.info(f"Camera intrinsics: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}")

            # Create alignment object (align depth to color)
            self.align = rs.align(rs.stream.color)
            
            # Initialize post-processing filters to reduce dead zones
            if self._depth_filters_enabled:
                self._init_depth_filters()

            self._is_running = True
            self._consecutive_failures = 0
            
            # Warm-up: skip first few frames for camera stabilization
            logger.info(f"Warming up camera ({self._warmup_frames} frames)...")
            for _ in range(self._warmup_frames):
                try:
                    self.pipeline.wait_for_frames(timeout_ms=1000)
                except Exception:
                    pass
            
            logger.info("RealSense camera started successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to start RealSense camera: {e}")
            self._is_running = False
            return False
    
    def _init_depth_filters(self) -> None:
        """
        Initialize RealSense post-processing filters to reduce dead zones.
        
        Filter chain:
        1. Decimation: Reduce resolution for faster processing (optional)
        2. Threshold: Filter depth range
        3. Spatial: Edge-preserving smoothing, fills small holes
        4. Temporal: Uses history for stabilization and hole filling
        5. Hole Filling: Fills remaining invalid pixels
        """
        try:
            # Get filter settings from config
            min_dist = getattr(config, 'DEPTH_FILTER_MIN_DISTANCE', 0.15)
            max_dist = getattr(config, 'DEPTH_FILTER_MAX_DISTANCE', 4.0)
            spatial_mag = getattr(config, 'DEPTH_SPATIAL_MAGNITUDE', 2)
            spatial_alpha = getattr(config, 'DEPTH_SPATIAL_ALPHA', 0.5)
            spatial_delta = getattr(config, 'DEPTH_SPATIAL_DELTA', 20)
            temporal_alpha = getattr(config, 'DEPTH_TEMPORAL_ALPHA', 0.4)
            hole_fill_mode = getattr(config, 'DEPTH_HOLE_FILLING_MODE', 1)
            
            # Threshold filter - removes depths outside valid range
            self._threshold_filter = rs.threshold_filter()
            self._threshold_filter.set_option(rs.option.min_distance, min_dist)
            self._threshold_filter.set_option(rs.option.max_distance, max_dist)
            
            # Disparity transform - convert to disparity space for better filtering
            # Spatial/temporal filters work better in disparity space (linear) than depth space (non-linear)
            self._depth_to_disparity = rs.disparity_transform(True)  # True = depth to disparity
            self._disparity_to_depth = rs.disparity_transform(False)  # False = disparity to depth
            
            # Spatial filter - smooth depth, fill small holes
            self._spatial_filter = rs.spatial_filter()
            self._spatial_filter.set_option(rs.option.filter_magnitude, spatial_mag)
            self._spatial_filter.set_option(rs.option.filter_smooth_alpha, spatial_alpha)
            self._spatial_filter.set_option(rs.option.filter_smooth_delta, spatial_delta)
            self._spatial_filter.set_option(rs.option.holes_fill, 2)  # Fill holes mode
            
            # Temporal filter - uses multiple frames to fill holes
            self._temporal_filter = rs.temporal_filter()
            self._temporal_filter.set_option(rs.option.filter_smooth_alpha, temporal_alpha)
            self._temporal_filter.set_option(rs.option.filter_smooth_delta, spatial_delta)
            self._temporal_filter.set_option(rs.option.holes_fill, 3)  # Fill with valid in 8-neighbors
            
            # Hole filling filter - fills remaining holes by interpolation
            self._hole_filling_filter = rs.hole_filling_filter()
            # Mode: 0=fill from left, 1=farthest from around, 2=nearest from around
            self._hole_filling_filter.set_option(rs.option.holes_fill, hole_fill_mode)
            
            logger.info("Depth post-processing filters initialized (reduces dead zones)")
            
        except Exception as e:
            logger.warning(f"Failed to initialize depth filters: {e}")
            self._depth_filters_enabled = False
    
    def _apply_depth_filters(self, depth_frame: rs.depth_frame) -> rs.depth_frame:
        """
        Apply post-processing filter chain to depth frame.
        
        Args:
            depth_frame: Raw depth frame from camera
            
        Returns:
            Filtered depth frame with fewer dead zones
        """
        if not self._depth_filters_enabled:
            return depth_frame
        
        try:
            filtered = depth_frame
            
            # Apply filter chain in order:
            # 1. Threshold (in depth space)
            # 2. Depth → Disparity
            # 3. Spatial (in disparity space - more effective)
            # 4. Temporal (in disparity space - more effective)
            # 5. Disparity → Depth
            # 6. Hole Filling
            
            if self._threshold_filter:
                filtered = self._threshold_filter.process(filtered)
            
            # Convert to disparity space for better filtering
            if self._depth_to_disparity:
                filtered = self._depth_to_disparity.process(filtered)
            
            if self._spatial_filter:
                filtered = self._spatial_filter.process(filtered)
            
            if self._temporal_filter:
                filtered = self._temporal_filter.process(filtered)
            
            # Convert back to depth space
            if self._disparity_to_depth:
                filtered = self._disparity_to_depth.process(filtered)
            
            if self._hole_filling_filter:
                filtered = self._hole_filling_filter.process(filtered)
            
            return filtered.as_depth_frame()
            
        except Exception as e:
            logger.debug(f"Depth filter error: {e}")
            return depth_frame

    def stop(self) -> None:
        """Stop the camera streams."""
        if self.pipeline is not None and self._is_running:
            self.pipeline.stop()
            self._is_running = False
            logger.info("RealSense camera stopped")

    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get synchronized RGB and depth frames.

        Returns:
            Tuple of (color_frame, depth_frame) as numpy arrays.
            depth_frame is in meters.
            Returns (None, None) if frames unavailable.
        """
        if not self._is_running or self.pipeline is None:
            return None, None

        try:
            # Wait for frames with shorter timeout
            frames = self.pipeline.wait_for_frames(timeout_ms=self._frame_timeout_ms)

            # Align depth to color frame
            aligned_frames = self.align.process(frames)

            # Get aligned frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                self._consecutive_failures += 1
                logger.warning(f"Incomplete frame ({self._consecutive_failures}/{self._max_failures})")
                return self._handle_failure()

            # Apply post-processing filters to reduce dead zones
            if self._depth_filters_enabled:
                depth_frame = self._apply_depth_filters(depth_frame)

            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Convert depth to meters
            depth_meters = depth_image.astype(np.float32) * self.depth_scale
            
            # Apply camera intrinsic calibration if enabled
            if config.CAMERA_INTRINSIC_ENABLED and config.CAMERA_MATRIX is not None:
                color_image = self.apply_camera_calibration(color_image)

            # Reset failure counter on success
            self._consecutive_failures = 0
            
            return color_image, depth_meters

        except Exception as e:
            self._consecutive_failures += 1
            logger.warning(f"Frame error ({self._consecutive_failures}/{self._max_failures}): {e}")
            return self._handle_failure()
    
    def _handle_failure(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Handle frame acquisition failure with auto-recovery."""
        if self._consecutive_failures >= self._max_failures:
            logger.warning("Too many failures, attempting camera restart...")
            self._restart()
        return None, None
    
    def _restart(self) -> bool:
        """Restart camera pipeline."""
        try:
            logger.info("Restarting camera...")
            self.stop()
            import time
            time.sleep(0.5)  # Brief pause
            success = self.start()
            if success:
                logger.info("Camera restarted successfully")
                self._consecutive_failures = 0
            return success
        except Exception as e:
            logger.error(f"Failed to restart camera: {e}")
            return False

    def apply_camera_calibration(self, image: np.ndarray) -> np.ndarray:
        """
        Apply camera intrinsic calibration to correct lens distortion.
        
        Args:
            image: Input image
            
        Returns:
            Undistorted image
        """
        if not config.CAMERA_INTRINSIC_ENABLED or config.CAMERA_MATRIX is None:
            return image
        
        try:
            import cv2
            
            h, w = image.shape[:2]
            
            # Get optimal camera matrix for undistortion
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                config.CAMERA_MATRIX, 
                config.DISTORTION_COEFFICIENTS, 
                (w, h), 1, (w, h)
            )
            
            # Undistort image
            undistorted = cv2.undistort(
                image, 
                config.CAMERA_MATRIX, 
                config.DISTORTION_COEFFICIENTS, 
                None, 
                new_camera_matrix
            )
            
            return undistorted
            
        except Exception as e:
            logger.warning(f"Camera calibration failed: {e}")
            return image

    def get_raw_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get raw frames without depth conversion to meters.
        
        Returns:
            Tuple of (color_frame, raw_depth_frame)
        """
        if not self._is_running or self.pipeline is None:
            return None, None

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return None, None

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            return color_image, depth_image

        except Exception as e:
            logger.error(f"Error getting raw frames: {e}")
            return None, None

    def pixel_to_point(
        self, 
        x: int, 
        y: int, 
        depth: float
    ) -> Tuple[float, float, float]:
        """
        Convert pixel coordinates to 3D point using camera intrinsics.
        
        Args:
            x: Pixel x coordinate
            y: Pixel y coordinate
            depth: Depth value in meters
            
        Returns:
            Tuple of (X, Y, Z) in meters, camera coordinate system
        """
        if self.intrinsics is None:
            logger.warning("Camera intrinsics not available")
            return (0.0, 0.0, depth)
        
        point = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [x, y], depth
        )
        return tuple(point)

    def get_depth_at_point(
        self,
        depth_frame: np.ndarray,
        x: int,
        y: int,
        filter_size: int = None
    ) -> float:
        """
        Get depth value at a specific point with median filtering.

        Args:
            depth_frame: Depth frame in meters
            x: X coordinate
            y: Y coordinate
            filter_size: Size of median filter kernel

        Returns:
            Depth value in meters, or -1 if invalid
        """
        if depth_frame is None:
            return -1.0

        filter_size = filter_size or config.DEPTH_MEDIAN_FILTER_SIZE

        # Ensure coordinates are within bounds
        h, w = depth_frame.shape[:2]
        half_size = filter_size // 2

        x_min = max(0, x - half_size)
        x_max = min(w, x + half_size + 1)
        y_min = max(0, y - half_size)
        y_max = min(h, y + half_size + 1)

        # Extract region around point
        region = depth_frame[y_min:y_max, x_min:x_max]

        if region.size == 0:
            return -1.0

        # Apply median filter
        valid_depths = region[
            (region > config.DEPTH_MIN_VALID) & 
            (region < config.DEPTH_MAX_VALID)
        ]

        if valid_depths.size == 0:
            return -1.0

        return float(np.median(valid_depths))

    @property
    def is_running(self) -> bool:
        """Check if camera is running."""
        return self._is_running

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        return False
