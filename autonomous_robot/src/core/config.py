"""
Autonomous Robot Configuration Module.
Centralized configuration with YAML support and runtime validation.
"""

import os
import yaml
import logging
from pathlib import Path
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Any

logger = logging.getLogger(__name__)

# =============================================================================
# PATH CONFIGURATION
# =============================================================================
# Get project root directory (autonomous_robot/)
PROJECT_ROOT = Path(__file__).parent.parent.parent
CONFIG_DIR = PROJECT_ROOT / "configs"
DATA_DIR = PROJECT_ROOT / "data"
MODELS_DIR = DATA_DIR / "models"
CALIBRATION_DIR = DATA_DIR / "calibration"

# =============================================================================
# CAMERA CONFIGURATION
# =============================================================================
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# Camera offset từ tâm xe (pixels)
# Dương = camera lệch phải so với tâm xe
# Âm = camera lệch trái so với tâm xe
CAMERA_OFFSET_X = 0  # Default centered camera

# Camera frame acquisition settings
CAMERA_FRAME_TIMEOUT_MS = 1500
CAMERA_WARMUP_FRAMES = 10
CAMERA_MAX_FAILURES = 5

# Camera Intrinsic Calibration
CAMERA_INTRINSIC_ENABLED = False
CAMERA_MATRIX = None
DISTORTION_COEFFICIENTS = None
REPROJECTION_ERROR = None

# Region of Interest - FPV Style (Trapezoidal)
ROI_TOP_RATIO = 0.4

# FPV Trapezoid ROI vertices (as ratio of image dimensions)
ROI_TOP_LEFT_X = 0.30
ROI_TOP_RIGHT_X = 0.70
ROI_BOTTOM_LEFT_X = 0.10
ROI_BOTTOM_RIGHT_X = 0.90
ROI_TOP_Y = 0.55
ROI_BOTTOM_Y = 0.95

# =============================================================================
# LANE DETECTION CONFIGURATION
# =============================================================================
# HLS Color Thresholds for White Lane Markings
WHITE_HLS_LOW = (0, 200, 0)
WHITE_HLS_HIGH = (255, 255, 255)

# HLS Color Thresholds for Yellow Lane Markings
YELLOW_HLS_LOW = (15, 100, 100)
YELLOW_HLS_HIGH = (35, 255, 255)

# Threshold for black lanes
BLACK_THRESHOLD = 80

# Morphological Operations
MORPH_KERNEL_SIZE = 3
MORPH_CLOSE_ITERATIONS = 3
MORPH_OPEN_ITERATIONS = 1

# Canny Edge Detection
CANNY_LOW_THRESHOLD = 50
CANNY_HIGH_THRESHOLD = 150

# Hough Transform Parameters
HOUGH_RHO = 1
HOUGH_THETA_DEGREES = 1
HOUGH_THRESHOLD = 25
HOUGH_MIN_LINE_LENGTH = 50
HOUGH_MAX_LINE_GAP = 30

# Lane Clustering Parameters
LEFT_LANE_X_MIN = 0
LEFT_LANE_X_MAX = 200
CENTER_LANE_X_MIN = 200
CENTER_LANE_X_MAX = 440
RIGHT_LANE_X_MIN = 440
RIGHT_LANE_X_MAX = 640

# Polynomial Fitting
POLY_ORDER = 2
MIN_POINTS_FOR_FIT = 10

# Look-ahead Distance
LOOK_AHEAD_DISTANCE = 100

# Lane Detection Confidence
MIN_LANE_CONFIDENCE = 0.5

# =============================================================================
# OBJECT DETECTION CONFIGURATION
# =============================================================================
YOLO_MODEL_PATH = str(MODELS_DIR / "yolov8n.pt")
YOLO_CONFIDENCE_THRESHOLD = 0.3
YOLO_NMS_THRESHOLD = 0.4

# Classes to detect (COCO dataset indices)
# Set to None to detect all classes
DETECT_CLASSES = None  # Detect all COCO classes

# =============================================================================
# DEPTH ESTIMATION CONFIGURATION
# =============================================================================
DEPTH_MEDIAN_FILTER_SIZE = 3
DEPTH_MIN_VALID = 0.1
DEPTH_MAX_VALID = 10.0

# RealSense Depth Post-Processing Filters
# Enable to significantly reduce dead zones in depth map
DEPTH_FILTERS_ENABLED = True
DEPTH_FILTER_MIN_DISTANCE = 0.15  # meters
DEPTH_FILTER_MAX_DISTANCE = 10.0  # meters (D435i max range)
DEPTH_SPATIAL_MAGNITUDE = 2       # Filter iterations (1-5)
DEPTH_SPATIAL_ALPHA = 0.5         # Smoothing factor (0.25-1.0)
DEPTH_SPATIAL_DELTA = 20          # Edge threshold (1-50)
DEPTH_TEMPORAL_ALPHA = 0.4        # Temporal smoothing (0.0-1.0)
DEPTH_HOLE_FILLING_MODE = 1       # 0=left, 1=farthest, 2=nearest

# Depth Calibration
DEPTH_CORRECTION_FACTOR = 1.0
DEPTH_OFFSET = 0.0
DEPTH_CALIBRATION_ENABLED = False

# =============================================================================
# OBSTACLE DETECTION CONFIGURATION
# =============================================================================
D_SAFE = 2.0
D_EMERGENCY = 0.5

# Obstacle lane assignment
OBSTACLE_LEFT_THRESHOLD = CAMERA_WIDTH / 3
OBSTACLE_RIGHT_THRESHOLD = 2 * CAMERA_WIDTH / 3

# =============================================================================
# STATE MACHINE CONFIGURATION
# =============================================================================
LANE_TRANSITION_HYSTERESIS = 5
OBSTACLE_CLEAR_FRAMES = 10

# =============================================================================
# MOTION CONTROL CONFIGURATION
# =============================================================================
# PID Gains
PID_KP = 0.005
PID_KI = 0.0001
PID_KD = 0.002

# PID Output Limits
PID_OUTPUT_MIN = -1.0
PID_OUTPUT_MAX = 1.0
PID_INTEGRAL_LIMIT = 100.0

# Speed Parameters
SPEED_MAX = 0.8
SPEED_MIN = 0.2
SPEED_NORMAL = 0.6
SPEED_SLOW = 0.3

# Curvature thresholds
CURVATURE_HIGH_THRESHOLD = 0.01
CURVATURE_LOW_THRESHOLD = 0.001

# Rate Limiting
MAX_VELOCITY_CHANGE_RATE = 0.1
MAX_YAW_CHANGE_RATE = 0.2

# =============================================================================
# UART CONFIGURATION
# =============================================================================
UART_PORT = "/dev/ttyACM0"
UART_BAUDRATE = 115200
UART_BYTESIZE = 8
UART_PARITY = 'N'
UART_STOPBITS = 1
UART_TIMEOUT = 0.1
UART_COMMAND_RATE_HZ = 10

# =============================================================================
# ROBOT PHYSICAL PARAMETERS
# =============================================================================
LEG_HEIGHT = 0.15  # meters

# =============================================================================
# SYSTEM SETTINGS
# =============================================================================
MAIN_LOOP_RATE_HZ = 30
LOG_LEVEL = "INFO"


# =============================================================================
# CONFIGURATION LOADER
# =============================================================================
class ConfigLoader:
    """Load and validate configuration from YAML files."""
    
    _instance = None
    _config: Dict[str, Any] = {}
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    @classmethod
    def load(cls, config_path: Optional[str] = None) -> Dict[str, Any]:
        """
        Load configuration from YAML file.
        
        Args:
            config_path: Path to config file. If None, uses default.
            
        Returns:
            Dictionary with configuration values
        """
        if config_path is None:
            config_path = CONFIG_DIR / "default_config.yaml"
        
        config_path = Path(config_path)
        
        if not config_path.exists():
            logger.warning(f"Config file not found: {config_path}, using defaults")
            return {}
        
        try:
            with open(config_path, 'r') as f:
                cls._config = yaml.safe_load(f) or {}
            logger.info(f"Loaded configuration from {config_path}")
            return cls._config
        except Exception as e:
            logger.error(f"Failed to load config: {e}")
            return {}
    
    @classmethod
    def get(cls, key: str, default: Any = None) -> Any:
        """Get configuration value by dot-notation key."""
        keys = key.split('.')
        value = cls._config
        
        for k in keys:
            if isinstance(value, dict):
                value = value.get(k)
            else:
                return default
            if value is None:
                return default
        
        return value


def load_config(config_path: Optional[str] = None) -> None:
    """
    Load configuration and update global variables.
    
    Args:
        config_path: Optional path to YAML config file
    """
    global CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, CAMERA_OFFSET_X
    global CAMERA_INTRINSIC_ENABLED, CAMERA_MATRIX, DISTORTION_COEFFICIENTS, REPROJECTION_ERROR
    global ROI_TOP_LEFT_X, ROI_TOP_RIGHT_X, ROI_BOTTOM_LEFT_X, ROI_BOTTOM_RIGHT_X
    global ROI_TOP_Y, ROI_BOTTOM_Y
    global DEPTH_MEDIAN_FILTER_SIZE, DEPTH_MIN_VALID, DEPTH_MAX_VALID
    global DEPTH_CORRECTION_FACTOR, DEPTH_OFFSET, DEPTH_CALIBRATION_ENABLED
    global YOLO_MODEL_PATH, YOLO_CONFIDENCE_THRESHOLD, YOLO_NMS_THRESHOLD
    global D_SAFE, D_EMERGENCY
    global PID_KP, PID_KI, PID_KD
    global SPEED_MAX, SPEED_MIN, SPEED_NORMAL, SPEED_SLOW
    global UART_PORT, UART_BAUDRATE
    global MAIN_LOOP_RATE_HZ, LOG_LEVEL, LEG_HEIGHT
    global MORPH_KERNEL_SIZE, MORPH_CLOSE_ITERATIONS, MORPH_OPEN_ITERATIONS
    global CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD, BLACK_THRESHOLD
    global HOUGH_RHO, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP
    
    config = ConfigLoader.load(config_path)
    
    if not config:
        return
    
    # Camera
    camera = config.get('camera', {})
    CAMERA_WIDTH = camera.get('width', CAMERA_WIDTH)
    CAMERA_HEIGHT = camera.get('height', CAMERA_HEIGHT)
    CAMERA_FPS = camera.get('fps', CAMERA_FPS)
    CAMERA_OFFSET_X = camera.get('offset_x', CAMERA_OFFSET_X)
    
    # Camera intrinsic calibration
    intrinsic = camera.get('intrinsic_calibration', {})
    CAMERA_INTRINSIC_ENABLED = intrinsic.get('enabled', CAMERA_INTRINSIC_ENABLED)
    if intrinsic.get('camera_matrix'):
        CAMERA_MATRIX = np.array(intrinsic['camera_matrix'])
    if intrinsic.get('distortion_coefficients'):
        DISTORTION_COEFFICIENTS = np.array(intrinsic['distortion_coefficients'])
    REPROJECTION_ERROR = intrinsic.get('reprojection_error', REPROJECTION_ERROR)
    
    # ROI
    roi = config.get('roi', {})
    ROI_TOP_LEFT_X = roi.get('top_left_x', ROI_TOP_LEFT_X)
    ROI_TOP_RIGHT_X = roi.get('top_right_x', ROI_TOP_RIGHT_X)
    ROI_BOTTOM_LEFT_X = roi.get('bottom_left_x', ROI_BOTTOM_LEFT_X)
    ROI_BOTTOM_RIGHT_X = roi.get('bottom_right_x', ROI_BOTTOM_RIGHT_X)
    ROI_TOP_Y = roi.get('top_y', ROI_TOP_Y)
    ROI_BOTTOM_Y = roi.get('bottom_y', ROI_BOTTOM_Y)
    
    # Depth estimation
    depth = config.get('depth', {})
    DEPTH_MEDIAN_FILTER_SIZE = depth.get('median_filter_size', DEPTH_MEDIAN_FILTER_SIZE)
    DEPTH_MIN_VALID = depth.get('min_valid', DEPTH_MIN_VALID)
    DEPTH_MAX_VALID = depth.get('max_valid', DEPTH_MAX_VALID)
    
    # Depth calibration
    depth_calib = depth.get('calibration', {})
    DEPTH_CORRECTION_FACTOR = depth_calib.get('correction_factor', DEPTH_CORRECTION_FACTOR)
    DEPTH_OFFSET = depth_calib.get('offset', DEPTH_OFFSET)
    DEPTH_CALIBRATION_ENABLED = depth_calib.get('enabled', DEPTH_CALIBRATION_ENABLED)
    
    # Object detection
    obj_det = config.get('object_detection', {})
    if obj_det.get('model_path'):
        YOLO_MODEL_PATH = obj_det['model_path']
    YOLO_CONFIDENCE_THRESHOLD = obj_det.get('confidence_threshold', YOLO_CONFIDENCE_THRESHOLD)
    YOLO_NMS_THRESHOLD = obj_det.get('nms_threshold', YOLO_NMS_THRESHOLD)
    
    # Obstacle
    obstacle = config.get('obstacle', {})
    D_SAFE = obstacle.get('d_safe', D_SAFE)
    D_EMERGENCY = obstacle.get('d_emergency', D_EMERGENCY)
    
    # Motion control
    motion = config.get('motion_control', {})
    pid = motion.get('pid', {})
    PID_KP = pid.get('kp', PID_KP)
    PID_KI = pid.get('ki', PID_KI)
    PID_KD = pid.get('kd', PID_KD)
    
    speed = motion.get('speed', {})
    SPEED_MAX = speed.get('max', SPEED_MAX)
    SPEED_MIN = speed.get('min', SPEED_MIN)
    SPEED_NORMAL = speed.get('normal', SPEED_NORMAL)
    SPEED_SLOW = speed.get('slow', SPEED_SLOW)
    
    # UART
    uart = config.get('uart', {})
    UART_PORT = uart.get('port', UART_PORT)
    UART_BAUDRATE = uart.get('baudrate', UART_BAUDRATE)
    
    # Lane detection
    lane = config.get('lane_detection', {})
    BLACK_THRESHOLD = lane.get('black_threshold', BLACK_THRESHOLD)
    MORPH_KERNEL_SIZE = lane.get('morph_kernel_size', MORPH_KERNEL_SIZE)
    MORPH_CLOSE_ITERATIONS = lane.get('morph_close_iterations', MORPH_CLOSE_ITERATIONS)
    MORPH_OPEN_ITERATIONS = lane.get('morph_open_iterations', MORPH_OPEN_ITERATIONS)
    CANNY_LOW_THRESHOLD = lane.get('canny_low', CANNY_LOW_THRESHOLD)
    CANNY_HIGH_THRESHOLD = lane.get('canny_high', CANNY_HIGH_THRESHOLD)
    HOUGH_RHO = lane.get('hough_rho', HOUGH_RHO)
    HOUGH_THRESHOLD = lane.get('hough_threshold', HOUGH_THRESHOLD)
    HOUGH_MIN_LINE_LENGTH = lane.get('hough_min_line_length', HOUGH_MIN_LINE_LENGTH)
    HOUGH_MAX_LINE_GAP = lane.get('hough_max_line_gap', HOUGH_MAX_LINE_GAP)
    
    # System
    system = config.get('system', {})
    MAIN_LOOP_RATE_HZ = system.get('main_loop_rate_hz', MAIN_LOOP_RATE_HZ)
    LOG_LEVEL = system.get('log_level', LOG_LEVEL)
    
    # Robot
    robot = config.get('robot', {})
    LEG_HEIGHT = robot.get('leg_height', LEG_HEIGHT)
    
    logger.info("Configuration loaded successfully")


def validate_config() -> List[str]:
    """
    Validate current configuration values.
    
    Returns:
        List of validation error messages (empty if valid)
    """
    errors = []
    
    # Camera validation
    if CAMERA_WIDTH <= 0 or CAMERA_HEIGHT <= 0:
        errors.append("Camera dimensions must be positive")
    if CAMERA_FPS <= 0:
        errors.append("Camera FPS must be positive")
    
    # ROI validation
    if not (0 <= ROI_TOP_Y < ROI_BOTTOM_Y <= 1):
        errors.append("ROI Y values must be: 0 <= top_y < bottom_y <= 1")
    
    # Distance validation
    if D_EMERGENCY >= D_SAFE:
        errors.append("D_EMERGENCY must be less than D_SAFE")
    if D_EMERGENCY <= 0:
        errors.append("D_EMERGENCY must be positive")
    
    # PID validation
    if PID_KP < 0 or PID_KI < 0 or PID_KD < 0:
        errors.append("PID gains must be non-negative")
    
    # Speed validation
    if not (0 < SPEED_MIN <= SPEED_SLOW <= SPEED_NORMAL <= SPEED_MAX):
        errors.append("Speed values must be: 0 < min <= slow <= normal <= max")
    
    return errors


def print_config() -> None:
    """Print current configuration to console."""
    print("\n" + "=" * 60)
    print("AUTONOMOUS ROBOT CONFIGURATION")
    print("=" * 60)
    print(f"\n[Camera]")
    print(f"  Resolution: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps")
    print(f"  Offset X: {CAMERA_OFFSET_X} px")
    print(f"\n[ROI Trapezoid]")
    print(f"  Top: Y={ROI_TOP_Y:.0%}, X=[{ROI_TOP_LEFT_X:.0%}-{ROI_TOP_RIGHT_X:.0%}]")
    print(f"  Bottom: Y={ROI_BOTTOM_Y:.0%}, X=[{ROI_BOTTOM_LEFT_X:.0%}-{ROI_BOTTOM_RIGHT_X:.0%}]")
    print(f"\n[Object Detection]")
    print(f"  Model: {YOLO_MODEL_PATH}")
    print(f"  Confidence: {YOLO_CONFIDENCE_THRESHOLD}")
    print(f"\n[Safety]")
    print(f"  Safe Distance: {D_SAFE}m")
    print(f"  Emergency Distance: {D_EMERGENCY}m")
    print(f"\n[Motion Control]")
    print(f"  PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
    print(f"  Speed: {SPEED_MIN}-{SPEED_MAX} m/s")
    print(f"\n[UART]")
    print(f"  Port: {UART_PORT} @ {UART_BAUDRATE}")
    print(f"\n[System]")
    print(f"  Loop Rate: {MAIN_LOOP_RATE_HZ}Hz")
    print(f"  Log Level: {LOG_LEVEL}")
    print("=" * 60 + "\n")
