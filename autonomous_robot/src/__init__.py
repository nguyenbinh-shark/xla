"""
Autonomous Robot Source Package.
Main modules for perception, control, and communication.
"""
# Core configuration
from .core import config
from .core.config import (
    load_config,
    validate_config,
    print_config,
    PROJECT_ROOT,
    CONFIG_DIR,
    DATA_DIR,
    MODELS_DIR,
)

# Perception modules
from .perception import (
    RealSenseCamera,
    SimpleLineDetector,
    LineDetectionResult,
    ObjectDetector,
    ObjectDetectionResult,
    DepthEstimator,
    BoxDepthResult,
)

# Control modules
from .control import (
    MotionController,
    MotionCommand,
)

# Communication modules
from .communication import (
    UARTController,
    MockUARTController,
)

__all__ = [
    # Config
    'config', 'load_config', 'validate_config', 'print_config',
    'PROJECT_ROOT', 'CONFIG_DIR', 'DATA_DIR', 'MODELS_DIR',
    
    # Perception
    'RealSenseCamera', 'SimpleLineDetector', 'LineDetectionResult',
    'ObjectDetector', 'ObjectDetectionResult',
    'DepthEstimator', 'BoxDepthResult',
    
    # Control
    'MotionController', 'MotionCommand',
    
    # Communication
    'UARTController', 'MockUARTController',
]
