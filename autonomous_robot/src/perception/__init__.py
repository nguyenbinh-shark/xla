"""
Perception Module for Autonomous Robot.
Contains camera, line detection, object detection, and depth estimation.
"""

from .camera import RealSenseCamera
from .simple_line_detector import SimpleLineDetector, LineDetectionResult
from .object_detector import ObjectDetector, ObjectDetectionResult, DetectedObject
from .depth_estimator import DepthEstimator, BoxDepthResult, measure_distance_from_detection
from .terrain_analyzer import TerrainAnalyzer, TerrainConfig, TerrainAnalysisResult, ClearanceAction

__all__ = [
    'RealSenseCamera',
    'SimpleLineDetector', 'LineDetectionResult',
    'ObjectDetector', 'ObjectDetectionResult', 'DetectedObject',
    'DepthEstimator', 'BoxDepthResult', 'measure_distance_from_detection',
    'TerrainAnalyzer', 'TerrainConfig', 'TerrainAnalysisResult', 'ClearanceAction',
]
