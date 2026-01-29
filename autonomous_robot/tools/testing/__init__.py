"""
Testing Tools Package.

Individual module tests with visual display:
    python tools/testing/test_camera.py          - Test camera
    python tools/testing/test_line_detector.py   - Test simple line detector
    python tools/testing/test_object_detector.py - Test object detector
    python tools/testing/test_depth_estimator.py - Test depth estimator
    python tools/testing/test_uart.py            - Test UART controller

Interactive multi-module test:
    python tools/testing/test_modules_interactive.py
"""

from pathlib import Path

TESTS_DIR = Path(__file__).parent
