#!/usr/bin/env python3
"""
Test Simple Line Detector với hiển thị trực quan.

Usage:
    python tools/testing/test_line_detector.py
"""

import cv2
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception import RealSenseCamera, SimpleLineDetector
from src.core import config


def nothing(x):
    """Callback for trackbar."""
    pass


def main():
    # Auto-load default config
    config_file = PROJECT_ROOT / "configs" / "default_config.yaml"
    print(f"Auto-loading config: {config_file}")
    config.load_config(str(config_file))
    
    print("=" * 60)
    print("  TEST: SIMPLE LINE DETECTOR (Centerline Method)")
    print("=" * 60)
    print("Controls:")
    print("  Q - Quit")
    print("  S - Save frame")
    print("  R - Reset detector")
    print("  Trackbar - Adjust camera offset (camera vs robot center)")
    print("=" * 60)
    
    camera = RealSenseCamera()
    detector = SimpleLineDetector()
    
    if not camera.start():
        print("ERROR: Cannot start camera!")
        return
    
    # Create window and trackbar for camera offset
    cv2.namedWindow("Line Detector Test")
    
    # Offset range: -200 to +200 pixels (trackbar shows 0-400, we subtract 200)
    initial_offset = config.CAMERA_OFFSET_X + 200  # Convert to trackbar value
    cv2.createTrackbar("Offset X", "Line Detector Test", initial_offset, 400, nothing)
    
    frame_count = 0
    
    try:
        while True:
            color_frame, _ = camera.get_frames()
            
            if color_frame is None:
                continue
            
            frame_count += 1
            
            # Get offset from trackbar and update config
            offset_trackbar = cv2.getTrackbarPos("Offset X", "Line Detector Test")
            config.CAMERA_OFFSET_X = offset_trackbar - 200  # Convert to actual offset
            # Detect line
            result = detector.detect(color_frame)
            # Visualize
            vis = detector.visualize(color_frame, result)
            # Add info panel
            h, w = vis.shape[:2]
            # Status color
            status_color = (0, 255, 0) if result.line_detected else (0, 0, 255)
            status_text = "LINE DETECTED" if result.line_detected else "NO LINE"
            
            # Draw info
            info = [
                f"Frame: {frame_count}",
                f"Status: {status_text}",
                f"Offset X: {config.CAMERA_OFFSET_X:+d} px",
                f"Position Error: {result.position_error:+.3f} ({result.position_error_pixels:+.0f}px)",
                f"Heading Error: {result.heading_error_degrees:+.1f} deg",
                f"Confidence: {result.confidence:.0%}",
            ]
            
            y = 30
            for i, text in enumerate(info):
                color = status_color if i == 1 else (255, 255, 255)
                cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                y += 25
            
            # UART preview
            if result.line_detected:
                p_val = int(result.position_error * 1000)
                a_val = int(result.heading_error_degrees * 10)
                uart_text = f"UART: P{p_val:+05d} A{a_val:+04d}"
                cv2.putText(vis, uart_text, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.imshow("Line Detector Test", vis)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"line_test_{frame_count}.jpg"
                cv2.imwrite(filename, vis)
                print(f"Saved: {filename}")
            elif key == ord('r'):
                detector = SimpleLineDetector()
                print("Detector reset!")
            elif key == ord('p'):
                # Print current offset for copying to config
                print(f"\n# Copy to config.py:")
                print(f"CAMERA_OFFSET_X = {config.CAMERA_OFFSET_X}")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print(f"\n# Final offset value:")
        print(f"CAMERA_OFFSET_X = {config.CAMERA_OFFSET_X}")
        print("Test completed.")


if __name__ == "__main__":
    main()
