"""
Interactive Module Test Tool.
Test individual modules with live camera feed.

Usage:
    python -m tools.testing.test_modules_interactive
"""

import cv2
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception import (
    RealSenseCamera,
    SimpleLineDetector,
    ObjectDetector,
    DepthEstimator,
)


class InteractiveModuleTester:
    """Interactive testing for robot modules."""
    
    def __init__(self):
        print("Initializing modules...")
        self.camera = RealSenseCamera()
        self.line_detector = SimpleLineDetector()
        self.object_detector = ObjectDetector()
        self.depth_estimator = DepthEstimator()
        
        self.current_mode = 'line'  # line, object, depth, all
        self.paused = False
    
    def run(self):
        """Run interactive test."""
        print("=" * 60)
        print("INTERACTIVE MODULE TESTER")
        print("=" * 60)
        print("Modes:")
        print("  1 - Line Detection (centerline)")
        print("  2 - Object Detection")
        print("  3 - Depth Estimation")
        print("  4 - All modules combined")
        print("Controls:")
        print("  SPACE - Pause/Resume")
        print("  S - Save current frame")
        print("  Q - Quit")
        print("=" * 60)
        
        if not self.camera.start():
            print("ERROR: Failed to start camera!")
            return
        
        frame_count = 0
        
        try:
            while True:
                if not self.paused:
                    color_frame, depth_frame = self.camera.get_frames()
                    
                    if color_frame is None:
                        continue
                    
                    frame_count += 1
                    
                    # Process based on mode
                    if self.current_mode == 'line':
                        vis = self._test_line_detection(color_frame)
                    elif self.current_mode == 'object':
                        vis = self._test_object_detection(color_frame, depth_frame)
                    elif self.current_mode == 'depth':
                        vis = self._test_depth_estimation(color_frame, depth_frame)
                    else:
                        vis = self._test_all_modules(color_frame, depth_frame)
                    
                    # Add mode indicator with background
                    mode_text = f"Mode: {self.current_mode.upper()} | Frame: {frame_count}"
                    cv2.rectangle(vis, (5, 5), (350, 35), (0, 0, 0), -1)
                    cv2.putText(vis, mode_text, (10, 28),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    
                    cv2.imshow("Module Tester", vis)
                
                # Handle input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('1'):
                    self.current_mode = 'line'
                    print("Mode: Line Detection")
                elif key == ord('2'):
                    self.current_mode = 'object'
                    print("Mode: Object Detection")
                elif key == ord('3'):
                    self.current_mode = 'depth'
                    print("Mode: Depth Estimation")
                elif key == ord('4'):
                    self.current_mode = 'all'
                    print("Mode: All Modules")
                elif key == ord(' '):
                    self.paused = not self.paused
                    print("PAUSED" if self.paused else "RESUMED")
                elif key == ord('s'):
                    filename = f"test_frame_{frame_count}.jpg"
                    cv2.imwrite(filename, vis)
                    print(f"Saved: {filename}")
        
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()
    
    def _test_line_detection(self, frame):
        """Test simple line detection module (centerline method)."""
        result = self.line_detector.detect(frame)
        vis = self.line_detector.visualize(frame, result)
        
        # Add extra info with background
        info = [
            f"Detected: {result.line_detected}",
            f"Position Error: {result.position_error:+.3f}",
            f"Heading Error: {result.heading_error_degrees:+.1f} deg",
            f"Confidence: {result.confidence:.0%}",
        ]
        
        # Draw info panel on the right side
        panel_x = vis.shape[1] - 280
        cv2.rectangle(vis, (panel_x, 45), (vis.shape[1] - 5, 155), (0, 0, 0), -1)
        
        y = 70
        for text in info:
            color = (0, 255, 0) if result.line_detected else (0, 0, 255)
            cv2.putText(vis, text, (panel_x + 10, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            y += 25
        
        return vis
    
    def _test_object_detection(self, frame, depth_frame):
        """Test object detection module."""
        result = self.object_detector.detect(frame, depth_frame)
        vis = self.object_detector.visualize(frame, result)
        
        # Add info
        info = [
            f"Objects: {len(result.objects)}",
            f"Obstacles: {len(result.obstacles)}",
            f"Emergency: {result.emergency_stop}",
        ]
        
        if result.closest_obstacle:
            info.append(f"Closest: {result.closest_obstacle.depth:.2f}m")
        
        # Draw info panel on the right side
        panel_x = vis.shape[1] - 250
        panel_h = 50 + len(info) * 25
        cv2.rectangle(vis, (panel_x, 45), (vis.shape[1] - 5, panel_h + 45), (0, 0, 0), -1)
        
        y = 70
        for text in info:
            cv2.putText(vis, text, (panel_x + 10, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y += 25
        
        return vis
    
    def _test_depth_estimation(self, frame, depth_frame):
        """Test depth estimation module."""
        vis = frame.copy()
        h, w = frame.shape[:2]
        
        # Draw grid of depth measurements
        for row in range(5):
            for col in range(5):
                x = int(w * (col + 1) / 6)
                y = int(h * (row + 1) / 6)
                
                depth = self.depth_estimator.get_depth_at_point(depth_frame, x, y)
                
                # Draw crosshair
                cv2.line(vis, (x - 10, y), (x + 10, y), (0, 255, 0), 1)
                cv2.line(vis, (x, y - 10), (x, y + 10), (0, 255, 0), 1)
                
                # Draw depth with background
                if depth > 0:
                    text = f"{depth:.1f}m"
                    color = (0, 255, 0)
                else:
                    text = "N/A"
                    color = (0, 0, 255)
                
                # Add small background for text
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                cv2.rectangle(vis, (x + 5, y - 15), (x + 10 + tw, y), (0, 0, 0), -1)
                cv2.putText(vis, text, (x + 7, y - 3),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Center depth with background panel
        center_depth = self.depth_estimator.get_depth_at_point(depth_frame, w // 2, h // 2)
        cv2.circle(vis, (w // 2, h // 2), 15, (255, 0, 0), 2)
        
        # Draw center depth info on right side
        panel_x = w - 200
        cv2.rectangle(vis, (panel_x, 45), (w - 5, 80), (0, 0, 0), -1)
        cv2.putText(vis, f"Center: {center_depth:.2f}m", (panel_x + 10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return vis
    
    def _test_all_modules(self, frame, depth_frame):
        """Test all modules combined."""
        # Simple line detection (primary)
        line_result = self.line_detector.detect(frame)
        vis = self.line_detector.visualize(frame, line_result)
        
        # Object detection
        obj_result = self.object_detector.detect(frame, depth_frame)
        vis = self.object_detector.visualize(vis, obj_result)
        # Combined info
        info = [
            f"Line: {line_result.line_detected}",
            f"PosErr: {line_result.position_error:+.3f}",
            f"HeadErr: {line_result.heading_error_degrees:+.1f} deg",
            f"Objects: {len(obj_result.objects)}",
            f"Emergency: {obj_result.emergency_stop}",
        ]
        
        # Draw info panel on the right side
        panel_x = vis.shape[1] - 250
        panel_h = 50 + len(info) * 22
        cv2.rectangle(vis, (panel_x, 45), (vis.shape[1] - 5, panel_h + 45), (0, 0, 0), -1)
        
        y = 68
        for text in info:
            cv2.putText(vis, text, (panel_x + 10, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            y += 22
        
        return vis
    
def main():
    tester = InteractiveModuleTester()
    tester.run()


if __name__ == "__main__":
    main()