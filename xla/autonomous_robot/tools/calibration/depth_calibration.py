"""
Depth Camera Calibration Tool.
Test and verify RealSense depth accuracy and range.

Usage:
    python -m tools.calibration.depth_calibration
"""

import cv2
import numpy as np
import sys
from pathlib import Path
from datetime import datetime

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception.camera import RealSenseCamera
from src.perception.depth_estimator import DepthEstimator


class DepthCalibrationTool:
    """Interactive tool for depth camera calibration and testing."""
    
    WINDOW_NAME = "Depth Calibration"
    
    def __init__(self):
        self.camera = RealSenseCamera()
        self.depth_estimator = DepthEstimator()
        self.click_point = None
        self.measurements = []
    
    def _mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_point = (x, y)
    
    def _create_depth_colormap(self, depth_frame: np.ndarray, max_depth: float = 5.0) -> np.ndarray:
        """Create colormap visualization of depth."""
        depth_normalized = (depth_frame / max_depth * 255).clip(0, 255).astype(np.uint8)
        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        return depth_colormap
    
    def _draw_grid_measurements(self, frame: np.ndarray, depth_frame: np.ndarray) -> np.ndarray:
        """Draw depth measurements at grid points."""
        h, w = frame.shape[:2]
        vis = frame.copy()
        
        # 3x3 grid
        for row in range(3):
            for col in range(3):
                x = int(w * (col + 1) / 4)
                y = int(h * (row + 1) / 4)
                
                depth = self.depth_estimator.get_depth_at_point(depth_frame, x, y)
                
                # Draw crosshair
                cv2.line(vis, (x - 10, y), (x + 10, y), (0, 255, 0), 1)
                cv2.line(vis, (x, y - 10), (x, y + 10), (0, 255, 0), 1)
                
                # Draw depth value
                if depth > 0:
                    text = f"{depth:.2f}m"
                    color = (0, 255, 0)
                else:
                    text = "N/A"
                    color = (0, 0, 255)
                
                cv2.putText(vis, text, (x + 5, y - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return vis
    
    def _draw_click_measurement(self, frame: np.ndarray, depth_frame: np.ndarray) -> np.ndarray:
        """Draw measurement at clicked point."""
        if self.click_point is None:
            return frame
        
        vis = frame.copy()
        x, y = self.click_point
        depth = self.depth_estimator.get_depth_at_point(depth_frame, x, y)
        
        # Store measurement
        self.measurements.append({
            'point': (x, y),
            'depth': depth,
            'time': datetime.now().isoformat()
        })
        
        # Draw circle and crosshair
        cv2.circle(vis, (x, y), 20, (0, 0, 255), 2)
        cv2.line(vis, (x - 30, y), (x + 30, y), (0, 0, 255), 2)
        cv2.line(vis, (x, y - 30), (x, y + 30), (0, 0, 255), 2)
        
        # Draw depth
        if depth > 0:
            text = f"DEPTH: {depth:.3f}m ({depth * 100:.1f}cm)"
            color = (0, 255, 255)
        else:
            text = "DEPTH: Invalid"
            color = (0, 0, 255)
        
        cv2.rectangle(vis, (x - 100, y + 25), (x + 150, y + 55), (0, 0, 0), -1)
        cv2.putText(vis, text, (x - 95, y + 45),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return vis
    
    def _draw_center_measurement(self, frame: np.ndarray, depth_frame: np.ndarray) -> np.ndarray:
        """Draw continuous center measurement."""
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        
        depth = self.depth_estimator.get_depth_at_point(depth_frame, cx, cy)
        
        vis = frame.copy()
        
        # Center crosshair
        cv2.line(vis, (cx - 30, cy), (cx + 30, cy), (255, 255, 0), 2)
        cv2.line(vis, (cx, cy - 30), (cx, cy + 30), (255, 255, 0), 2)
        cv2.circle(vis, (cx, cy), 5, (255, 255, 0), -1)
        
        # Depth display
        if depth > 0:
            text = f"Center: {depth:.3f}m"
            color = (0, 255, 0)
        else:
            text = "Center: N/A"
            color = (0, 0, 255)
        
        cv2.rectangle(vis, (10, 10), (250, 45), (0, 0, 0), -1)
        cv2.putText(vis, text, (15, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        return vis
    
    def run(self):
        """Run depth calibration tool."""
        print("=" * 60)
        print("DEPTH CAMERA CALIBRATION TOOL")
        print("=" * 60)
        print("Controls:")
        print("  Left Click - Measure depth at point")
        print("  G - Toggle grid measurements")
        print("  D - Toggle depth colormap view")
        print("  P - Print all measurements")
        print("  C - Clear measurements")
        print("  Q - Quit")
        print("=" * 60)
        
        if not self.camera.start():
            print("ERROR: Failed to start camera!")
            return
        
        cv2.namedWindow(self.WINDOW_NAME)
        cv2.setMouseCallback(self.WINDOW_NAME, self._mouse_callback)
        
        show_grid = False
        show_depth_map = False
        
        try:
            while True:
                color_frame, depth_frame = self.camera.get_frames()
                
                if color_frame is None or depth_frame is None:
                    continue
                
                # Choose base frame
                if show_depth_map:
                    vis = self._create_depth_colormap(depth_frame)
                else:
                    vis = color_frame.copy()
                
                # Draw center measurement
                vis = self._draw_center_measurement(vis, depth_frame)
                
                # Draw grid
                if show_grid:
                    vis = self._draw_grid_measurements(vis, depth_frame)
                
                # Draw click measurement
                vis = self._draw_click_measurement(vis, depth_frame)
                
                # Instructions
                instr = "[G]rid [D]epth [P]rint [C]lear [Q]uit | Click to measure"
                cv2.putText(vis, instr, (10, vis.shape[0] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow(self.WINDOW_NAME, vis)
                
                # Handle keys
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('g'):
                    show_grid = not show_grid
                    print(f"Grid: {'ON' if show_grid else 'OFF'}")
                elif key == ord('d'):
                    show_depth_map = not show_depth_map
                    print(f"Depth map: {'ON' if show_depth_map else 'OFF'}")
                elif key == ord('p'):
                    print("\n=== Measurements ===")
                    for i, m in enumerate(self.measurements):
                        print(f"{i+1}. Point {m['point']}: {m['depth']:.3f}m")
                    print("====================\n")
                elif key == ord('c'):
                    self.measurements.clear()
                    self.click_point = None
                    print("Measurements cleared")
        
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()


def main():
    tool = DepthCalibrationTool()
    tool.run()


if __name__ == "__main__":
    main()
