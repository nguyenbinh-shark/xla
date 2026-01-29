"""
Depth Camera Calibration Tool.
Calibrate RealSense depth accuracy by comparing with real measurements.

Features:
- Click to measure depth at any point
- Enter actual distance to calculate correction factor
- Save/load calibration to config

Usage:
    python tools/calibration/depth_calibration.py
"""

import cv2
import numpy as np
import sys
import json
from pathlib import Path
from datetime import datetime

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception.camera import RealSenseCamera
from src.perception.depth_estimator import DepthEstimator
from src.core import config


class DepthCalibrationTool:
    """Interactive tool for depth camera calibration."""
    
    WINDOW_NAME = "Depth Calibration"
    
    def __init__(self):
        self.camera = RealSenseCamera()
        self.depth_estimator = DepthEstimator()
        self.click_point = None
        self.click_depth = None
        
        # Calibration data
        self.calibration_points = []  # [(measured, actual), ...]
        self.correction_factor = 1.0
        self.depth_offset = 0.0
        
        # Input mode
        self.input_mode = False
        self.input_buffer = ""
    
    def _mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks."""
        if event == cv2.EVENT_LBUTTONDOWN and not self.input_mode:
            self.click_point = (x, y)
            depth_frame = param
            if depth_frame is not None:
                h, w = depth_frame.shape[:2]
                x = max(2, min(w - 3, x))
                y = max(2, min(h - 3, y))
                # Get median of 5x5 region
                region = depth_frame[y-2:y+3, x-2:x+3]
                valid = region[region > 0.1]
                self.click_depth = float(np.median(valid)) if len(valid) > 0 else None
    
    def add_calibration_point(self, measured: float, actual: float):
        """Add calibration point and recalculate correction."""
        if measured > 0 and actual > 0:
            self.calibration_points.append((measured, actual))
            self._calculate_correction()
            
    def _calculate_correction(self):
        """Calculate correction factor using linear regression."""
        if len(self.calibration_points) < 1:
            return
            
        measured = np.array([p[0] for p in self.calibration_points])
        actual = np.array([p[1] for p in self.calibration_points])
        
        if len(self.calibration_points) == 1:
            self.correction_factor = actual[0] / measured[0]
            self.depth_offset = 0.0
        else:
            # Linear regression: actual = factor * measured + offset
            A = np.vstack([measured, np.ones(len(measured))]).T
            result = np.linalg.lstsq(A, actual, rcond=None)
            self.correction_factor = result[0][0]
            self.depth_offset = result[0][1]
            
    def apply_correction(self, depth: float) -> float:
        """Apply correction to depth value."""
        if depth <= 0:
            return depth
        return depth * self.correction_factor + self.depth_offset
        
    def save_calibration(self):
        """Save calibration to file."""
        calib_file = PROJECT_ROOT / "data" / "calibration" / "depth_calibration.json"
        calib_file.parent.mkdir(parents=True, exist_ok=True)
        
        data = {
            "correction_factor": self.correction_factor,
            "depth_offset": self.depth_offset,
            "calibration_points": self.calibration_points,
            "timestamp": datetime.now().isoformat()
        }
        
        with open(calib_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"\n✅ Saved: factor={self.correction_factor:.4f}, offset={self.depth_offset:.4f}m")
        
    def load_calibration(self):
        """Load calibration from file."""
        calib_file = PROJECT_ROOT / "data" / "calibration" / "depth_calibration.json"
        if calib_file.exists():
            with open(calib_file, 'r') as f:
                data = json.load(f)
            self.correction_factor = data.get("correction_factor", 1.0)
            self.depth_offset = data.get("depth_offset", 0.0)
            self.calibration_points = data.get("calibration_points", [])
            print(f"✅ Loaded: factor={self.correction_factor:.4f}, offset={self.depth_offset:.4f}m")
            return True
        return False

    def _create_panel(self, h: int) -> np.ndarray:
        """Create info panel on right side."""
        panel_w = 300
        panel = np.zeros((h, panel_w, 3), dtype=np.uint8)
        panel[:] = (40, 40, 40)
        
        # Title
        cv2.putText(panel, "DEPTH CALIBRATION", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.line(panel, (10, 35), (panel_w - 10, 35), (100, 100, 100), 1)
        
        y = 55
        
        # Instructions
        cv2.putText(panel, "HOW TO CALIBRATE:", (10, y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        y += 18
        for instr in ["1. Place object at known distance",
                      "2. Click on object",
                      "3. Press [A] to add point",
                      "4. Enter actual distance",
                      "5. Repeat at 3-5 distances"]:
            cv2.putText(panel, instr, (10, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1)
            y += 14
            
        y += 10
        cv2.line(panel, (10, y), (panel_w - 10, y), (100, 100, 100), 1)
        y += 18
        
        # Current measurement
        cv2.putText(panel, "CURRENT:", (10, y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        y += 18
        
        if self.click_point and self.click_depth:
            cv2.putText(panel, f"Position: ({self.click_point[0]}, {self.click_point[1]})", 
                       (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
            y += 16
            cv2.putText(panel, f"Measured: {self.click_depth:.3f} m", 
                       (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)
            y += 18
            corrected = self.apply_correction(self.click_depth)
            cv2.putText(panel, f"Corrected: {corrected:.3f} m", 
                       (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 200, 255), 1)
            y += 16
        else:
            cv2.putText(panel, "Click to measure", (10, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
            y += 16
            
        y += 10
        cv2.line(panel, (10, y), (panel_w - 10, y), (100, 100, 100), 1)
        y += 18
        
        # Calibration results
        cv2.putText(panel, "CALIBRATION:", (10, y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        y += 18
        cv2.putText(panel, f"Points: {len(self.calibration_points)}", 
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
        y += 16
        
        # Draw factor with bar
        cv2.putText(panel, f"Factor: {self.correction_factor:.4f}", 
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)
        bar_x = 150
        cv2.rectangle(panel, (bar_x, y-10), (bar_x + 100, y + 2), (60, 60, 60), -1)
        fill = int(100 * min(2.0, self.correction_factor) / 2.0)
        cv2.rectangle(panel, (bar_x, y-10), (bar_x + fill, y + 2), (100, 255, 100), -1)
        y += 18
        
        cv2.putText(panel, f"Offset: {self.depth_offset:+.4f} m", 
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 200, 255), 1)
        y += 20
        
        # Calibration points
        if self.calibration_points:
            cv2.putText(panel, "Points (Meas -> Actual):", (10, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.32, (150, 150, 150), 1)
            y += 14
            for i, (m, a) in enumerate(self.calibration_points[-6:]):
                error = abs(self.apply_correction(m) - a)
                color = (0, 255, 0) if error < 0.05 else (0, 255, 255) if error < 0.1 else (0, 0, 255)
                cv2.putText(panel, f"  {i+1}. {m:.2f}m -> {a:.2f}m (e:{error:.2f})", 
                           (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
                y += 13
                
        # Input mode overlay
        if self.input_mode:
            y = h - 80
            cv2.rectangle(panel, (5, y - 5), (panel_w - 5, y + 40), (80, 80, 80), -1)
            cv2.rectangle(panel, (5, y - 5), (panel_w - 5, y + 40), (0, 255, 255), 1)
            cv2.putText(panel, "Enter actual distance (m):", (10, y + 12), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            cv2.putText(panel, self.input_buffer + "_", (10, y + 32), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Controls
        y = h - 35
        cv2.line(panel, (10, y), (panel_w - 10, y), (100, 100, 100), 1)
        y += 14
        cv2.putText(panel, "[A]dd [S]ave [L]oad [C]lear [D]epth [Q]uit", (5, y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.28, (150, 150, 150), 1)
        
        return panel
    
    def run(self):
        """Run depth calibration tool."""
        print("=" * 60)
        print("  DEPTH CALIBRATION TOOL")
        print("=" * 60)
        print("Controls:")
        print("  Click - Measure depth at point")
        print("  A     - Add calibration point (enter actual distance)")
        print("  S     - Save calibration")
        print("  L     - Load calibration") 
        print("  C     - Clear all points")
        print("  D     - Toggle depth colormap")
        print("  Q     - Quit")
        print("=" * 60)
        
        if not self.camera.start():
            print("ERROR: Failed to start camera!")
            return
        
        # Try load existing
        self.load_calibration()
        
        cv2.namedWindow(self.WINDOW_NAME)
        show_depth_map = False
        
        try:
            while True:
                color_frame, depth_frame = self.camera.get_frames()
                
                if color_frame is None or depth_frame is None:
                    continue
                    
                h, w = color_frame.shape[:2]
                
                # Set mouse callback with depth frame
                cv2.setMouseCallback(self.WINDOW_NAME, self._mouse_callback, depth_frame)
                
                # Base visualization
                if show_depth_map:
                    depth_norm = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
                    vis = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)
                else:
                    vis = color_frame.copy()
                    # Blend with light depth overlay
                    depth_norm = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
                    depth_colored = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)
                    vis = cv2.addWeighted(vis, 0.8, depth_colored, 0.2, 0)
                
                # Draw center crosshair
                cx, cy = w // 2, h // 2
                center_depth = self.depth_estimator.get_depth_at_point(depth_frame, cx, cy)
                cv2.line(vis, (cx - 20, cy), (cx + 20, cy), (255, 255, 0), 1)
                cv2.line(vis, (cx, cy - 20), (cx, cy + 20), (255, 255, 0), 1)
                if center_depth > 0:
                    corrected = self.apply_correction(center_depth)
                    cv2.putText(vis, f"Center: {center_depth:.2f}m -> {corrected:.2f}m", (10, 25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # Draw click point
                if self.click_point and self.click_depth:
                    cv2.circle(vis, self.click_point, 15, (0, 255, 0), 2)
                    cv2.drawMarker(vis, self.click_point, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
                    
                    corrected = self.apply_correction(self.click_depth)
                    cv2.putText(vis, f"Raw: {self.click_depth:.3f}m", 
                               (self.click_point[0] + 20, self.click_point[1] - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                    cv2.putText(vis, f"Corrected: {corrected:.3f}m", 
                               (self.click_point[0] + 20, self.click_point[1] + 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
                
                # Create and attach panel
                panel = self._create_panel(h)
                vis = np.hstack([vis, panel])
                
                cv2.imshow(self.WINDOW_NAME, vis)
                
                key = cv2.waitKey(1) & 0xFF
                
                # Handle input mode
                if self.input_mode:
                    if key == 13:  # Enter
                        try:
                            actual = float(self.input_buffer)
                            if actual > 0 and self.click_depth:
                                self.add_calibration_point(self.click_depth, actual)
                                print(f"✅ Added: {self.click_depth:.3f}m -> {actual:.3f}m")
                        except ValueError:
                            print("❌ Invalid number")
                        self.input_mode = False
                        self.input_buffer = ""
                    elif key == 27:  # Escape
                        self.input_mode = False
                        self.input_buffer = ""
                    elif key == 8 or key == 127:  # Backspace
                        self.input_buffer = self.input_buffer[:-1]
                    elif key < 128 and chr(key) in '0123456789.':
                        self.input_buffer += chr(key)
                else:
                    if key == ord('q'):
                        break
                    elif key == ord('a') and self.click_depth:
                        self.input_mode = True
                        self.input_buffer = ""
                    elif key == ord('s'):
                        self.save_calibration()
                    elif key == ord('l'):
                        self.load_calibration()
                    elif key == ord('c'):
                        self.calibration_points = []
                        self.correction_factor = 1.0
                        self.depth_offset = 0.0
                        print("🗑️ Cleared calibration")
                    elif key == ord('d'):
                        show_depth_map = not show_depth_map
        
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()


def main():
    # Load config
    config_file = PROJECT_ROOT / "configs" / "default_config.yaml"
    try:
        config.load_config(str(config_file))
    except:
        pass
        
    tool = DepthCalibrationTool()
    tool.run()


if __name__ == "__main__":
    main()
