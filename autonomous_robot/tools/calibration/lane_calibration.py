"""
Camera Calibration Tool for Lane Detection.
Interactive tool to tune ROI, thresholds, and Hough parameters.

Usage:
    python -m tools.calibration.lane_calibration
    
Controls:
    s - Save current parameters
    c - Capture image
    r - Reset to defaults
    q - Quit
"""

import cv2
import numpy as np
import os
import sys
import json
from datetime import datetime
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.core import config
from src.perception.camera import RealSenseCamera


class LaneCalibrationTool:
    """Interactive tool for calibrating lane detection parameters."""
    
    WINDOW_NAME = "Lane Calibration"
    PARAMS_FILE = PROJECT_ROOT / "data" / "calibration" / "lane_params.json"
    
    def __init__(self):
        self.camera = RealSenseCamera()
        self.params = self._get_default_params()
        self._setup_ui()
        
    def _get_default_params(self) -> dict:
        """Get default parameters from config."""
        return {
            # ROI Shape
            'roi_top_y': int(config.ROI_TOP_Y * 100),
            'roi_bottom_y': int(config.ROI_BOTTOM_Y * 100),
            'roi_top_left_x': int(config.ROI_TOP_LEFT_X * 100),
            'roi_top_right_x': int(config.ROI_TOP_RIGHT_X * 100),
            'roi_bottom_left_x': int(config.ROI_BOTTOM_LEFT_X * 100),
            'roi_bottom_right_x': int(config.ROI_BOTTOM_RIGHT_X * 100),
            # Thresholds
            'black_threshold': 100,
            'morph_kernel': config.MORPH_KERNEL_SIZE,
            # Canny
            'canny_low': config.CANNY_LOW_THRESHOLD,
            'canny_high': config.CANNY_HIGH_THRESHOLD,
            # Hough
            'hough_threshold': config.HOUGH_THRESHOLD,
            'hough_min_length': config.HOUGH_MIN_LINE_LENGTH,
            'hough_max_gap': config.HOUGH_MAX_LINE_GAP,
        }
    
    def _setup_ui(self):
        """Create trackbars for parameter adjustment."""
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW_NAME, 1280, 720)
        
        # ROI controls
        cv2.createTrackbar('ROI Top Y %', self.WINDOW_NAME, 
                          self.params['roi_top_y'], 100, lambda x: None)
        cv2.createTrackbar('ROI Bottom Y %', self.WINDOW_NAME,
                          self.params['roi_bottom_y'], 100, lambda x: None)
        cv2.createTrackbar('Top Left X %', self.WINDOW_NAME,
                          self.params['roi_top_left_x'], 100, lambda x: None)
        cv2.createTrackbar('Top Right X %', self.WINDOW_NAME,
                          self.params['roi_top_right_x'], 100, lambda x: None)
        cv2.createTrackbar('Bot Left X %', self.WINDOW_NAME,
                          self.params['roi_bottom_left_x'], 100, lambda x: None)
        cv2.createTrackbar('Bot Right X %', self.WINDOW_NAME,
                          self.params['roi_bottom_right_x'], 100, lambda x: None)
        
        # Processing controls
        cv2.createTrackbar('Threshold', self.WINDOW_NAME,
                          self.params['black_threshold'], 255, lambda x: None)
        cv2.createTrackbar('Morph Kernel', self.WINDOW_NAME,
                          self.params['morph_kernel'], 15, lambda x: None)
        cv2.createTrackbar('Canny Low', self.WINDOW_NAME,
                          self.params['canny_low'], 255, lambda x: None)
        cv2.createTrackbar('Canny High', self.WINDOW_NAME,
                          self.params['canny_high'], 255, lambda x: None)
        cv2.createTrackbar('Hough Thresh', self.WINDOW_NAME,
                          self.params['hough_threshold'], 100, lambda x: None)
        cv2.createTrackbar('Hough MinLen', self.WINDOW_NAME,
                          self.params['hough_min_length'], 200, lambda x: None)
        cv2.createTrackbar('Hough MaxGap', self.WINDOW_NAME,
                          self.params['hough_max_gap'], 100, lambda x: None)
    
    def _read_trackbars(self) -> dict:
        """Read current trackbar values."""
        return {
            'roi_top_y': cv2.getTrackbarPos('ROI Top Y %', self.WINDOW_NAME),
            'roi_bottom_y': max(1, cv2.getTrackbarPos('ROI Bottom Y %', self.WINDOW_NAME)),
            'roi_top_left_x': cv2.getTrackbarPos('Top Left X %', self.WINDOW_NAME),
            'roi_top_right_x': cv2.getTrackbarPos('Top Right X %', self.WINDOW_NAME),
            'roi_bottom_left_x': cv2.getTrackbarPos('Bot Left X %', self.WINDOW_NAME),
            'roi_bottom_right_x': cv2.getTrackbarPos('Bot Right X %', self.WINDOW_NAME),
            'black_threshold': cv2.getTrackbarPos('Threshold', self.WINDOW_NAME),
            'morph_kernel': max(1, cv2.getTrackbarPos('Morph Kernel', self.WINDOW_NAME)),
            'canny_low': cv2.getTrackbarPos('Canny Low', self.WINDOW_NAME),
            'canny_high': cv2.getTrackbarPos('Canny High', self.WINDOW_NAME),
            'hough_threshold': max(1, cv2.getTrackbarPos('Hough Thresh', self.WINDOW_NAME)),
            'hough_min_length': max(1, cv2.getTrackbarPos('Hough MinLen', self.WINDOW_NAME)),
            'hough_max_gap': max(1, cv2.getTrackbarPos('Hough MaxGap', self.WINDOW_NAME)),
        }
    
    def _get_roi_vertices(self, width: int, height: int, params: dict) -> np.ndarray:
        """Get ROI trapezoid vertices."""
        return np.array([[
            (int(width * params['roi_bottom_left_x'] / 100), 
             int(height * params['roi_bottom_y'] / 100)),
            (int(width * params['roi_bottom_right_x'] / 100), 
             int(height * params['roi_bottom_y'] / 100)),
            (int(width * params['roi_top_right_x'] / 100), 
             int(height * params['roi_top_y'] / 100)),
            (int(width * params['roi_top_left_x'] / 100), 
             int(height * params['roi_top_y'] / 100)),
        ]], dtype=np.int32)
    
    def _process_frame(self, frame: np.ndarray, params: dict) -> dict:
        """Process frame and return intermediate results."""
        h, w = frame.shape[:2]
        results = {'original': frame.copy()}
        
        # Create ROI mask
        vertices = self._get_roi_vertices(w, h, params)
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(roi_mask, vertices, 255)
        
        # Apply ROI
        roi = cv2.bitwise_and(frame, frame, mask=roi_mask)
        results['roi'] = roi
        
        # Draw ROI on original
        frame_with_roi = frame.copy()
        cv2.polylines(frame_with_roi, vertices, True, (0, 255, 0), 2)
        results['frame_with_roi'] = frame_with_roi
        
        # Grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Threshold
        _, binary = cv2.threshold(gray, params['black_threshold'], 255, cv2.THRESH_BINARY_INV)
        binary = cv2.bitwise_and(binary, roi_mask)
        results['binary'] = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        
        # Morphology
        kernel_size = params['morph_kernel'] | 1  # Ensure odd
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        morphed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        morphed = cv2.morphologyEx(morphed, cv2.MORPH_OPEN, kernel, iterations=1)
        results['morphed'] = cv2.cvtColor(morphed, cv2.COLOR_GRAY2BGR)
        
        # Canny
        edges = cv2.Canny(morphed, params['canny_low'], params['canny_high'])
        results['edges'] = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        
        # Hough lines
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=params['hough_threshold'],
            minLineLength=params['hough_min_length'],
            maxLineGap=params['hough_max_gap']
        )
        
        # Draw lines
        lines_frame = frame.copy()
        cv2.polylines(lines_frame, vertices, True, (0, 255, 0), 2)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(lines_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        results['lines'] = lines_frame
        results['line_count'] = len(lines) if lines is not None else 0
        
        return results
    
    def _create_display(self, results: dict, params: dict) -> np.ndarray:
        """Create multi-view display with parameter panel."""
        # Resize all images to same size
        target_h, target_w = 240, 320
        
        imgs = []
        for key in ['frame_with_roi', 'binary', 'edges', 'lines']:
            img = cv2.resize(results[key], (target_w, target_h))
            
            # Add label
            label = {
                'frame_with_roi': 'ROI',
                'binary': 'Threshold',
                'edges': 'Edges',
                'lines': f'Lines ({results["line_count"]})'
            }[key]
            
            cv2.putText(img, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (0, 255, 0), 2)
            imgs.append(img)
        
        # Arrange 2x2
        top_row = np.hstack([imgs[0], imgs[1]])
        bottom_row = np.hstack([imgs[2], imgs[3]])
        image_display = np.vstack([top_row, bottom_row])
        
        # Create parameter panel on the right side
        param_panel = self._create_param_panel(params, image_display.shape[0])
        
        # Combine image and param panel
        display = np.hstack([image_display, param_panel])
        
        # Add instructions at bottom
        instr = "[S]ave [C]apture [R]eset [Q]uit"
        cv2.putText(display, instr, (10, display.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return display
    
    def _create_param_panel(self, params: dict, height: int) -> np.ndarray:
        """Create side panel showing all parameters with visual bars."""
        width = 350
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        panel[:] = (40, 40, 40)
        
        # Title
        cv2.putText(panel, "PARAMETERS", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.line(panel, (10, 35), (width - 10, 35), (100, 100, 100), 1)
        
        y = 55
        bar_x = 150
        bar_w = 150
        bar_h = 12
        
        param_list = [
            ("=== ROI ===", None, None, None),
            ("Top Y", params['roi_top_y'], 100, (100, 255, 100)),
            ("Bottom Y", params['roi_bottom_y'], 100, (100, 255, 100)),
            ("Top Left X", params['roi_top_left_x'], 100, (100, 200, 255)),
            ("Top Right X", params['roi_top_right_x'], 100, (100, 200, 255)),
            ("Bot Left X", params['roi_bottom_left_x'], 100, (255, 200, 100)),
            ("Bot Right X", params['roi_bottom_right_x'], 100, (255, 200, 100)),
            ("=== PROCESS ===", None, None, None),
            ("Threshold", params['black_threshold'], 255, (200, 200, 200)),
            ("Morph Kernel", params['morph_kernel'], 15, (200, 150, 255)),
            ("=== CANNY ===", None, None, None),
            ("Canny Low", params['canny_low'], 255, (255, 150, 150)),
            ("Canny High", params['canny_high'], 255, (255, 100, 100)),
            ("=== HOUGH ===", None, None, None),
            ("Hough Thresh", params['hough_threshold'], 100, (150, 255, 150)),
            ("Hough MinLen", params['hough_min_length'], 200, (150, 255, 200)),
            ("Hough MaxGap", params['hough_max_gap'], 100, (150, 200, 255)),
        ]
        
        for name, value, max_val, color in param_list:
            if value is None:
                # Section header
                y += 5
                cv2.putText(panel, name, (10, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                y += 18
                continue
            
            # Parameter name
            cv2.putText(panel, name, (10, y + 3), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            
            # Value
            cv2.putText(panel, str(value), (bar_x - 35, y + 3), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # Bar background
            cv2.rectangle(panel, (bar_x, y - bar_h//2), 
                         (bar_x + bar_w, y + bar_h//2), (60, 60, 60), -1)
            
            # Bar fill
            fill_w = int(bar_w * value / max_val) if max_val > 0 else 0
            if fill_w > 0:
                cv2.rectangle(panel, (bar_x, y - bar_h//2), 
                             (bar_x + fill_w, y + bar_h//2), color, -1)
            
            # Bar border
            cv2.rectangle(panel, (bar_x, y - bar_h//2), 
                         (bar_x + bar_w, y + bar_h//2), (100, 100, 100), 1)
            
            y += 22
        
        return panel
    
    def save_params(self, params: dict):
        """Save parameters to file."""
        self.PARAMS_FILE.parent.mkdir(parents=True, exist_ok=True)
        
        # Convert to config format
        save_data = {
            'roi': {
                'top_y': params['roi_top_y'] / 100,
                'bottom_y': params['roi_bottom_y'] / 100,
                'top_left_x': params['roi_top_left_x'] / 100,
                'top_right_x': params['roi_top_right_x'] / 100,
                'bottom_left_x': params['roi_bottom_left_x'] / 100,
                'bottom_right_x': params['roi_bottom_right_x'] / 100,
            },
            'lane_detection': {
                'black_threshold': params['black_threshold'],
                'morph_kernel_size': params['morph_kernel'],
                'canny_low': params['canny_low'],
                'canny_high': params['canny_high'],
                'hough_threshold': params['hough_threshold'],
                'hough_min_line_length': params['hough_min_length'],
                'hough_max_line_gap': params['hough_max_gap'],
            },
            'saved_at': datetime.now().isoformat(),
        }
        
        with open(self.PARAMS_FILE, 'w') as f:
            json.dump(save_data, f, indent=2)
        
        print(f"✓ Parameters saved to {self.PARAMS_FILE}")
        
        # Also print config format
        print("\n# Copy to config.py or default_config.yaml:")
        print(f"ROI_TOP_Y = {params['roi_top_y'] / 100}")
        print(f"ROI_BOTTOM_Y = {params['roi_bottom_y'] / 100}")
        print(f"ROI_TOP_LEFT_X = {params['roi_top_left_x'] / 100}")
        print(f"ROI_TOP_RIGHT_X = {params['roi_top_right_x'] / 100}")
        print(f"ROI_BOTTOM_LEFT_X = {params['roi_bottom_left_x'] / 100}")
        print(f"ROI_BOTTOM_RIGHT_X = {params['roi_bottom_right_x'] / 100}")
        print(f"CANNY_LOW_THRESHOLD = {params['canny_low']}")
        print(f"CANNY_HIGH_THRESHOLD = {params['canny_high']}")
        print(f"HOUGH_THRESHOLD = {params['hough_threshold']}")
        print(f"HOUGH_MIN_LINE_LENGTH = {params['hough_min_length']}")
        print(f"HOUGH_MAX_LINE_GAP = {params['hough_max_gap']}")
    
    def capture_image(self, frame: np.ndarray):
        """Save current frame."""
        capture_dir = PROJECT_ROOT / "data" / "calibration" / "captures"
        capture_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = capture_dir / f"capture_{timestamp}.jpg"
        cv2.imwrite(str(filename), frame)
        print(f"✓ Image saved: {filename}")
    
    def run(self):
        """Run calibration tool."""
        print("=" * 60)
        print("LANE DETECTION CALIBRATION TOOL")
        print("=" * 60)
        print("Controls:")
        print("  S - Save parameters")
        print("  C - Capture image")
        print("  R - Reset to defaults")
        print("  Q - Quit")
        print("=" * 60)
        
        if not self.camera.start():
            print("ERROR: Failed to start camera!")
            return
        
        try:
            while True:
                color_frame, _ = self.camera.get_frames()
                
                if color_frame is None:
                    continue
                
                # Read parameters
                params = self._read_trackbars()
                
                # Process frame
                results = self._process_frame(color_frame, params)
                
                # Create display
                display = self._create_display(results, params)
                
                cv2.imshow(self.WINDOW_NAME, display)
                
                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    self.save_params(params)
                elif key == ord('c'):
                    self.capture_image(color_frame)
                elif key == ord('r'):
                    self.params = self._get_default_params()
                    self._setup_ui()
                    print("✓ Reset to defaults")
        
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()

def main():
    tool = LaneCalibrationTool()
    tool.run()

if __name__ == "__main__":
    main()
