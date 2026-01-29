#!/usr/bin/env python3
"""
Camera Intrinsic Calibration Tool.
Calibrate camera intrinsics using checkerboard pattern to correct lens distortion.

Usage:
    python -m tools.calibration.camera_calibration
    
Requirements:
    - Print a 9x6 checkerboard pattern (A4 size recommended)
    - Place pattern at various angles and distances
    - Capture 15-20 good images with different poses
    
Controls:
    SPACE - Capture calibration image
    c - Start calibration process
    s - Save calibration results
    q - Quit
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
from src.core import config


class CameraCalibrationTool:
    """Interactive tool for camera intrinsic calibration."""
    
    WINDOW_NAME = "Camera Calibration"
    CALIB_FILE = PROJECT_ROOT / "data" / "calibration" / "camera_intrinsics.json"
    
    def __init__(self):
        self.camera = RealSenseCamera()
        
        # Checkerboard settings (internal corners)
        self.board_size = (9, 6)  # 9x6 internal corners
        self.square_size = 25.0   # Square size in mm
        
        # Calibration data
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane
        self.calibration_images = []
        self.capture_count = 0
        
        # Results
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_error = 0.0
        self.calibrated = False
        
        # Prepare object points
        self._prepare_object_points()
    
    def _prepare_object_points(self):
        """Prepare 3D object points for checkerboard."""
        # 3D points in real world space (Z=0)
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)
        objp *= self.square_size  # Scale by square size
        self.object_points_template = objp
    
    def _detect_checkerboard(self, frame):
        """Detect checkerboard corners in frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, 
            self.board_size, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        if ret:
            # Refine corners to sub-pixel accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        return ret, corners
    
    def _capture_calibration_image(self, frame, corners):
        """Capture frame for calibration."""
        self.obj_points.append(self.object_points_template)
        self.img_points.append(corners)
        self.calibration_images.append(frame.copy())
        self.capture_count += 1
        
        print(f"Captured image {self.capture_count}/20. Move checkerboard to new position.")
        
        if self.capture_count >= 20:
            print("✅ Enough images captured! Press 'C' to start calibration.")
    
    def _calibrate_camera(self):
        """Perform camera calibration."""
        if len(self.obj_points) < 10:
            print("❌ Need at least 10 images for calibration!")
            return False
        
        print(f"📸 Calibrating camera with {len(self.obj_points)} images...")
        
        # Get image size
        h, w = self.calibration_images[0].shape[:2]
        
        # Calibrate camera
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, 
            self.img_points, 
            (w, h), 
            None, 
            None
        )
        
        if ret:
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs
            self.calibrated = True
            
            # Calculate reprojection error
            total_error = 0
            for i in range(len(self.obj_points)):
                img_points_proj, _ = cv2.projectPoints(
                    self.obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
                )
                error = cv2.norm(self.img_points[i], img_points_proj, cv2.NORM_L2) / len(img_points_proj)
                total_error += error
            
            self.calibration_error = total_error / len(self.obj_points)
            
            print("✅ Calibration successful!")
            print(f"📊 Reprojection error: {self.calibration_error:.3f} pixels")
            print(f"📷 Camera matrix:")
            print(f"   fx: {camera_matrix[0,0]:.2f}")
            print(f"   fy: {camera_matrix[1,1]:.2f}")  
            print(f"   cx: {camera_matrix[0,2]:.2f}")
            print(f"   cy: {camera_matrix[1,2]:.2f}")
            print("💾 Press 'S' to save calibration results.")
            
            return True
        else:
            print("❌ Calibration failed!")
            return False
    
    def _save_calibration(self):
        """Save calibration results to file."""
        if not self.calibrated:
            print("❌ No calibration data to save!")
            return
        
        calib_data = {
            "timestamp": datetime.now().isoformat(),
            "image_count": len(self.obj_points),
            "reprojection_error": float(self.calibration_error),
            "image_size": {
                "width": config.CAMERA_WIDTH,
                "height": config.CAMERA_HEIGHT
            },
            "camera_matrix": self.camera_matrix.tolist(),
            "distortion_coefficients": self.dist_coeffs.flatten().tolist(),
            "board_size": self.board_size,
            "square_size_mm": self.square_size
        }
        
        # Create calibration directory if needed
        self.CALIB_FILE.parent.mkdir(parents=True, exist_ok=True)
        
        # Save to JSON
        with open(self.CALIB_FILE, 'w') as f:
            json.dump(calib_data, f, indent=2)
        
        print(f"✅ Calibration saved to: {self.CALIB_FILE}")
    
    def _draw_info(self, frame, corners_found=False):
        """Draw information overlay."""
        h, w = frame.shape[:2]
        
        # Status panel background
        cv2.rectangle(frame, (10, 10), (w-10, 120), (0, 0, 0), -1)
        
        # Title and instructions
        cv2.putText(frame, "Camera Calibration Tool", (20, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        
        # Status
        if corners_found:
            status = f"✅ Checkerboard detected - Press SPACE to capture"
            color = (0, 255, 0)
        else:
            status = f"🔍 Show {self.board_size[0]}x{self.board_size[1]} checkerboard to camera"
            color = (0, 255, 255)
        
        cv2.putText(frame, status, (20, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Progress
        progress = f"Images captured: {self.capture_count}/20"
        cv2.putText(frame, progress, (20, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        if self.calibrated:
            cv2.putText(frame, f"Calibrated! Error: {self.calibration_error:.3f}px", (20, 105),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    
    def run(self):
        """Run calibration tool."""
        print("=" * 60)
        print("CAMERA INTRINSIC CALIBRATION TOOL")
        print("=" * 60)
        print("Instructions:")
        print("1. Print a 9x6 checkerboard pattern")
        print("2. Show pattern to camera at different angles/distances")
        print("3. When pattern is detected, press SPACE to capture")
        print("4. Capture 15-20 images with good variety")
        print("5. Press 'C' to calibrate")
        print("6. Press 'S' to save results")
        print()
        print("Controls:")
        print("  SPACE - Capture calibration image")
        print("  C - Start calibration")
        print("  S - Save calibration")
        print("  Q - Quit")
        print("=" * 60)
        
        if not self.camera.start():
            print("❌ Failed to start camera!")
            return
        
        cv2.namedWindow(self.WINDOW_NAME)
        
        try:
            while True:
                color_frame, _ = self.camera.get_frames()
                
                if color_frame is None:
                    continue
                
                # Detect checkerboard
                found, corners = self._detect_checkerboard(color_frame)
                
                # Draw checkerboard corners if found
                vis = color_frame.copy()
                if found:
                    cv2.drawChessboardCorners(vis, self.board_size, corners, found)
                
                # Draw info overlay
                self._draw_info(vis, found)
                
                cv2.imshow(self.WINDOW_NAME, vis)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord(' ') and found:  # Space key
                    self._capture_calibration_image(color_frame, corners)
                elif key == ord('c'):
                    self._calibrate_camera()
                elif key == ord('s'):
                    self._save_calibration()
        
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()
            print("📸 Camera calibration completed.")


def main():
    """Main function."""
    tool = CameraCalibrationTool()
    tool.run()


if __name__ == "__main__":
    main()