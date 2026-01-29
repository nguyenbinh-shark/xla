#!/usr/bin/env python3
"""
Test Depth Estimator với hiển thị trực quan.

Usage:
    python tools/testing/test_depth_estimator.py
"""

import cv2
import numpy as np
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception import RealSenseCamera, DepthEstimator
from src.core import config


def nothing(x):
    """Callback for trackbar."""
    pass


def main():
    # Auto-load default config
    config_file = PROJECT_ROOT / "configs" / "default_config.yaml"
    print(f"Auto-loading config: {config_file}")
    try:
        config.load_config(str(config_file))
    except Exception as e:
        print(f"Warning: Could not load config: {e}")
        
    print("=" * 60)
    print("  TEST: DEPTH ESTIMATOR (RealSense)")
    print("=" * 60)
    print("Controls:")
    print("  Q - Quit")
    print("  S - Save frame") 
    print("  D - Toggle depth view mode")
    print("  P - Pause/Resume")
    print("  R - Reset statistics")
    print("  H - Show depth histogram")
    print("  Click - Measure depth at point")
    print("  Trackbars - Adjust parameters")
    print("=" * 60)
    
    camera = RealSenseCamera()
    estimator = DepthEstimator()
    
    if not camera.start():
        print("ERROR: Cannot start camera!")
        return
    
    # Create window and trackbars
    cv2.namedWindow("Depth Estimator Test")
    cv2.createTrackbar("Filter Size", "Depth Estimator Test", config.DEPTH_MEDIAN_FILTER_SIZE, 15, nothing)
    cv2.createTrackbar("Min Depth(cm)", "Depth Estimator Test", int(config.DEPTH_MIN_VALID*100), 500, nothing)
    cv2.createTrackbar("Max Depth(cm)", "Depth Estimator Test", int(config.DEPTH_MAX_VALID*100), 2000, nothing)
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    click_point = None
    click_depth = None
    paused = False
    show_histogram = False
    depth_mode = 0  # 0=grid, 1=full colormap, 2=side-by-side
    depth_history = []

    def mouse_callback(event, x, y, flags, param):
        nonlocal click_point, click_depth
        if event == cv2.EVENT_LBUTTONDOWN:
            click_point = (x, y)
            if param is not None:
                click_depth = estimator.get_depth_at_point(param, x, y)
                print(f"Depth at ({x}, {y}): {click_depth:.3f}m")
    
    cv2.namedWindow("Depth Estimator Test")
    try:
        while True:
            color_frame, depth_frame = camera.get_frames()
            
            if color_frame is None or depth_frame is None:
                continue
            # Handle pause
            if paused:
                cv2.waitKey(30)
                continue
            
            frame_count += 1
            
            # Update config from trackbars
            filter_size = cv2.getTrackbarPos("Filter Size", "Depth Estimator Test")
            min_depth = cv2.getTrackbarPos("Min Depth(cm)", "Depth Estimator Test") / 100.0
            max_depth = cv2.getTrackbarPos("Max Depth(cm)", "Depth Estimator Test") / 100.0
            
            estimator.median_filter_size = max(1, filter_size)
            estimator.min_valid_depth = min_depth  
            estimator.max_valid_depth = max_depth
            
            # Calculate FPS
            if time.time() - fps_time > 1.0:
                fps = frame_count / (time.time() - fps_time)
                fps_time = time.time()
                frame_count = 0
            
            # Set mouse callback with depth frame
            cv2.setMouseCallback("Depth Estimator Test", mouse_callback, depth_frame)
            
            # Choose visualization mode
            h, w = color_frame.shape[:2]
            
            if depth_mode == 1:  # Full colormap
                depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
                vis = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
            elif depth_mode == 2:  # Side-by-side
                depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
                vis = np.hstack([color_frame, depth_colored])
                cv2.putText(vis, "RGB", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(vis, "DEPTH", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            else:  # Grid mode (default)
                vis = color_frame.copy()
            
            # Draw grid of depth measurements (only in grid mode)
            if depth_mode == 0:
                grid_size = 5
                valid_depths = []
                for row in range(grid_size):
                    for col in range(grid_size):
                        x = int(w * (col + 1) / (grid_size + 1))
                        y = int(h * (row + 1) / (grid_size + 1))
                        
                        depth = estimator.get_depth_at_point(depth_frame, x, y)
                        if depth > 0:
                            valid_depths.append(depth)
                        
                        # Draw crosshair
                        cv2.line(vis, (x - 8, y), (x + 8, y), (0, 255, 0), 1)
                        cv2.line(vis, (x, y - 8), (x, y + 8), (0, 255, 0), 1)
                        
                        # Draw depth
                        if depth > 0:
                            text = f"{depth:.2f}m"
                            color = (0, 255, 0) if depth > 0.5 else (0, 255, 255) if depth > 0.3 else (0, 0, 255)
                        else:
                            text = "N/A"
                            color = (128, 128, 128)
                        
                        cv2.putText(vis, text, (x + 5, y - 5),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                
                # Update depth history for statistics
                if valid_depths:
                    depth_history.extend(valid_depths)
                    if len(depth_history) > 1000:  # Keep last 1000 measurements
                        depth_history = depth_history[-1000:]
            
            # Center depth (highlighted)
            cx, cy = w // 2, h // 2
            center_depth = estimator.get_depth_at_point(depth_frame, cx, cy)
            cv2.circle(vis, (cx, cy), 20, (255, 0, 0), 2)
            
            # Draw click point
            if click_point and click_depth:
                cv2.circle(vis, click_point, 10, (0, 0, 255), 2)
                cv2.putText(vis, f"{click_depth:.3f}m", 
                           (click_point[0] + 15, click_point[1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Comprehensive info panel
            info = [
                f"Frame: {frame_count}  FPS: {fps:.1f}",
                f"Mode: {['Grid', 'Colormap', 'Side-by-Side'][depth_mode]}",
                f"Center: {center_depth:.3f}m",
                f"Config: Filter={estimator.median_filter_size} Range={min_depth:.1f}-{max_depth:.1f}m",
            ]
            
            if click_point and click_depth:
                info.append(f"Click: ({click_point[0]}, {click_point[1]}) = {click_depth:.3f}m")
                
            # Add depth statistics if available
            if depth_history:
                avg_depth = sum(depth_history) / len(depth_history)
                min_hist = min(depth_history)
                max_hist = max(depth_history)
                info.append(f"Stats: Avg={avg_depth:.2f}m Min={min_hist:.2f}m Max={max_hist:.2f}m")
            
            y = 30
            for text in info:
                cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                y += 25
            
            # Add preview/histogram based on mode
            if depth_mode == 0:  # Grid mode - show depth preview
                depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
                depth_small = cv2.resize(depth_colored, (160, 120))
                vis[h-130:h-10, w-170:w-10] = depth_small
                cv2.rectangle(vis, (w-170, h-130), (w-10, h-10), (255, 255, 255), 1)
                
            # Show histogram if requested
            if show_histogram and depth_history:
                hist_img = np.zeros((200, 300, 3), dtype=np.uint8)
                hist, bins = np.histogram(depth_history, bins=50, range=(0, 5))
                hist = hist * 180 / max(hist)  # Normalize to fit display
                
                for i in range(len(hist)):
                    pt1 = (int(i * 6), 200)
                    pt2 = (int(i * 6), 200 - int(hist[i]))
                    cv2.line(hist_img, pt1, pt2, (0, 255, 0), 2)
                    
                cv2.putText(hist_img, "Depth Histogram (0-5m)", (10, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                vis[10:210, w-310:w-10] = hist_img
            
            cv2.imshow("Depth Estimator Test", vis)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"depth_test_{frame_count}.jpg"
                cv2.imwrite(filename, vis)
                print(f"Saved: {filename}")
            elif key == ord('d'):
                depth_mode = (depth_mode + 1) % 3
                print(f"Depth mode: {['Grid', 'Colormap', 'Side-by-Side'][depth_mode]}")
            elif key == ord('p'):
                paused = not paused
                print(f"{'PAUSED' if paused else 'RESUMED'}")
            elif key == ord('r'):
                depth_history = []
                frame_count = 0
                fps_time = time.time()
                print("Statistics reset!")
            elif key == ord('h'):
                show_histogram = not show_histogram
                print(f"Histogram: {'ON' if show_histogram else 'OFF'}")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Test completed.")


if __name__ == "__main__":
    main()
