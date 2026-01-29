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
            valid_depths = []
            if depth_mode == 0:
                grid_size = 5
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
            
            # Create parameter panel on right side
            panel_w = 280
            panel = np.zeros((h, panel_w, 3), dtype=np.uint8)
            panel[:] = (40, 40, 40)
            
            # Panel title
            cv2.putText(panel, "DEPTH ESTIMATOR", (10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.line(panel, (10, 35), (panel_w - 10, 35), (100, 100, 100), 1)
            
            # Draw parameters with bars
            y_pos = 55
            bar_x = 130
            bar_w = 120
            bar_h = 12
            
            params = [
                ("=== CONFIG ===", None, None, None),
                ("Filter Size", estimator.median_filter_size, 15, (100, 255, 100)),
                ("Min Depth", f"{min_depth:.1f}m", min_depth / 5, (100, 200, 255)),
                ("Max Depth", f"{max_depth:.1f}m", max_depth / 20, (255, 200, 100)),
                ("=== CURRENT ===", None, None, None),
                ("Center", f"{center_depth:.3f}m", center_depth / 5, (255, 255, 0)),
                ("Frame", frame_count, None, (200, 200, 200)),
                ("FPS", f"{fps:.1f}", fps / 60, (0, 255, 0)),
            ]
            
            # Add statistics if available
            if depth_history:
                avg_d = sum(depth_history) / len(depth_history)
                min_d = min(depth_history)
                max_d = max(depth_history)
                params.extend([
                    ("=== STATS ===", None, None, None),
                    ("Average", f"{avg_d:.2f}m", avg_d / 5, (150, 255, 150)),
                    ("Min", f"{min_d:.2f}m", min_d / 5, (150, 150, 255)),
                    ("Max", f"{max_d:.2f}m", max_d / 5, (255, 150, 150)),
                    ("Samples", len(depth_history), len(depth_history) / 1000, (200, 200, 200)),
                ])
            
            # Add click info
            if click_point and click_depth:
                params.extend([
                    ("=== CLICK ===", None, None, None),
                    ("Position", f"({click_point[0]}, {click_point[1]})", None, (255, 100, 100)),
                    ("Depth", f"{click_depth:.3f}m", click_depth / 5, (255, 100, 100)),
                ])
            
            for name, value, ratio, color in params:
                if value is None:
                    y_pos += 5
                    cv2.putText(panel, name, (10, y_pos), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                    y_pos += 18
                    continue
                
                # Name
                cv2.putText(panel, name, (10, y_pos + 3), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                
                # Value
                val_str = str(value) if not isinstance(value, float) else f"{value}"
                cv2.putText(panel, val_str, (bar_x - 10, y_pos + 3), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
                
                # Bar (if ratio provided)
                if ratio is not None:
                    cv2.rectangle(panel, (bar_x + 40, y_pos - bar_h//2), 
                                 (bar_x + 40 + bar_w, y_pos + bar_h//2), (60, 60, 60), -1)
                    fill_w = int(bar_w * min(1.0, ratio)) if ratio else 0
                    if fill_w > 0:
                        cv2.rectangle(panel, (bar_x + 40, y_pos - bar_h//2), 
                                     (bar_x + 40 + fill_w, y_pos + bar_h//2), color, -1)
                    cv2.rectangle(panel, (bar_x + 40, y_pos - bar_h//2), 
                                 (bar_x + 40 + bar_w, y_pos + bar_h//2), (100, 100, 100), 1)
                
                y_pos += 20
            
            # Controls at bottom
            y_pos = h - 120
            cv2.line(panel, (10, y_pos), (panel_w - 10, y_pos), (100, 100, 100), 1)
            y_pos += 20
            cv2.putText(panel, "CONTROLS:", (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            controls = [
                "[Q] Quit  [S] Save",
                "[D] Mode  [P] Pause",
                "[R] Reset [H] Histogram",
                "Click to measure"
            ]
            for ctrl in controls:
                y_pos += 18
                cv2.putText(panel, ctrl, (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
            
            # Mode indicator
            mode_text = ['GRID', 'COLORMAP', 'SIDE-BY-SIDE'][depth_mode]
            cv2.putText(panel, f"Mode: {mode_text}", (10, h - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            
            # Combine vis and panel
            if depth_mode == 2:  # Side-by-side mode is wider
                panel_resized = cv2.resize(panel, (panel_w, vis.shape[0]))
                vis = np.hstack([vis, panel_resized])
            else:
                vis = np.hstack([vis, panel])
            
            # Show histogram if requested (overlay on panel area)
            if show_histogram and depth_history:
                hist_h, hist_w = 150, 250
                hist_img = np.zeros((hist_h, hist_w, 3), dtype=np.uint8)
                hist_img[:] = (30, 30, 30)
                hist, bins = np.histogram(depth_history, bins=50, range=(0, 5))
                if hist.max() > 0:
                    hist_norm = hist * (hist_h - 30) / hist.max()
                    for i in range(len(hist)):
                        pt1 = (int(i * 5), hist_h - 10)
                        pt2 = (int(i * 5), hist_h - 10 - int(hist_norm[i]))
                        cv2.line(hist_img, pt1, pt2, (0, 255, 0), 2)
                cv2.putText(hist_img, "Histogram (0-5m)", (10, 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.rectangle(hist_img, (0, 0), (hist_w-1, hist_h-1), (100, 100, 100), 1)
                
                # Place histogram on top-right of vis area (before panel)
                vis_h, vis_w = vis.shape[:2]
                x_offset = vis_w - panel_w - hist_w - 10
                y_offset = 10
                vis[y_offset:y_offset+hist_h, x_offset:x_offset+hist_w] = hist_img
            
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
