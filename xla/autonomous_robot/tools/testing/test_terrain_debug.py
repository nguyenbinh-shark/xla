#!/usr/bin/env python3
"""
Test Terrain Analyzer - DEBUG VERSION
Hiển thị chi tiết dữ liệu debug để phân tích và tinh chỉnh.

Hiển thị:
- Raw depth data từng vùng
- Histogram depth distribution
- Zone analysis chi tiết
- Point cloud visualization
- All internal calculations

Usage:
    python tools/testing/test_terrain_debug.py
"""

import cv2
import sys
import time
import numpy as np
from pathlib import Path
from collections import deque

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception import RealSenseCamera
from src.perception.terrain_analyzer import TerrainAnalyzer, TerrainConfig, ClearanceAction


class DebugStats:
    """Lưu trữ thống kê debug."""
    def __init__(self, history_size=100):
        self.ceiling_history = deque(maxlen=history_size)
        self.ground_history = deque(maxlen=history_size)
        self.action_history = deque(maxlen=history_size)
        self.fps_history = deque(maxlen=30)
        
    def update(self, result, fps):
        self.ceiling_history.append(result.ceiling_clearance)
        self.ground_history.append(result.ground_clearance)
        self.action_history.append(result.action.value)
        self.fps_history.append(fps)


def nothing(x):
    pass


def create_histogram(data, bins=50, width=300, height=150, title="Histogram"):
    """Tạo histogram từ dữ liệu depth."""
    # Filter valid data
    valid_data = data[np.isfinite(data) & (data > 0) & (data < 5)]
    
    if len(valid_data) == 0:
        hist_img = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(hist_img, "No valid data", (10, height//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return hist_img
    
    # Create histogram
    hist, bin_edges = np.histogram(valid_data, bins=bins, range=(0, 3))
    
    # Normalize
    max_val = max(hist.max(), 1)
    hist_normalized = (hist / max_val * (height - 30)).astype(np.int32)
    
    # Draw
    hist_img = np.zeros((height, width, 3), dtype=np.uint8)
    bar_width = width // bins
    
    for i, h in enumerate(hist_normalized):
        x1 = i * bar_width
        x2 = (i + 1) * bar_width - 1
        y1 = height - 10 - h
        y2 = height - 10
        
        # Color based on distance (near=red, far=green)
        distance = bin_edges[i]
        if distance < 0.5:
            color = (0, 0, 255)  # Red - dangerous
        elif distance < 1.0:
            color = (0, 165, 255)  # Orange - warning
        elif distance < 2.0:
            color = (0, 255, 255)  # Yellow - caution
        else:
            color = (0, 255, 0)  # Green - safe
            
        cv2.rectangle(hist_img, (x1, y1), (x2, y2), color, -1)
    
    # Title and stats
    cv2.putText(hist_img, title, (5, 15), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    mean_val = np.mean(valid_data)
    min_val = np.min(valid_data)
    max_val = np.max(valid_data)
    std_val = np.std(valid_data)
    
    stats_text = f"Min:{min_val:.2f} Max:{max_val:.2f} Mean:{mean_val:.2f} Std:{std_val:.2f}"
    cv2.putText(hist_img, stats_text, (5, height - 2),
               cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
    
    return hist_img


def draw_depth_zones(depth_frame, config):
    """Vẽ các zone và hiển thị raw depth values."""
    h, w = depth_frame.shape[:2]
    
    # Convert depth to meters
    depth_m = depth_frame.astype(np.float32) / 1000.0
    
    # Zone boundaries
    y_ceil_end = int(h * config.ceiling_zone_bottom)
    y_ground_start = int(h * config.ground_zone_top)
    
    # Create colored depth
    depth_colored = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_frame, alpha=50), 
        cv2.COLORMAP_JET
    )
    
    # Draw zone lines
    cv2.line(depth_colored, (0, y_ceil_end), (w, y_ceil_end), (255, 255, 255), 2)
    cv2.line(depth_colored, (0, y_ground_start), (w, y_ground_start), (255, 255, 255), 2)
    
    # Zone labels
    cv2.putText(depth_colored, "CEILING ZONE", (10, y_ceil_end//2),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(depth_colored, "GROUND ZONE", (10, (y_ground_start + h)//2),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Extract zones
    ceiling_zone = depth_m[0:y_ceil_end, :]
    ground_zone = depth_m[y_ground_start:h, :]
    
    # Sample grid points for depth display
    grid_rows = 5
    grid_cols = 7
    
    for zone, y_offset, color in [(ceiling_zone, 0, (0, 255, 255)), 
                                   (ground_zone, y_ground_start, (255, 0, 255))]:
        zh, zw = zone.shape
        for r in range(grid_rows):
            for c in range(grid_cols):
                y = int((r + 0.5) * zh / grid_rows)
                x = int((c + 0.5) * zw / grid_cols)
                depth_val = zone[y, x]
                
                if 0 < depth_val < 10:
                    screen_y = y_offset + y
                    cv2.circle(depth_colored, (x, screen_y), 3, color, -1)
                    cv2.putText(depth_colored, f"{depth_val:.2f}", (x+5, screen_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.25, color, 1)
    
    return depth_colored, ceiling_zone, ground_zone


def draw_debug_panel(result, config, stats, current_height):
    """Vẽ panel debug với tất cả thông tin."""
    panel_width = 400
    panel_height = 500
    panel = np.zeros((panel_height, panel_width, 3), dtype=np.uint8)
    
    y = 25
    line_height = 18
    
    def put_text(text, color=(255, 255, 255), size=0.45):
        nonlocal y
        cv2.putText(panel, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1)
        y += line_height
    
    def draw_separator():
        nonlocal y
        cv2.line(panel, (5, y-5), (panel_width-5, y-5), (100, 100, 100), 1)
        y += 5
    
    # Header
    put_text("=== TERRAIN ANALYZER DEBUG ===", (0, 255, 255), 0.6)
    y += 5
    draw_separator()
    
    # Action status
    action_colors = {
        ClearanceAction.NORMAL: (0, 255, 0),
        ClearanceAction.RAISE: (0, 255, 255),
        ClearanceAction.LOWER: (255, 165, 0),
        ClearanceAction.STOP: (0, 0, 255),
    }
    action_color = action_colors.get(result.action, (255, 255, 255))
    put_text(f"ACTION: {result.action.name}", action_color, 0.7)
    put_text(f"Reason: {result.reason}", (200, 200, 200), 0.4)
    y += 5
    draw_separator()
    
    # Height information
    put_text("--- HEIGHT INFO ---", (100, 255, 100))
    put_text(f"Current Height: {current_height*100:.1f} cm")
    put_text(f"Recommended: {result.recommended_height*100:.1f} cm", (0, 255, 255))
    put_text(f"Confidence: {result.confidence:.1%}", 
             (0, 255, 0) if result.confidence > 0.7 else (0, 165, 255))
    y += 5
    draw_separator()
    
    # Ceiling analysis
    put_text("--- CEILING ANALYSIS ---", (255, 255, 100))
    if result.ceiling_clearance is not None:
        ceiling_cm = result.ceiling_clearance * 100
        put_text(f"Ceiling Distance: {ceiling_cm:.1f} cm")
        put_text(f"Warning Threshold: {config.ceiling_warning_distance*100:.0f} cm")
        put_text(f"Min Clearance: {config.ceiling_min_clearance*100:.0f} cm")
        
        if result.ceiling_clearance < config.ceiling_min_clearance:
            put_text("  >> DANGER: Too low!", (0, 0, 255))
        elif result.ceiling_clearance < config.ceiling_warning_distance:
            put_text("  >> WARNING: Getting low", (0, 165, 255))
        else:
            put_text("  >> OK: Sufficient clearance", (0, 255, 0))
    else:
        put_text("Ceiling: Not detected", (128, 128, 128))
    y += 5
    draw_separator()
    
    # Ground analysis
    put_text("--- GROUND ANALYSIS ---", (255, 100, 255))
    if result.ground_clearance is not None:
        ground_cm = result.ground_clearance * 100
        put_text(f"Ground Distance: {ground_cm:.1f} cm")
        put_text(f"Expected (camera height): {config.camera_height*100:.0f} cm")
        put_text(f"Max Step Height: {config.max_step_height*100:.1f} cm")
        put_text(f"Obstacle Thresh: {config.obstacle_threshold*100:.1f} cm")
        
        # Obstacle detection
        expected = config.camera_height
        deviation = abs(ground_cm/100 - expected)
        if deviation > config.obstacle_threshold:
            put_text(f"  >> OBSTACLE: {deviation*100:.1f}cm deviation!", (0, 0, 255))
        elif deviation > config.max_step_height:
            put_text(f"  >> STEP: {deviation*100:.1f}cm step", (0, 165, 255))
        else:
            put_text("  >> OK: Clear path", (0, 255, 0))
    else:
        put_text("Ground: Not detected", (128, 128, 128))
    y += 5
    draw_separator()
    
    # Robot config
    put_text("--- ROBOT CONFIG ---", (150, 150, 255))
    put_text(f"Robot Height: {config.robot_height*100:.0f} cm")
    put_text(f"Camera Height: {config.camera_height*100:.0f} cm")
    put_text(f"Camera Tilt: {config.camera_tilt_angle:.1f} deg")
    put_text(f"Ground Clearance Range: {config.min_ground_clearance*100:.0f}-{config.max_ground_clearance*100:.0f} cm")
    y += 5
    draw_separator()
    
    # Statistics
    put_text("--- STATISTICS ---", (255, 200, 100))
    if len(stats.fps_history) > 0:
        avg_fps = np.mean(stats.fps_history)
        put_text(f"Avg FPS: {avg_fps:.1f}")
    
    if len(stats.action_history) > 0:
        actions = list(stats.action_history)
        normal_pct = actions.count("normal") / len(actions) * 100
        raise_pct = actions.count("raise") / len(actions) * 100
        lower_pct = actions.count("lower") / len(actions) * 100
        stop_pct = actions.count("stop") / len(actions) * 100
        
        put_text(f"Normal: {normal_pct:.0f}% | Raise: {raise_pct:.0f}%")
        put_text(f"Lower: {lower_pct:.0f}% | Stop: {stop_pct:.0f}%")
    
    return panel


def draw_history_graph(stats, width=400, height=150):
    """Vẽ đồ thị lịch sử clearance."""
    graph = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Grid
    for i in range(5):
        y = int(i * height / 4)
        cv2.line(graph, (0, y), (width, y), (50, 50, 50), 1)
        val = 2.0 - i * 0.5  # 2.0m to 0m
        cv2.putText(graph, f"{val:.1f}m", (5, y+12), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (128, 128, 128), 1)
    
    def draw_line(data, color, label, x_offset=0):
        if len(data) < 2:
            return
        
        points = []
        for i, val in enumerate(data):
            if val is not None and val > 0:
                x = int(i * width / len(data))
                y = int(height - (val / 2.0) * height)  # Normalize to 0-2m
                y = max(0, min(height-1, y))
                points.append((x, y))
        
        if len(points) > 1:
            for i in range(len(points)-1):
                cv2.line(graph, points[i], points[i+1], color, 1)
        
        # Label
        cv2.putText(graph, label, (width - 100 + x_offset, 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
    
    draw_line(stats.ceiling_history, (0, 255, 255), "Ceiling", 0)
    draw_line(stats.ground_history, (255, 0, 255), "Ground", 50)
    
    cv2.putText(graph, "Clearance History", (width//2 - 50, height - 5),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    
    return graph


def main():
    print("=" * 70)
    print("  TERRAIN ANALYZER - DEBUG MODE")
    print("  Hiển thị chi tiết dữ liệu để phân tích và tinh chỉnh")
    print("=" * 70)
    print("\nControls:")
    print("  Q       - Quit")
    print("  R       - Reset analyzer & stats")
    print("  S       - Save debug snapshot")
    print("  SPACE   - Pause/Resume")
    print("  +/-     - Adjust simulation height")
    print("  1/2/3   - Quick set height (min/normal/max)")
    print("  Trackbars - Điều chỉnh thông số realtime")
    print("=" * 70)
    
    # Initialize
    camera = RealSenseCamera()
    if not camera.start():
        print("ERROR: Cannot start camera!")
        return
    
    config = TerrainConfig(
        # Robot dimensions
        robot_height=0.25,              # 25cm
        min_ground_clearance=0.07,      # 7cm
        max_ground_clearance=0.18,      # 18cm
        normal_ground_clearance=0.13,   # 13cm
        
        # Camera mounting
        camera_height=0.20,             # 20cm
        camera_tilt_angle=15.0,         # 15 độ
        
        # Detection thresholds
        ceiling_min_clearance=0.4,      # 40cm
        ceiling_warning_distance=0.8,   # 80cm
        max_step_height=0.05,           # 5cm
        obstacle_threshold=0.10,        # 10cm
    )
    
    analyzer = TerrainAnalyzer(config)
    stats = DebugStats()
    
    # Create windows
    cv2.namedWindow("Main View", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Debug Panel", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Histograms", cv2.WINDOW_NORMAL)
    
    cv2.resizeWindow("Main View", 1280, 480)
    cv2.resizeWindow("Debug Panel", 400, 500)
    cv2.resizeWindow("Histograms", 620, 350)
    
    cv2.moveWindow("Main View", 50, 50)
    cv2.moveWindow("Debug Panel", 1350, 50)
    cv2.moveWindow("Histograms", 50, 580)
    
    # Trackbars
    cv2.createTrackbar("Ceil Warn (cm)", "Debug Panel", 
                       int(config.ceiling_warning_distance * 100), 200, nothing)
    cv2.createTrackbar("Ceil Min (cm)", "Debug Panel", 
                       int(config.ceiling_min_clearance * 100), 100, nothing)
    cv2.createTrackbar("Max Step (cm)", "Debug Panel", 
                       int(config.max_step_height * 100), 20, nothing)
    cv2.createTrackbar("Obstacle Th (cm)", "Debug Panel", 
                       int(config.obstacle_threshold * 100), 50, nothing)
    cv2.createTrackbar("Cam Height (cm)", "Debug Panel",
                       int(config.camera_height * 100), 50, nothing)
    cv2.createTrackbar("Cam Tilt (deg)", "Debug Panel",
                       int(config.camera_tilt_angle), 45, nothing)
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    paused = False
    last_frame = None
    last_depth = None
    
    # Simulated current height
    current_height = config.normal_ground_clearance
    
    try:
        while True:
            if not paused:
                color_frame, depth_frame = camera.get_frames()
                
                if color_frame is None or depth_frame is None:
                    continue
                
                last_frame = color_frame.copy()
                last_depth = depth_frame.copy()
            else:
                color_frame = last_frame
                depth_frame = last_depth
                if color_frame is None:
                    continue
            
            frame_count += 1
            
            # Update config from trackbars
            config.ceiling_warning_distance = cv2.getTrackbarPos("Ceil Warn (cm)", "Debug Panel") / 100.0
            config.ceiling_min_clearance = cv2.getTrackbarPos("Ceil Min (cm)", "Debug Panel") / 100.0
            config.max_step_height = cv2.getTrackbarPos("Max Step (cm)", "Debug Panel") / 100.0
            config.obstacle_threshold = cv2.getTrackbarPos("Obstacle Th (cm)", "Debug Panel") / 100.0
            config.camera_height = cv2.getTrackbarPos("Cam Height (cm)", "Debug Panel") / 100.0
            config.camera_tilt_angle = cv2.getTrackbarPos("Cam Tilt (deg)", "Debug Panel")
            
            # Analyze
            result = analyzer.analyze(depth_frame, color_frame)
            
            # Simulate height adjustment
            if result.action != ClearanceAction.STOP:
                height_diff = result.recommended_height - current_height
                current_height += height_diff * 0.05  # Smooth 5%
            
            # Calculate FPS
            if time.time() - fps_time > 0.5:
                fps = frame_count / (time.time() - fps_time)
                fps_time = time.time()
                frame_count = 0
            
            # Update stats
            stats.update(result, fps)
            
            # === MAIN VIEW ===
            # Original visualization
            vis = analyzer.visualize(color_frame, depth_frame, result)
            
            # Depth with zones and values
            depth_vis, ceiling_zone, ground_zone = draw_depth_zones(depth_frame, config)
            
            # Combine main view
            main_view = cv2.hconcat([vis, depth_vis])
            
            # Add FPS and pause indicator
            status_color = (0, 0, 255) if paused else (0, 255, 0)
            status_text = "PAUSED" if paused else f"FPS: {fps:.1f}"
            cv2.putText(main_view, status_text, (10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            
            # Current height bar
            bar_x = main_view.shape[1] - 50
            bar_h = main_view.shape[0] - 40
            cv2.rectangle(main_view, (bar_x, 20), (bar_x + 30, bar_h), (50, 50, 50), -1)
            
            # Height indicator
            height_ratio = (current_height - config.min_ground_clearance) / \
                          (config.max_ground_clearance - config.min_ground_clearance)
            height_ratio = max(0, min(1, height_ratio))
            indicator_y = int(bar_h - height_ratio * (bar_h - 20))
            cv2.rectangle(main_view, (bar_x, indicator_y), (bar_x + 30, bar_h), (0, 255, 0), -1)
            cv2.putText(main_view, f"{current_height*100:.0f}cm", (bar_x-25, indicator_y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            
            cv2.imshow("Main View", main_view)
            
            # === DEBUG PANEL ===
            debug_panel = draw_debug_panel(result, config, stats, current_height)
            cv2.imshow("Debug Panel", debug_panel)
            
            # === HISTOGRAMS ===
            hist_ceiling = create_histogram(ceiling_zone, title="Ceiling Zone Depth Distribution")
            hist_ground = create_histogram(ground_zone, title="Ground Zone Depth Distribution")
            
            # History graph
            history_graph = draw_history_graph(stats, width=300, height=150)
            
            # Combine histograms
            hist_row = cv2.hconcat([hist_ceiling, hist_ground])
            
            # Pad history graph to match width
            pad_width = hist_row.shape[1] - history_graph.shape[1]
            if pad_width > 0:
                history_graph = cv2.copyMakeBorder(history_graph, 0, 0, 0, pad_width,
                                                   cv2.BORDER_CONSTANT, value=(0, 0, 0))
            
            hist_view = cv2.vconcat([hist_row, history_graph])
            cv2.imshow("Histograms", hist_view)
            
            # === CONSOLE OUTPUT (every 2 seconds) ===
            if int(time.time() * 10) % 20 == 0:
                print(f"\r[{result.action.name:6s}] "
                      f"Ceiling: {result.ceiling_clearance*100 if result.ceiling_clearance else 0:.0f}cm | "
                      f"Ground: {result.ground_clearance*100 if result.ground_clearance else 0:.0f}cm | "
                      f"Height: {current_height*100:.1f}cm -> {result.recommended_height*100:.1f}cm | "
                      f"Conf: {result.confidence:.0%}", end="")
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\nQuitting...")
                break
            elif key == ord('r'):
                analyzer.reset()
                stats = DebugStats()
                current_height = config.normal_ground_clearance
                print("\nReset!")
            elif key == ord('s'):
                timestamp = int(time.time())
                cv2.imwrite(f"debug_main_{timestamp}.jpg", main_view)
                cv2.imwrite(f"debug_panel_{timestamp}.jpg", debug_panel)
                cv2.imwrite(f"debug_hist_{timestamp}.jpg", hist_view)
                print(f"\nSaved debug snapshots: debug_*_{timestamp}.jpg")
            elif key == ord(' '):
                paused = not paused
                print(f"\n{'Paused' if paused else 'Resumed'}")
            elif key == ord('+') or key == ord('='):
                current_height = min(config.max_ground_clearance, current_height + 0.01)
                print(f"\nHeight: {current_height*100:.0f}cm")
            elif key == ord('-'):
                current_height = max(config.min_ground_clearance, current_height - 0.01)
                print(f"\nHeight: {current_height*100:.0f}cm")
            elif key == ord('1'):
                current_height = config.min_ground_clearance
                print(f"\nSet to MIN: {current_height*100:.0f}cm")
            elif key == ord('2'):
                current_height = config.normal_ground_clearance
                print(f"\nSet to NORMAL: {current_height*100:.0f}cm")
            elif key == ord('3'):
                current_height = config.max_ground_clearance
                print(f"\nSet to MAX: {current_height*100:.0f}cm")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("\n" + "=" * 70)
        print("Session Summary:")
        if len(stats.action_history) > 0:
            actions = list(stats.action_history)
            print(f"  Total frames analyzed: {len(actions)}")
            print(f"  Action distribution:")
            for action in ["normal", "raise", "lower", "stop"]:
                pct = actions.count(action) / len(actions) * 100
                print(f"    - {action.upper()}: {pct:.1f}%")
        print("=" * 70)


if __name__ == "__main__":
    main()
