#!/usr/bin/env python3
"""
Terrain Obstacle Calibration Tool - Công cụ Calibrate Phát hiện Vật cản

Mục đích:
- Visualize baseline tuyến tính theo y
- Debug thuật toán phát hiện vật cản
- Điều chỉnh các thông số detection
- Xem đồ thị depth profile theo từng hàng

Đồ thị hiển thị:
1. Depth heatmap với vùng ROI
2. Baseline line (fit tuyến tính) vs actual depth
3. Obstacle detection mask
4. Depth profile theo y (cross-section)

Usage:
    python tools/calibration/terrain_obstacle_calibration.py
    
Controls:
    Q - Quit
    S - Save config
    L - Load config  
    R - Reset to defaults
    P - Pause/Resume
    SPACE - Capture single frame for analysis
    Mouse click - Select column for depth profile
"""

import cv2
import sys
import time
import json
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception import RealSenseCamera
from src.perception.terrain_analyzer import TerrainConfig


@dataclass
class CalibrationState:
    """Trạng thái calibration."""
    selected_column: int = -1  # Column được chọn để xem profile
    paused: bool = False
    captured_depth: Optional[np.ndarray] = None
    captured_color: Optional[np.ndarray] = None


class TerrainObstacleCalibrator:
    """
    Tool calibration cho phát hiện vật cản terrain.
    Hiển thị đồ thị chi tiết về baseline tuyến tính và obstacle detection.
    """
    
    def __init__(self, config: Optional[TerrainConfig] = None):
        self.config = config or TerrainConfig()
        self.state = CalibrationState()
        self.config_file = PROJECT_ROOT / "data/calibration/terrain_config.json"
        
        # Analysis results cache
        self._last_analysis = {}
        
        # Setup matplotlib figure
        self.setup_plots()
    
    def setup_plots(self):
        """Setup matplotlib figures cho đồ thị."""
        self.fig, self.axes = plt.subplots(2, 2, figsize=(10, 7))
        self.fig.suptitle('Terrain Obstacle Calibration', fontsize=12)
        plt.tight_layout()
        
    def analyze_ground_detailed(
        self, 
        depth_frame: np.ndarray
    ) -> dict:
        """
        Phân tích chi tiết vùng ground để debug.
        Trả về tất cả dữ liệu trung gian.
        """
        h, w = depth_frame.shape[:2]
        
        # Extract ground zone
        y_start = int(h * self.config.ground_zone_top)
        y_end = int(h * self.config.ground_zone_bottom)
        ground_zone = depth_frame[y_start:y_end, :]
        roi_h, roi_w = ground_zone.shape[:2]
        
        result = {
            'y_start': y_start,
            'y_end': y_end,
            'roi_h': roi_h,
            'roi_w': roi_w,
            'ground_zone': ground_zone.copy(),
        }
        
        # Tính baseline depth cho từng hàng (median theo hàng)
        row_baseline = np.zeros(roi_h)
        row_valid_count = np.zeros(roi_h)
        
        for i in range(roi_h):
            row = ground_zone[i, :]
            valid = (row > self.config.depth_min_valid) & (row < self.config.depth_max_valid)
            row_valid_count[i] = np.sum(valid)
            if row_valid_count[i] > 10:
                row_baseline[i] = np.median(row[valid])
            else:
                row_baseline[i] = np.nan
        
        result['row_baseline'] = row_baseline
        result['row_valid_count'] = row_valid_count
        
        # Loại bỏ các hàng không hợp lệ
        valid_rows = ~np.isnan(row_baseline)
        result['valid_rows_mask'] = valid_rows
        result['num_valid_rows'] = np.sum(valid_rows)
        
        if np.sum(valid_rows) < 5:
            result['fit_success'] = False
            result['obstacle_detected'] = False
            return result
        
        # Fit baseline tuyến tính theo y
        y_idx = np.arange(roi_h)[valid_rows]
        baseline_vals = row_baseline[valid_rows]
        
        fit = np.polyfit(y_idx, baseline_vals, 1)
        baseline_line = np.polyval(fit, np.arange(roi_h))
        
        result['fit_success'] = True
        result['fit_coeffs'] = fit  # [slope, intercept]
        result['baseline_line'] = baseline_line
        result['y_indices'] = y_idx
        result['baseline_values'] = baseline_vals
        
        # Tính slope (m/pixel) - cho biết depth thay đổi bao nhiêu theo y
        slope_m_per_pixel = fit[0]
        result['slope_m_per_pixel'] = slope_m_per_pixel
        result['slope_cm_per_row'] = slope_m_per_pixel * 100
        
        # Tìm các điểm có depth nhỏ hơn baseline (vật cản)
        obstacle_mask = np.zeros_like(ground_zone, dtype=bool)
        depth_diff_map = np.zeros_like(ground_zone, dtype=float)
        
        for i in range(roi_h):
            row = ground_zone[i, :]
            valid = (row > self.config.depth_min_valid) & (row < self.config.depth_max_valid)
            diff = baseline_line[i] - row
            depth_diff_map[i, :] = np.where(valid, diff, 0)
            obstacle_mask[i, :] = valid & (row < (baseline_line[i] - self.config.obstacle_threshold))
        
        result['obstacle_mask'] = obstacle_mask
        result['depth_diff_map'] = depth_diff_map
        result['obstacle_pixel_count'] = np.sum(obstacle_mask)
        result['obstacle_detected'] = np.any(obstacle_mask)
        
        if result['obstacle_detected']:
            # Tính chiều cao vật cản
            obstacle_depths = ground_zone[obstacle_mask]
            obstacle_distance = float(np.min(obstacle_depths))
            
            y_obstacle, x_obstacle = np.unravel_index(
                np.argmin(np.where(obstacle_mask, ground_zone, np.inf)), 
                ground_zone.shape
            )
            baseline_at_obstacle = baseline_line[y_obstacle]
            depth_diff = baseline_at_obstacle - obstacle_distance
            
            # Góc trung bình của ground zone
            avg_ground_angle = self.config.camera_tilt_angle + self.config.camera_vfov * 0.25
            angle_rad = np.radians(avg_ground_angle)
            estimated_height = depth_diff * np.sin(angle_rad)
            estimated_height = min(0.3, max(0, estimated_height))
            
            result['obstacle_distance'] = obstacle_distance
            result['obstacle_y'] = y_obstacle
            result['obstacle_x'] = x_obstacle
            result['baseline_at_obstacle'] = baseline_at_obstacle
            result['depth_diff'] = depth_diff
            result['estimated_height_m'] = estimated_height
            result['estimated_height_cm'] = estimated_height * 100
            result['avg_ground_angle'] = avg_ground_angle
        
        self._last_analysis = result
        return result
    
    def update_plots(self, analysis: dict):
        """Cập nhật các đồ thị matplotlib."""
        for ax in self.axes.flat:
            ax.clear()
        
        if 'ground_zone' not in analysis:
            return
        
        ground_zone = analysis['ground_zone']
        roi_h, roi_w = ground_zone.shape
        
        # === Plot 1: Depth Heatmap ===
        ax1 = self.axes[0, 0]
        valid_depth = np.where(
            (ground_zone > self.config.depth_min_valid) & 
            (ground_zone < self.config.depth_max_valid),
            ground_zone, np.nan
        )
        im1 = ax1.imshow(valid_depth, cmap='jet', aspect='auto')
        ax1.set_title(f'Ground ROI [{analysis["y_start"]}:{analysis["y_end"]}]')
        ax1.set_xlabel('X (pixel)')
        ax1.set_ylabel('Y in ROI')
        
        if self.state.selected_column >= 0 and self.state.selected_column < roi_w:
            ax1.axvline(x=self.state.selected_column, color='white', linestyle='--', linewidth=2)
        
        # === Plot 2: Baseline Fit ===
        ax2 = self.axes[0, 1]
        if analysis.get('fit_success', False):
            y_idx = analysis['y_indices']
            baseline_vals = analysis['baseline_values']
            baseline_line = analysis['baseline_line']
            
            ax2.scatter(y_idx, baseline_vals, c='blue', s=10, alpha=0.6, label='Row Median')
            ax2.plot(np.arange(roi_h), baseline_line, 'r-', linewidth=2, 
                    label=f'Fit: {analysis["slope_cm_per_row"]:.3f} cm/row')
            ax2.plot(np.arange(roi_h), baseline_line - self.config.obstacle_threshold, 
                    'g--', linewidth=1, label=f'Thresh (-{self.config.obstacle_threshold*100:.0f}cm)')
            
            ax2.set_xlabel('Y in ROI')
            ax2.set_ylabel('Depth (m)')
            ax2.set_title('Baseline Fit')
            ax2.legend(fontsize=7)
            ax2.grid(True, alpha=0.3)
        else:
            ax2.text(0.5, 0.5, 'No data', ha='center', va='center')
        
        # === Plot 3: Obstacle Mask ===
        ax3 = self.axes[1, 0]
        if 'obstacle_mask' in analysis:
            obstacle_vis = np.zeros((*ground_zone.shape, 3), dtype=np.uint8)
            valid_mask = (ground_zone > self.config.depth_min_valid) & \
                        (ground_zone < self.config.depth_max_valid)
            depth_norm = np.clip((ground_zone - 0.3) / 2.0, 0, 1)
            obstacle_vis[valid_mask] = (depth_norm[valid_mask, None] * 200).astype(np.uint8)
            obstacle_vis[analysis['obstacle_mask'], 0] = 255
            obstacle_vis[analysis['obstacle_mask'], 1:] = 0
            
            ax3.imshow(obstacle_vis, aspect='auto')
            title = f"Obstacles: {analysis['obstacle_pixel_count']} px"
            if analysis.get('obstacle_detected'):
                title += f" | {analysis.get('estimated_height_cm', 0):.1f}cm"
            ax3.set_title(title)
        ax3.set_xlabel('X (pixel)')
        ax3.set_ylabel('Y in ROI')
        
        # === Plot 4: Depth Profile ===
        ax4 = self.axes[1, 1]
        col = self.state.selected_column
        if col >= 0 and col < roi_w:
            column_depth = ground_zone[:, col]
            y_range = np.arange(roi_h)
            valid = (column_depth > self.config.depth_min_valid) & \
                   (column_depth < self.config.depth_max_valid)
            
            ax4.scatter(y_range[valid], column_depth[valid], c='blue', s=10, label='Depth')
            
            if analysis.get('fit_success'):
                ax4.plot(y_range, analysis['baseline_line'], 'r-', linewidth=2, label='Baseline')
                ax4.plot(y_range, analysis['baseline_line'] - self.config.obstacle_threshold,
                        'g--', linewidth=1, label='Threshold')
            
            if 'obstacle_mask' in analysis:
                obs_in_col = analysis['obstacle_mask'][:, col]
                if np.any(obs_in_col):
                    ax4.scatter(y_range[obs_in_col], column_depth[obs_in_col], 
                               c='red', s=50, marker='x', label='Obstacle')
            
            ax4.set_title(f'Profile @ Col {col}')
            ax4.legend(fontsize=7)
            ax4.grid(True, alpha=0.3)
        else:
            ax4.text(0.5, 0.5, 'Click to select', ha='center', va='center')
            ax4.set_title('Depth Profile')
        
        ax4.set_xlabel('Y in ROI')
        ax4.set_ylabel('Depth (m)')
        
        plt.tight_layout()
    
    def render_plots_to_image(self) -> np.ndarray:
        """Render matplotlib figure thành numpy image."""
        canvas = FigureCanvasAgg(self.fig)
        canvas.draw()
        buf = canvas.buffer_rgba()
        img = np.asarray(buf)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        return img
    
    def create_main_view(
        self, 
        color_frame: np.ndarray, 
        depth_frame: np.ndarray,
        analysis: dict
    ) -> np.ndarray:
        """Tạo main visualization với ROI và info."""
        vis = color_frame.copy()
        h, w = vis.shape[:2]
        
        # Draw ground zone ROI
        y_start = int(h * self.config.ground_zone_top)
        y_end = int(h * self.config.ground_zone_bottom)
        
        color = (0, 255, 0) if not analysis.get('obstacle_detected') else (0, 0, 255)
        cv2.rectangle(vis, (0, y_start), (w, y_end), color, 2)
        cv2.putText(vis, f"Ground ROI [{y_start}:{y_end}]", 
                   (10, y_start - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Draw ceiling zone ROI
        y_ceil_start = int(h * self.config.ceiling_zone_top)
        y_ceil_end = int(h * self.config.ceiling_zone_bottom)
        cv2.rectangle(vis, (0, y_ceil_start), (w, y_ceil_end), (255, 255, 0), 2)
        cv2.putText(vis, "Ceiling ROI", (10, y_ceil_end + 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Draw selected column
        if self.state.selected_column >= 0:
            x = self.state.selected_column
            cv2.line(vis, (x, y_start), (x, y_end), (255, 0, 255), 2)
        
        # Info panel
        info_y = 30
        cv2.putText(vis, f"Camera Tilt: {self.config.camera_tilt_angle:.1f} deg", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 20
        cv2.putText(vis, f"Obstacle Thresh: {self.config.obstacle_threshold*100:.0f} cm", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 20
        cv2.putText(vis, f"Max Step: {self.config.max_step_height*100:.0f} cm", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if analysis.get('fit_success'):
            info_y += 25
            slope = analysis.get('slope_cm_per_row', 0)
            cv2.putText(vis, f"Baseline Slope: {slope:.4f} cm/row", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        if analysis.get('obstacle_detected'):
            info_y += 25
            height = analysis.get('estimated_height_cm', 0)
            dist = analysis.get('obstacle_distance', 0)
            cv2.putText(vis, f"OBSTACLE: {height:.1f}cm @ {dist:.2f}m", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Draw obstacle marker
            obs_y = analysis.get('obstacle_y', 0) + y_start
            obs_x = analysis.get('obstacle_x', w//2)
            cv2.circle(vis, (obs_x, obs_y), 10, (0, 0, 255), 3)
            cv2.drawMarker(vis, (obs_x, obs_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        
        # Status bar
        status = "PAUSED" if self.state.paused else "LIVE"
        status_color = (0, 255, 255) if self.state.paused else (0, 255, 0)
        cv2.putText(vis, status, (w - 80, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        return vis
    
    def save_config(self):
        """Lưu config ra file."""
        try:
            self.config_file.parent.mkdir(parents=True, exist_ok=True)
            
            config_dict = {
                'robot_dimensions': {
                    'robot_height': self.config.robot_height,
                    'min_ground_clearance': self.config.min_ground_clearance,
                    'max_ground_clearance': self.config.max_ground_clearance,
                    'normal_ground_clearance': self.config.normal_ground_clearance,
                },
                'camera_setup': {
                    'camera_height': self.config.camera_height,
                    'camera_tilt_angle': self.config.camera_tilt_angle,
                    'camera_vfov': self.config.camera_vfov,
                },
                'detection_zones': {
                    'ceiling_zone_top': self.config.ceiling_zone_top,
                    'ceiling_zone_bottom': self.config.ceiling_zone_bottom,
                    'ground_zone_top': self.config.ground_zone_top,
                    'ground_zone_bottom': self.config.ground_zone_bottom,
                },
                'ceiling_detection': {
                    'ceiling_min_clearance': self.config.ceiling_min_clearance,
                    'ceiling_warning_distance': self.config.ceiling_warning_distance,
                },
                'ground_obstacle_detection': {
                    'ground_baseline_distance': self.config.ground_baseline_distance,
                    'obstacle_threshold': self.config.obstacle_threshold,
                    'max_step_height': self.config.max_step_height,
                },
                'processing': {
                    'depth_min_valid': self.config.depth_min_valid,
                    'depth_max_valid': self.config.depth_max_valid,
                    'smoothing_window': self.config.smoothing_window,
                }
            }
            
            with open(self.config_file, 'w') as f:
                json.dump(config_dict, f, indent=2)
            print(f"✅ Config saved: {self.config_file}")
            return True
        except Exception as e:
            print(f"❌ Save failed: {e}")
            return False
    
    def load_config(self):
        """Load config từ file."""
        if not self.config_file.exists():
            print(f"No config file found: {self.config_file}")
            return False
        
        try:
            with open(self.config_file, 'r') as f:
                data = json.load(f)
            
            if 'camera_setup' in data:
                cs = data['camera_setup']
                self.config.camera_height = cs.get('camera_height', self.config.camera_height)
                self.config.camera_tilt_angle = cs.get('camera_tilt_angle', self.config.camera_tilt_angle)
                self.config.camera_vfov = cs.get('camera_vfov', self.config.camera_vfov)
            
            if 'detection_zones' in data:
                dz = data['detection_zones']
                self.config.ceiling_zone_top = dz.get('ceiling_zone_top', self.config.ceiling_zone_top)
                self.config.ceiling_zone_bottom = dz.get('ceiling_zone_bottom', self.config.ceiling_zone_bottom)
                self.config.ground_zone_top = dz.get('ground_zone_top', self.config.ground_zone_top)
                self.config.ground_zone_bottom = dz.get('ground_zone_bottom', self.config.ground_zone_bottom)
            
            if 'ground_obstacle_detection' in data:
                god = data['ground_obstacle_detection']
                self.config.obstacle_threshold = god.get('obstacle_threshold', self.config.obstacle_threshold)
                self.config.max_step_height = god.get('max_step_height', self.config.max_step_height)
            
            if 'processing' in data:
                p = data['processing']
                self.config.depth_min_valid = p.get('depth_min_valid', self.config.depth_min_valid)
                self.config.depth_max_valid = p.get('depth_max_valid', self.config.depth_max_valid)
            
            print(f"✅ Config loaded: {self.config_file}")
            return True
        except Exception as e:
            print(f"❌ Load failed: {e}")
            return False


def nothing(x):
    pass


def mouse_callback(event, x, y, flags, param):
    """Mouse callback để chọn column."""
    calibrator = param
    if event == cv2.EVENT_LBUTTONDOWN:
        # Convert x to ROI coordinate
        h = calibrator._last_frame_height
        y_start = int(h * calibrator.config.ground_zone_top)
        y_end = int(h * calibrator.config.ground_zone_bottom)
        
        if y_start <= y <= y_end:
            calibrator.state.selected_column = x
            print(f"Selected column: {x}")


def main():
    print("=" * 70)
    print("  TERRAIN OBSTACLE CALIBRATION TOOL")
    print("  Công cụ Calibrate Phát hiện Vật cản với Đồ thị Debug")
    print("=" * 70)
    print("\nControls:")
    print("  Q       - Quit")
    print("  S       - Save config to JSON")
    print("  L       - Load config from JSON")
    print("  R       - Reset to defaults")
    print("  P       - Pause/Resume")
    print("  SPACE   - Capture frame for detailed analysis")
    print("  Click   - Select column for depth profile")
    print("\nTrackbars - Điều chỉnh các thông số detection")
    print("=" * 70)
    
    # Initialize camera
    camera = RealSenseCamera()
    if not camera.start():
        print("ERROR: Cannot start camera!")
        return
    
    # Initialize calibrator
    config = TerrainConfig(
        camera_tilt_angle=15.0,
        camera_height=0.20,
        ground_zone_top=0.55,
        ground_zone_bottom=0.90,
        obstacle_threshold=0.08,
        max_step_height=0.05,
    )
    
    calibrator = TerrainObstacleCalibrator(config)
    calibrator.load_config()  # Try to load existing config
    calibrator._last_frame_height = 480  # Will be updated
    
    # Create windows
    cv2.namedWindow("Main View", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Config", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Main View", 800, 600)
    cv2.resizeWindow("Config", 400, 400)
    
    # Mouse callback
    cv2.setMouseCallback("Main View", mouse_callback, calibrator)
    
    # Trackbars in Config window
    cv2.createTrackbar("Camera Tilt (deg)", "Config", 
                      int(config.camera_tilt_angle), 45, nothing)
    cv2.createTrackbar("Ground Top (%)", "Config", 
                      int(config.ground_zone_top * 100), 100, nothing)
    cv2.createTrackbar("Ground Bottom (%)", "Config", 
                      int(config.ground_zone_bottom * 100), 100, nothing)
    cv2.createTrackbar("Obstacle Thresh (cm)", "Config", 
                      int(config.obstacle_threshold * 100), 50, nothing)
    cv2.createTrackbar("Max Step (cm)", "Config", 
                      int(config.max_step_height * 100), 20, nothing)
    cv2.createTrackbar("Min Valid Depth (cm)", "Config", 
                      int(config.depth_min_valid * 100), 50, nothing)
    cv2.createTrackbar("Max Valid Depth (m*10)", "Config", 
                      int(config.depth_max_valid * 10), 100, nothing)
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    
    try:
        while True:
            # Update config from trackbars
            config.camera_tilt_angle = cv2.getTrackbarPos("Camera Tilt (deg)", "Config")
            config.ground_zone_top = cv2.getTrackbarPos("Ground Top (%)", "Config") / 100.0
            config.ground_zone_bottom = cv2.getTrackbarPos("Ground Bottom (%)", "Config") / 100.0
            config.obstacle_threshold = cv2.getTrackbarPos("Obstacle Thresh (cm)", "Config") / 100.0
            config.max_step_height = cv2.getTrackbarPos("Max Step (cm)", "Config") / 100.0
            config.depth_min_valid = cv2.getTrackbarPos("Min Valid Depth (cm)", "Config") / 100.0
            config.depth_max_valid = cv2.getTrackbarPos("Max Valid Depth (m*10)", "Config") / 10.0
            
            # Validation
            config.ground_zone_bottom = max(config.ground_zone_top + 0.1, config.ground_zone_bottom)
            
            # Get frames
            if not calibrator.state.paused:
                color_frame, depth_frame = camera.get_frames()
                if color_frame is None or depth_frame is None:
                    continue
                calibrator.state.captured_color = color_frame
                calibrator.state.captured_depth = depth_frame
            else:
                color_frame = calibrator.state.captured_color
                depth_frame = calibrator.state.captured_depth
                if color_frame is None:
                    continue
            
            calibrator._last_frame_height = color_frame.shape[0]
            frame_count += 1
            
            # Analyze
            analysis = calibrator.analyze_ground_detailed(depth_frame)
            
            # Update matplotlib plots
            calibrator.update_plots(analysis)
            
            # Create visualizations
            main_view = calibrator.create_main_view(color_frame, depth_frame, analysis)
            plot_view = calibrator.render_plots_to_image()
            
            # Resize plot view to fit
            plot_h = main_view.shape[0]
            plot_w = int(plot_view.shape[1] * plot_h / plot_view.shape[0])
            plot_view = cv2.resize(plot_view, (plot_w, plot_h))
            
            # FPS
            if time.time() - fps_time > 1.0:
                fps = frame_count / (time.time() - fps_time)
                fps_time = time.time()
                frame_count = 0
            
            cv2.putText(main_view, f"FPS: {fps:.1f}", (main_view.shape[1] - 100, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Show windows
            cv2.imshow("Main View", main_view)
            cv2.imshow("Depth Analysis", plot_view)
            
            # Config window - show current values
            config_info = np.zeros((400, 400, 3), dtype=np.uint8)
            y_offset = 30
            cv2.putText(config_info, "Current Config:", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            y_offset += 30
            cv2.putText(config_info, f"Camera Tilt: {config.camera_tilt_angle:.1f} deg", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            y_offset += 25
            cv2.putText(config_info, f"Ground Zone: {config.ground_zone_top:.2f} - {config.ground_zone_bottom:.2f}", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            y_offset += 25
            cv2.putText(config_info, f"Obstacle Thresh: {config.obstacle_threshold*100:.0f} cm", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            y_offset += 25
            cv2.putText(config_info, f"Max Step Height: {config.max_step_height*100:.0f} cm", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            y_offset += 25
            cv2.putText(config_info, f"Valid Depth: {config.depth_min_valid:.2f} - {config.depth_max_valid:.1f} m", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            if analysis.get('fit_success'):
                y_offset += 40
                cv2.putText(config_info, "Analysis Results:", (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                y_offset += 25
                cv2.putText(config_info, f"Baseline Slope: {analysis.get('slope_cm_per_row', 0):.4f} cm/row", 
                           (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                y_offset += 25
                cv2.putText(config_info, f"Valid Rows: {analysis.get('num_valid_rows', 0)}", 
                           (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                if analysis.get('obstacle_detected'):
                    y_offset += 25
                    cv2.putText(config_info, f"OBSTACLE: {analysis.get('estimated_height_cm', 0):.1f} cm", 
                               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Instructions
            y_offset = 320
            cv2.putText(config_info, "Keys: Q=Quit S=Save L=Load", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
            y_offset += 20
            cv2.putText(config_info, "R=Reset P=Pause Click=Select", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
            
            cv2.imshow("Config", config_info)
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                calibrator.save_config()
            elif key == ord('l'):
                calibrator.load_config()
                # Update trackbars
                cv2.setTrackbarPos("Camera Tilt (deg)", "Config", int(config.camera_tilt_angle))
                cv2.setTrackbarPos("Ground Top (%)", "Config", int(config.ground_zone_top * 100))
                cv2.setTrackbarPos("Ground Bottom (%)", "Config", int(config.ground_zone_bottom * 100))
                cv2.setTrackbarPos("Obstacle Thresh (cm)", "Config", int(config.obstacle_threshold * 100))
                cv2.setTrackbarPos("Max Step (cm)", "Config", int(config.max_step_height * 100))
            elif key == ord('r'):
                # Reset to defaults
                config.camera_tilt_angle = 15.0
                config.ground_zone_top = 0.55
                config.ground_zone_bottom = 0.90
                config.obstacle_threshold = 0.08
                config.max_step_height = 0.05
                config.depth_min_valid = 0.1
                config.depth_max_valid = 5.0
                # Update trackbars
                cv2.setTrackbarPos("Camera Tilt (deg)", "Config", int(config.camera_tilt_angle))
                cv2.setTrackbarPos("Ground Top (%)", "Config", int(config.ground_zone_top * 100))
                cv2.setTrackbarPos("Ground Bottom (%)", "Config", int(config.ground_zone_bottom * 100))
                cv2.setTrackbarPos("Obstacle Thresh (cm)", "Config", int(config.obstacle_threshold * 100))
                cv2.setTrackbarPos("Max Step (cm)", "Config", int(config.max_step_height * 100))
                print("🔄 Reset to defaults")
            elif key == ord('p') or key == ord(' '):
                calibrator.state.paused = not calibrator.state.paused
                status = "PAUSED" if calibrator.state.paused else "LIVE"
                print(f"📸 {status}")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        plt.close('all')
        print("Done.")


if __name__ == "__main__":
    main()
