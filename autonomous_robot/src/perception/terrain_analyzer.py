"""
Terrain Analyzer - Phân tích địa hình để điều chỉnh độ cao gầm xe.

Chức năng:
1. Phát hiện trần/giới hạn chiều cao phía trên
2. Phát hiện chướng ngại vật nhỏ trên mặt đất
3. Đề xuất hành động: LOWER (hạ gầm), RAISE (nâng gầm), NORMAL

Sử dụng với RealSense depth camera.
"""

import cv2
import numpy as np
import logging
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import Enum, auto

logger = logging.getLogger(__name__)


class ClearanceAction(Enum):
    """Hành động điều chỉnh độ cao gầm."""
    NORMAL = auto()      # Giữ nguyên
    RAISE = auto()       # Nâng gầm (vượt chướng ngại vật)
    LOWER = auto()       # Hạ gầm (qua trần thấp)
    STOP = auto()        # Dừng (không thể qua)


@dataclass
class TerrainAnalysisResult:
    """Kết quả phân tích địa hình."""
    # Ceiling detection
    ceiling_detected: bool = False
    ceiling_distance: float = -1.0      # Khoảng cách đến trần (m)
    ceiling_clearance_ok: bool = True   # Có đủ không gian không
    
    # Ground obstacle detection  
    ground_obstacle: bool = False
    obstacle_height: float = 0.0        # Chiều cao chướng ngại vật (m)
    obstacle_distance: float = -1.0     # Khoảng cách đến chướng ngại vật (m)
    can_step_over: bool = True          # Có thể bước qua không
    
    # Recommended action
    action: ClearanceAction = ClearanceAction.NORMAL
    recommended_height: float = 0.05    # Độ cao gầm đề xuất (m)
    
    # Confidence
    confidence: float = 0.0
    message: str = ""


@dataclass 
class TerrainConfig:
    """Cấu hình phân tích địa hình."""
    # Robot dimensions
    robot_height: float = 0.3           # Chiều cao robot khi đứng bình thường (m)
    min_ground_clearance: float = 0.07  # Độ cao gầm tối thiểu - hạ gầm (m) = 7cm
    max_ground_clearance: float = 0.18  # Độ cao gầm tối đa - nâng gầm (m) = 18cm
    normal_ground_clearance: float = 0.08  # Độ cao gầm bình thường (m) = 8cm
    raised_ground_clearance: float = 0.13  # Độ cao gầm khi có vật cản (m) = 13cm
    
    # Camera mounting (calibrated values)
    camera_height: float = 0.20         # Chiều cao camera so với mặt đất (m)
    camera_tilt_angle: float = 14.0     # Góc so với phương ngang (độ), dương = cúi xuống
    camera_vfov: float = 58.0           # Vertical FOV của camera (độ) - RealSense D435
    
    # Ceiling detection zones (ratio of frame height)
    ceiling_zone_top: float = 0.0       # Bắt đầu từ top
    ceiling_zone_bottom: float = 0.30   # Đến 30% frame height
    ceiling_min_clearance: float = 0.5  # Khoảng cách tối thiểu đến trần (m)
    ceiling_warning_distance: float = 1.5  # Cảnh báo khi trần < 1.5m
    
    # Ground obstacle detection zones (calibrated values)
    ground_zone_top: float = 0.60       # Từ 60% frame height
    ground_zone_bottom: float = 1.0     # Đến 100% (bottom)
    ground_baseline_distance: float = 1.0  # Khoảng cách baseline mặt đất (m)
    obstacle_threshold: float = 0.03    # Chênh lệch depth để coi là obstacle (m) = 3cm
    max_step_height: float = 0.05       # Chiều cao tối đa có thể bước qua (m) = 5cm
    
    # Processing (calibrated values)
    depth_min_valid: float = 0.1
    depth_max_valid: float = 2.5        # Max depth 2.5m
    smoothing_window: int = 5           # Số frame để smooth kết quả


class TerrainAnalyzer:
    """
    Phân tích địa hình từ depth camera.
    
    Sử dụng:
        analyzer = TerrainAnalyzer()
        result = analyzer.analyze(depth_frame)
        
        if result.action == ClearanceAction.LOWER:
            robot.set_height(result.recommended_height)
    """
    
    def __init__(self, config: Optional[TerrainConfig] = None):
        """Khởi tạo terrain analyzer."""
        self.config = config or TerrainConfig()
        
        # Smoothing buffers
        self._ceiling_history = []
        self._obstacle_history = []
        
        logger.info("TerrainAnalyzer initialized")
    
    def analyze(
        self, 
        depth_frame: np.ndarray,
        color_frame: Optional[np.ndarray] = None
    ) -> TerrainAnalysisResult:
        """
        Phân tích depth frame để detect địa hình.
        
        Args:
            depth_frame: Depth image (meters)
            color_frame: Optional color frame for visualization
            
        Returns:
            TerrainAnalysisResult với action đề xuất
        """
        if depth_frame is None or depth_frame.size == 0:
            return TerrainAnalysisResult(
                message="Invalid depth frame",
                confidence=0.0
            )
        
        h, w = depth_frame.shape[:2]
        
        # 1. Analyze ceiling zone
        ceiling_result = self._analyze_ceiling(depth_frame, h, w)
        
        # 2. Analyze ground obstacles
        ground_result = self._analyze_ground(depth_frame, h, w)
        
        # 3. Determine action
        result = self._determine_action(ceiling_result, ground_result)
        
        return result
    
    def _analyze_ceiling(
        self, 
        depth_frame: np.ndarray, 
        h: int, 
        w: int
    ) -> dict:
        """Phân tích vùng trần/phía trên."""
        # Extract ceiling zone
        y_start = int(h * self.config.ceiling_zone_top)
        y_end = int(h * self.config.ceiling_zone_bottom)
        x_margin = int(w * 0.1)  # Bỏ 10% hai bên
        
        ceiling_zone = depth_frame[y_start:y_end, x_margin:w-x_margin]
        
        # Filter valid depths
        valid_mask = (
            (ceiling_zone > self.config.depth_min_valid) & 
            (ceiling_zone < self.config.depth_max_valid)
        )
        
        valid_depths = ceiling_zone[valid_mask]
        
        if valid_depths.size < 100:  # Không đủ data
            return {
                'detected': False,
                'distance': -1.0,
                'clearance_ok': True
            }
        
        # Tính khoảng cách trần (dùng percentile thấp để lấy điểm gần nhất)
        ceiling_distance = float(np.percentile(valid_depths, 10))
        
        # Smooth với history
        self._ceiling_history.append(ceiling_distance)
        if len(self._ceiling_history) > self.config.smoothing_window:
            self._ceiling_history = self._ceiling_history[-self.config.smoothing_window:]
        
        smoothed_distance = np.median(self._ceiling_history)
        
        # Check clearance
        clearance_ok = smoothed_distance >= self.config.ceiling_min_clearance
        detected = smoothed_distance < self.config.ceiling_warning_distance
        
        return {
            'detected': detected,
            'distance': smoothed_distance,
            'clearance_ok': clearance_ok
        }
    
    def _analyze_ground(
        self,
        depth_frame: np.ndarray,
        h: int,
        w: int
    ) -> dict:
        """Phân tích vùng mặt đất để tìm chướng ngại vật (baseline là hàm tuyến tính theo y)."""
        # Extract ground zone
        y_start = int(h * self.config.ground_zone_top)
        y_end = int(h * self.config.ground_zone_bottom)
        ground_zone = depth_frame[y_start:y_end, :]
        roi_h, roi_w = ground_zone.shape[:2]

        # Tính baseline depth cho từng hàng (median theo hàng)
        row_baseline = np.zeros(roi_h)
        for i in range(roi_h):
            row = ground_zone[i, :]
            valid = (row > self.config.depth_min_valid) & (row < self.config.depth_max_valid)
            if np.sum(valid) > 10:
                row_baseline[i] = np.median(row[valid])
            else:
                row_baseline[i] = np.nan

        # Loại bỏ các hàng không hợp lệ
        valid_rows = ~np.isnan(row_baseline)
        if np.sum(valid_rows) < 5:
            return {
                'obstacle': False,
                'height': 0.0,
                'distance': -1.0,
                'can_step_over': True
            }

        # Fit baseline tuyến tính theo y (nếu đủ hàng hợp lệ)
        y_idx = np.arange(roi_h)[valid_rows]
        baseline_vals = row_baseline[valid_rows]
        fit = np.polyfit(y_idx, baseline_vals, 1)
        baseline_line = np.polyval(fit, np.arange(roi_h))

        # Tìm các điểm có depth nhỏ hơn baseline tương ứng nhiều (vật cản)
        obstacle_mask = np.zeros_like(ground_zone, dtype=bool)
        for i in range(roi_h):
            row = ground_zone[i, :]
            valid = (row > self.config.depth_min_valid) & (row < self.config.depth_max_valid)
            # So sánh từng điểm với baseline tại hàng đó
            obstacle_mask[i, :] = valid & (row < (baseline_line[i] - self.config.obstacle_threshold))

        if not np.any(obstacle_mask):
            # Không có vật cản
            baseline_median = float(np.median(baseline_vals))
            return {
                'obstacle': False,
                'height': 0.0,
                'distance': baseline_median,
                'can_step_over': True
            }

        # Lấy depth nhỏ nhất trong các điểm vật cản
        obstacle_depths = ground_zone[obstacle_mask]
        obstacle_distance = float(np.min(obstacle_depths))

        # Chọn y (hàng) của điểm vật cản gần nhất
        y_obstacle, _ = np.unravel_index(np.argmin(np.where(obstacle_mask, ground_zone, np.inf)), ground_zone.shape)
        baseline_at_obstacle = baseline_line[y_obstacle]
        depth_diff = baseline_at_obstacle - obstacle_distance

        # Góc trung bình của ground zone so với horizon
        avg_ground_angle = self.config.camera_tilt_angle + self.config.camera_vfov * 0.25
        angle_rad = np.radians(avg_ground_angle)
        estimated_height = depth_diff * np.sin(angle_rad)
        estimated_height = min(0.3, max(0, estimated_height))  # Clamp

        # Smooth
        self._obstacle_history.append(estimated_height)
        if len(self._obstacle_history) > self.config.smoothing_window:
            self._obstacle_history = self._obstacle_history[-self.config.smoothing_window:]
        smoothed_height = np.median(self._obstacle_history)
        can_step_over = smoothed_height <= self.config.max_step_height

        return {
            'obstacle': True,
            'height': smoothed_height,
            'distance': obstacle_distance,
            'can_step_over': can_step_over
        }
    
    def _determine_action(
        self, 
        ceiling: dict, 
        ground: dict
    ) -> TerrainAnalysisResult:
        """Xác định hành động dựa trên phân tích."""
        result = TerrainAnalysisResult()
        
        # Fill in analysis data
        result.ceiling_detected = ceiling['detected']
        result.ceiling_distance = ceiling['distance']
        result.ceiling_clearance_ok = ceiling['clearance_ok']
        
        result.ground_obstacle = ground['obstacle']
        result.obstacle_height = ground['height']
        result.obstacle_distance = ground['distance']
        result.can_step_over = ground['can_step_over']
        
        # Priority logic:
        # 1. Cả trần thấp VÀ vật cản → STOP (không thể hạ gầm để qua trần, không thể nâng gầm để qua vật cản)
        # 2. Trần quá thấp → LOWER
        # 3. Vật cản quá cao → STOP
        # 4. Vật cản có thể bước qua → RAISE
        # 5. Trần thấp nhưng qua được → LOWER
        # 6. Bình thường → NORMAL
        
        # Case 0: CẢ trần thấp VÀ vật cản trên mặt đất → STOP
        if ceiling['detected'] and not ceiling['clearance_ok'] and ground['obstacle']:
            result.action = ClearanceAction.STOP
            result.recommended_height = self.config.normal_ground_clearance
            result.message = f"⛔ Trần thấp ({ceiling['distance']:.2f}m) + Vật cản ({ground['height']*100:.1f}cm) - DỪNG!"
            result.confidence = 0.95
        
        # Case 1: Ceiling too low (không có vật cản)
        elif ceiling['detected'] and not ceiling['clearance_ok']:
            result.action = ClearanceAction.LOWER
            result.recommended_height = self.config.min_ground_clearance
            result.message = f"⚠️ Trần thấp ({ceiling['distance']:.2f}m) - HẠ GẦM!"
            result.confidence = 0.9
            
        # Case 2: Ground obstacle too high (không thể bước qua)
        elif ground['obstacle'] and not ground['can_step_over']:
            result.action = ClearanceAction.STOP
            result.recommended_height = self.config.normal_ground_clearance
            result.message = f"⛔ Chướng ngại quá cao ({ground['height']*100:.1f}cm) - DỪNG/TRÁNH!"
            result.confidence = 0.9
        
        # Case 3: Ground obstacle that can be stepped over
        elif ground['obstacle'] and ground['can_step_over']:
            # Nâng gầm lên mức cố định 13cm khi phát hiện vật cản
            result.action = ClearanceAction.RAISE
            result.recommended_height = self.config.raised_ground_clearance
            result.message = f"Chướng ngại vật - NÂNG GẦM lên {self.config.raised_ground_clearance*100:.0f}cm"
            result.confidence = 0.85
            
        # Case 4: Ceiling low but passable (không có vật cản)
        elif ceiling['detected']:
            # Tính độ cao cần hạ
            available_space = ceiling['distance'] - 0.1  # Buffer 10cm
            needed_height = max(
                self.config.min_ground_clearance,
                min(self.config.normal_ground_clearance, available_space - self.config.robot_height)
            )
            
            if needed_height < self.config.normal_ground_clearance:
                result.action = ClearanceAction.LOWER
                result.recommended_height = needed_height
                result.message = f"Trần {ceiling['distance']:.2f}m - giảm gầm xuống {needed_height*100:.0f}cm"
                result.confidence = 0.8
            else:
                result.action = ClearanceAction.NORMAL
                result.recommended_height = self.config.normal_ground_clearance
                result.message = "OK - đủ không gian"
                result.confidence = 0.7
        
        # Case 5: All clear
        else:
            result.action = ClearanceAction.NORMAL
            result.recommended_height = self.config.normal_ground_clearance
            result.message = "✓ Địa hình bình thường"
            result.confidence = 0.5
        
        return result
    
    def visualize(
        self, 
        color_frame: np.ndarray, 
        depth_frame: np.ndarray,
        result: TerrainAnalysisResult
    ) -> np.ndarray:
        """Tạo visualization với các vùng phân tích."""
        vis = color_frame.copy()
        h, w = vis.shape[:2]
        
        # Draw ceiling zone
        y_ceil_start = int(h * self.config.ceiling_zone_top)
        y_ceil_end = int(h * self.config.ceiling_zone_bottom)
        ceil_color = (0, 0, 255) if result.ceiling_detected else (0, 255, 0)
        cv2.rectangle(vis, (0, y_ceil_start), (w, y_ceil_end), ceil_color, 2)
        cv2.putText(vis, f"CEILING: {result.ceiling_distance:.2f}m", 
                   (10, y_ceil_end + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ceil_color, 1)
        
        # Draw ground zone
        y_ground_start = int(h * self.config.ground_zone_top)
        y_ground_end = int(h * self.config.ground_zone_bottom)
        ground_color = (0, 0, 255) if result.ground_obstacle else (0, 255, 0)
        cv2.rectangle(vis, (0, y_ground_start), (w, y_ground_end), ground_color, 2)
        cv2.putText(vis, f"GROUND: {result.obstacle_height*100:.1f}cm @ {result.obstacle_distance:.2f}m", 
                   (10, y_ground_start - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ground_color, 1)
        
        # Action indicator
        action_colors = {
            ClearanceAction.NORMAL: (0, 255, 0),
            ClearanceAction.RAISE: (0, 255, 255),
            ClearanceAction.LOWER: (255, 165, 0),
            ClearanceAction.STOP: (0, 0, 255),
        }
        action_color = action_colors.get(result.action, (255, 255, 255))
        
        # Status bar
        cv2.rectangle(vis, (0, h - 60), (w, h), (40, 40, 40), -1)
        cv2.putText(vis, f"ACTION: {result.action.name}", 
                   (10, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, action_color, 2)
        cv2.putText(vis, result.message, 
                   (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(vis, f"Height: {result.recommended_height*100:.0f}cm", 
                   (w - 150, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return vis
    
    def reset(self):
        """Reset history buffers."""
        self._ceiling_history = []
        self._obstacle_history = []
