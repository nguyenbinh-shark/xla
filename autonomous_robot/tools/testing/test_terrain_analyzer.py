#!/usr/bin/env python3
"""
Test Terrain Analyzer - Phát hiện trần thấp và chướng ngại vật.

Hiển thị:
- Vùng scan trần (top frame)
- Vùng scan mặt đất (bottom frame)
- Đề xuất hành động: NORMAL, RAISE, LOWER, STOP

UART Buzzer:
- Trần thấp: Kêu 1 tiếng (beep)
- STOP (vật cản): Kêu liên tục

Usage:
    python tools/testing/test_terrain_analyzer.py [--mock-uart]
"""

import cv2
import sys
import time
import argparse
import numpy as np
import json
from pathlib import Path
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception import RealSenseCamera
from src.perception.terrain_analyzer import TerrainAnalyzer, TerrainConfig, ClearanceAction
from src.communication import UARTController, MockUARTController


def nothing(x):
    pass


class CalibrationManager:
    """
    Quản lý calibration cho Terrain Analyzer.
    Lưu/load config, update từ trackbar.
    """
    
    def __init__(self, config: TerrainConfig, window_name: str):
        self.config = config
        self.window_name = window_name
        self.config_file = Path(PROJECT_ROOT) / "data/calibration/terrain_config.json"
        
        # Backup original config
        self.default_config = TerrainConfig(
            robot_height=0.25,
            min_ground_clearance=0.02,
            max_ground_clearance=0.10,
            normal_ground_clearance=0.10,
            raised_ground_clearance=0.15,
            camera_height=0.20,
            camera_tilt_angle=15.0,
            camera_vfov=58.0,
            ceiling_zone_top=0.0,
            ceiling_zone_bottom=0.30,
            ceiling_min_clearance=0.5,
            ceiling_warning_distance=1.5,
            ground_zone_top=0.55,
            ground_zone_bottom=0.90,
            ground_baseline_distance=1.0,
            obstacle_threshold=0.08,
            max_step_height=0.05,
            depth_min_valid=0.1,
            depth_max_valid=5.0,
            smoothing_window=5
        )
        
        self.create_trackbars()
        self.load_config()
    
    def create_trackbars(self):
        """Tạo tất cả trackbar cho calibration."""
        
        # Camera parameters
        cv2.createTrackbar("Camera Tilt (deg)", self.window_name, 
                          int(self.config.camera_tilt_angle), 45, nothing)
        cv2.createTrackbar("Camera Height (cm)", self.window_name, 
                          int(self.config.camera_height * 100), 50, nothing)
        
        # Zone parameters (percentage * 100)
        cv2.createTrackbar("Ceiling Top (%)", self.window_name, 
                          int(self.config.ceiling_zone_top * 100), 50, nothing)
        cv2.createTrackbar("Ceiling Bottom (%)", self.window_name, 
                          int(self.config.ceiling_zone_bottom * 100), 80, nothing)
        cv2.createTrackbar("Ground Top (%)", self.window_name, 
                          int(self.config.ground_zone_top * 100), 100, nothing)
        cv2.createTrackbar("Ground Bottom (%)", self.window_name, 
                          int(self.config.ground_zone_bottom * 100), 100, nothing)
        
        # Ceiling detection
        cv2.createTrackbar("Ceil Warning (cm)", self.window_name, 
                          int(self.config.ceiling_warning_distance * 100), 300, nothing)
        cv2.createTrackbar("Ceil Min Clear (cm)", self.window_name, 
                          int(self.config.ceiling_min_clearance * 100), 100, nothing)
        
        # Ground obstacle detection  
        cv2.createTrackbar("Obstacle Thresh (cm)", self.window_name, 
                          int(self.config.obstacle_threshold * 100), 50, nothing)
        cv2.createTrackbar("Max Step (cm)", self.window_name, 
                          int(self.config.max_step_height * 100), 20, nothing)
        cv2.createTrackbar("Baseline Dist (cm)", self.window_name, 
                          int(self.config.ground_baseline_distance * 100), 300, nothing)
        
        # Processing parameters
        cv2.createTrackbar("Smooth Window", self.window_name, 
                          self.config.smoothing_window, 20, nothing)
        cv2.createTrackbar("Min Valid Depth (cm)", self.window_name, 
                          int(self.config.depth_min_valid * 100), 50, nothing)
        cv2.createTrackbar("Max Valid Depth (m)", self.window_name, 
                          int(self.config.depth_max_valid), 10, nothing)
    
    def update_config_from_trackbars(self):
        """Cập nhật config từ các trackbar."""
        
        # Camera
        self.config.camera_tilt_angle = cv2.getTrackbarPos("Camera Tilt (deg)", self.window_name)
        self.config.camera_height = cv2.getTrackbarPos("Camera Height (cm)", self.window_name) / 100.0
        
        # Zones
        self.config.ceiling_zone_top = cv2.getTrackbarPos("Ceiling Top (%)", self.window_name) / 100.0
        self.config.ceiling_zone_bottom = cv2.getTrackbarPos("Ceiling Bottom (%)", self.window_name) / 100.0
        self.config.ground_zone_top = cv2.getTrackbarPos("Ground Top (%)", self.window_name) / 100.0
        self.config.ground_zone_bottom = cv2.getTrackbarPos("Ground Bottom (%)", self.window_name) / 100.0
        
        # Ceiling
        self.config.ceiling_warning_distance = cv2.getTrackbarPos("Ceil Warning (cm)", self.window_name) / 100.0
        self.config.ceiling_min_clearance = cv2.getTrackbarPos("Ceil Min Clear (cm)", self.window_name) / 100.0
        
        # Ground obstacle
        self.config.obstacle_threshold = cv2.getTrackbarPos("Obstacle Thresh (cm)", self.window_name) / 100.0
        self.config.max_step_height = cv2.getTrackbarPos("Max Step (cm)", self.window_name) / 100.0
        self.config.ground_baseline_distance = cv2.getTrackbarPos("Baseline Dist (cm)", self.window_name) / 100.0
        
        # Processing
        self.config.smoothing_window = cv2.getTrackbarPos("Smooth Window", self.window_name)
        self.config.depth_min_valid = cv2.getTrackbarPos("Min Valid Depth (cm)", self.window_name) / 100.0
        self.config.depth_max_valid = cv2.getTrackbarPos("Max Valid Depth (m)", self.window_name)
        
        # Validation
        self.config.ceiling_zone_bottom = max(self.config.ceiling_zone_top + 0.1, self.config.ceiling_zone_bottom)
        self.config.ground_zone_bottom = max(self.config.ground_zone_top + 0.1, self.config.ground_zone_bottom)
        self.config.smoothing_window = max(1, self.config.smoothing_window)
        
    def update_trackbars_from_config(self):
        """Cập nhật trackbar từ config."""
        cv2.setTrackbarPos("Camera Tilt (deg)", self.window_name, int(self.config.camera_tilt_angle))
        cv2.setTrackbarPos("Camera Height (cm)", self.window_name, int(self.config.camera_height * 100))
        
        cv2.setTrackbarPos("Ceiling Top (%)", self.window_name, int(self.config.ceiling_zone_top * 100))
        cv2.setTrackbarPos("Ceiling Bottom (%)", self.window_name, int(self.config.ceiling_zone_bottom * 100))
        cv2.setTrackbarPos("Ground Top (%)", self.window_name, int(self.config.ground_zone_top * 100))
        cv2.setTrackbarPos("Ground Bottom (%)", self.window_name, int(self.config.ground_zone_bottom * 100))
        
        cv2.setTrackbarPos("Ceil Warning (cm)", self.window_name, int(self.config.ceiling_warning_distance * 100))
        cv2.setTrackbarPos("Ceil Min Clear (cm)", self.window_name, int(self.config.ceiling_min_clearance * 100))
        
        cv2.setTrackbarPos("Obstacle Thresh (cm)", self.window_name, int(self.config.obstacle_threshold * 100))
        cv2.setTrackbarPos("Max Step (cm)", self.window_name, int(self.config.max_step_height * 100))
        cv2.setTrackbarPos("Baseline Dist (cm)", self.window_name, int(self.config.ground_baseline_distance * 100))
        
        cv2.setTrackbarPos("Smooth Window", self.window_name, self.config.smoothing_window)
        cv2.setTrackbarPos("Min Valid Depth (cm)", self.window_name, int(self.config.depth_min_valid * 100))
        cv2.setTrackbarPos("Max Valid Depth (m)", self.window_name, int(self.config.depth_max_valid))
        
    def save_config(self):
        """Lưu config vào file JSON."""
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
        """Load config từ file JSON."""
        if not self.config_file.exists():
            print(f"No config file found, using defaults")
            return False
            
        try:
            with open(self.config_file, 'r') as f:
                data = json.load(f)
            
            # Load từ JSON vào config
            if 'robot_dimensions' in data:
                rd = data['robot_dimensions']
                self.config.robot_height = rd.get('robot_height', self.config.robot_height)
                self.config.min_ground_clearance = rd.get('min_ground_clearance', self.config.min_ground_clearance)
                self.config.max_ground_clearance = rd.get('max_ground_clearance', self.config.max_ground_clearance)
                self.config.normal_ground_clearance = rd.get('normal_ground_clearance', self.config.normal_ground_clearance)
            
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
            
            if 'ceiling_detection' in data:
                cd = data['ceiling_detection']
                self.config.ceiling_min_clearance = cd.get('ceiling_min_clearance', self.config.ceiling_min_clearance)
                self.config.ceiling_warning_distance = cd.get('ceiling_warning_distance', self.config.ceiling_warning_distance)
            
            if 'ground_obstacle_detection' in data:
                god = data['ground_obstacle_detection']
                self.config.ground_baseline_distance = god.get('ground_baseline_distance', self.config.ground_baseline_distance)
                self.config.obstacle_threshold = god.get('obstacle_threshold', self.config.obstacle_threshold)
                self.config.max_step_height = god.get('max_step_height', self.config.max_step_height)
            
            if 'processing' in data:
                p = data['processing']
                self.config.depth_min_valid = p.get('depth_min_valid', self.config.depth_min_valid)
                self.config.depth_max_valid = p.get('depth_max_valid', self.config.depth_max_valid)
                self.config.smoothing_window = p.get('smoothing_window', self.config.smoothing_window)
            
            # Update trackbars
            self.update_trackbars_from_config()
            print(f"✅ Config loaded: {self.config_file}")
            return True
            
        except Exception as e:
            print(f"❌ Load failed: {e}")
            return False
    
    def reset_to_default(self):
        """Reset về default config."""
        # Copy default values
        self.config.robot_height = self.default_config.robot_height
        self.config.min_ground_clearance = self.default_config.min_ground_clearance
        self.config.max_ground_clearance = self.default_config.max_ground_clearance
        self.config.normal_ground_clearance = self.default_config.normal_ground_clearance
        self.config.camera_height = self.default_config.camera_height
        self.config.camera_tilt_angle = self.default_config.camera_tilt_angle
        self.config.camera_vfov = self.default_config.camera_vfov
        self.config.ceiling_zone_top = self.default_config.ceiling_zone_top
        self.config.ceiling_zone_bottom = self.default_config.ceiling_zone_bottom
        self.config.ceiling_min_clearance = self.default_config.ceiling_min_clearance
        self.config.ceiling_warning_distance = self.default_config.ceiling_warning_distance
        self.config.ground_zone_top = self.default_config.ground_zone_top
        self.config.ground_zone_bottom = self.default_config.ground_zone_bottom
        self.config.ground_baseline_distance = self.default_config.ground_baseline_distance
        self.config.obstacle_threshold = self.default_config.obstacle_threshold
        self.config.max_step_height = self.default_config.max_step_height
        self.config.depth_min_valid = self.default_config.depth_min_valid
        self.config.depth_max_valid = self.default_config.depth_max_valid
        self.config.smoothing_window = self.default_config.smoothing_window
        
        # Update trackbars
        self.update_trackbars_from_config()
        print("🔄 Reset to default config")
    
    def get_info_text(self):
        """Tạo text hiển thị thông số config."""
        return f"""Camera: {self.config.camera_tilt_angle:.1f}° @ {self.config.camera_height*100:.0f}cm
Zones: C[{self.config.ceiling_zone_top:.2f}-{self.config.ceiling_zone_bottom:.2f}] G[{self.config.ground_zone_top:.2f}-{self.config.ground_zone_bottom:.2f}]
Thresh: Ceil={self.config.ceiling_warning_distance*100:.0f}cm Obs={self.config.obstacle_threshold*100:.0f}cm Step={self.config.max_step_height*100:.0f}cm
Smooth: {self.config.smoothing_window} frames"""


class BuzzerController:
    """
    Điều khiển còi qua UART.
    - B0: Tắt còi
    - B1: Beep (single beep)
    - B2: Alarm (continuous)
    """
    def __init__(self, uart):
        self.uart = uart
        self.is_beeping = False
        self.continuous_beep = False
        self.beep_start_time = 0
        self.beep_duration = 0.2  # 200ms cho single beep
        self.last_ceiling_beep = 0
        self.ceiling_beep_cooldown = 2.0  # Không kêu lại trong 2s
        
    def single_beep(self):
        """Kêu 1 tiếng ngắn (trần thấp)."""
        now = time.time()
        # Cooldown để không kêu liên tục
        if now - self.last_ceiling_beep < self.ceiling_beep_cooldown:
            return
        
        self.last_ceiling_beep = now
        self._send_beep()
        self.beep_start_time = now
        self.is_beeping = True
        self.continuous_beep = False
        print("🔔 BEEP! (Trần thấp)")
        
    def continuous_alarm(self):
        """Kêu liên tục (STOP - vật cản)."""
        if not self.continuous_beep:
            self._send_alarm()
            self.continuous_beep = True
            self.is_beeping = True
            print("🚨 ALARM! (Vật cản - STOP)")
    def stop_alarm(self):
        """Tắt còi."""
        if self.is_beeping or self.continuous_beep:
            self._send_off()
            self.is_beeping = False
            self.continuous_beep = False
    def update(self):
        """Gọi mỗi frame để xử lý timing."""
        if self.is_beeping and not self.continuous_beep:
            # Single beep - tắt sau duration
            if time.time() - self.beep_start_time > self.beep_duration:
                self._send_off()
                self.is_beeping = False
    def _send_beep(self):
        """Gửi lệnh beep qua UART."""
        if self.uart and self.uart.is_connected:
            self.uart._send_command_direct("B1")
    def _send_alarm(self):
        """Gửi lệnh alarm qua UART."""
        if self.uart and self.uart.is_connected:
            self.uart._send_command_direct("B2")
    
    def _send_off(self):
        """Tắt còi qua UART."""
        if self.uart and self.uart.is_connected:
            self.uart._send_command_direct("B0")


class HeightController:
    """
    Điều khiển chiều cao chân qua UART.
    Gửi lệnh Hxxx (height * 1000)
    VD: H60 = 0.06m = 6cm, H100 = 0.10m = 10cm, H150 = 0.15m = 15cm
    """
    def __init__(self, uart, normal_height=0.10, raised_height=0.15, lowered_height=0.06):
        self.uart = uart
        self.normal_height = normal_height
        self.raised_height = raised_height
        self.lowered_height = lowered_height
        self.current_height = normal_height
        self.last_sent_height = None
        self.height_change_threshold = 0.005  # 5mm threshold để tránh spam
        self.is_enabled = False
        
    def enable(self):
        """Enable motor control (E1)."""
        if self.uart and self.uart.is_connected and not self.is_enabled:
            self.uart._send_command_direct("E1")
            self.is_enabled = True
            print("✅ Motor control ENABLED (E1)")
    
    def disable(self):
        """Disable motor control (E0)."""
        if self.uart and self.uart.is_connected and self.is_enabled:
            self.uart._send_command_direct("E0")
            self.is_enabled = False
            print("⛔ Motor control DISABLED (E0)")
        
    def set_height(self, height_m: float):
        """
        Gửi chiều cao mới qua UART.
        
        Args:
            height_m: Chiều cao tính bằng mét
        """
        # Đảm bảo đã enable
        if not self.is_enabled:
            self.enable()
        
        # Kiểm tra xem có thay đổi đáng kể không
        if self.last_sent_height is not None:
            if abs(height_m - self.last_sent_height) < self.height_change_threshold:
                return  # Không gửi nếu thay đổi quá nhỏ
        
        self.current_height = height_m
        self._send_height(height_m)
        self.last_sent_height = height_m
        print(f"📏 UART Height: {height_m*100:.1f}cm (H{int(height_m*1000)})")
    
    def set_normal(self):
        """Đặt chiều cao bình thường (8cm)."""
        self.set_height(self.normal_height)
    
    def set_raised(self):
        """Đặt chiều cao nâng cao (13cm)."""
        self.set_height(self.raised_height)    
    def set_lowered(self):
        """Đặt chiều cao hạ thấp (6cm) - trần thấp."""
        self.set_height(self.lowered_height)    
    def _send_height(self, height_m: float):
        """Gửi lệnh chiều cao qua UART."""
        if self.uart and self.uart.is_connected:
            # Format: Hxxx với xxx = height * 1000
            scaled = int(height_m * 1000)
            cmd = f"H{scaled}"
            self.uart._send_command_direct(cmd)


def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="Terrain Analyzer Test with UART Buzzer")
    parser.add_argument("--mock-uart", action="store_true", help="Use mock UART (no hardware)")
    parser.add_argument("--no-uart", action="store_true", help="Disable UART completely")
    args = parser.parse_args()
    
    print("=" * 60)
    print("  TERRAIN ANALYZER TEST + UART BUZZER")
    print("  Phát hiện trần thấp & chướng ngại vật")
    print("=" * 60)
    print("Controls:")
    print("  Q - Quit")
    print("  R - Reset analyzer")
    print("  S - Save frame")
    print("  B - Test beep")
    print("  C - Save calibration")
    print("  L - Load calibration")
    print("  D - Reset to defaults")
    print("  Trackbars - Điều chỉnh thông số")
    print("-" * 60)
    print("Buzzer behavior:")
    print("  🔔 Trần thấp (LOWER) → Kêu 1 tiếng")
    print("  🚨 Vật cản (STOP) → Kêu liên tục")
    print("Height control via UART:")
    print("  📏 NORMAL → H100 (10cm)")
    print("  📏 LOWER (trần thấp) → H60 (6cm)")
    print("  📏 RAISE (vật cản) → H150 (15cm)")
    print("=" * 60)
    
    # Initialize UART
    uart = None
    buzzer = None
    height_ctrl = None
    if not args.no_uart:
        try:
            if args.mock_uart:
                uart = MockUARTController()
                print("Using MOCK UART (no hardware)")
            else:
                uart = UARTController()
            
            if uart.connect():
                buzzer = BuzzerController(uart)
                height_ctrl = HeightController(
                    uart, 
                    normal_height=0.10,  # 10cm bình thường
                    raised_height=0.15,  # 15cm khi có vật cản
                    lowered_height=0.06  # 6cm khi trần thấp
                )
                print(f"UART connected: {uart.port if hasattr(uart, 'port') else 'mock'}")
            else:
                print("WARNING: UART connection failed, buzzer disabled")
                uart = None
        except Exception as e:
            print(f"WARNING: UART init failed: {e}, buzzer disabled")
            uart = None
    else:
        print("UART disabled (--no-uart)")
    
    # Initialize camera
    camera = RealSenseCamera()
    if not camera.start():
        print("ERROR: Cannot start camera!")
        if uart:
            uart.disconnect()
        return
    
    config = TerrainConfig(
        # Robot dimensions
        robot_height=0.25,              # Chiều cao robot 25cm
        min_ground_clearance=0.02,      # Gầm tối thiểu 2cm
        max_ground_clearance=0.10,      # Gầm tối đa 10cm
        normal_ground_clearance=0.10,   # Gầm bình thường 10cm
        raised_ground_clearance=0.15,   # Gầm khi có vật cản 15cm
        
        # Camera mounting (quan trọng!)
        camera_height=0.20,             # Camera cao 20cm so với mặt đất
        camera_tilt_angle=15.0,         # Nghiêng xuống 15 độ
        
        # Detection thresholds
        ceiling_min_clearance=0.4,      # Cần ít nhất 40cm headroom
        ceiling_warning_distance=0.8,   # Cảnh báo khi trần < 80cm
        max_step_height=0.05,           # Bước qua được vật cao 5cm
    )
    
    analyzer = TerrainAnalyzer(config)
    
    # Create window
    window_name = "Terrain Analyzer"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    
    # Calibration manager
    calib = CalibrationManager(config, window_name)
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    
    # Simulated current height
    current_height = config.normal_ground_clearance
    
    try:
        while True:
            color_frame, depth_frame = camera.get_frames()
            
            if color_frame is None or depth_frame is None:
                continue
            
            frame_count += 1
            
            # Update config from trackbars
            calib.update_config_from_trackbars()
            
            # Analyze
            result = analyzer.analyze(depth_frame, color_frame)
            
            # === BUZZER CONTROL ===
            if buzzer:
                buzzer.update()  # Update timing for single beep
                
                if result.action == ClearanceAction.STOP:
                    # Vật cản STOP → kêu liên tục
                    buzzer.continuous_alarm()
                elif result.action == ClearanceAction.LOWER:
                    # Trần thấp → kêu 1 tiếng
                    buzzer.stop_alarm()  # Tắt alarm liên tục nếu có
                    buzzer.single_beep()
                else:
                    # NORMAL hoặc RAISE → tắt còi
                    buzzer.stop_alarm()
            
            # === HEIGHT CONTROL via UART ===
            if height_ctrl:
                if result.action == ClearanceAction.RAISE:
                    # Có vật cản → nâng lên 15cm
                    height_ctrl.set_raised()
                elif result.action == ClearanceAction.LOWER:
                    # Trần thấp → hạ xuống 6cm
                    height_ctrl.set_lowered()
                elif result.action == ClearanceAction.STOP:
                    # STOP → giữ nguyên (không gửi lệnh mới)
                    pass
                else:
                    # NORMAL → về 10cm
                    height_ctrl.set_normal()
                
                # Cập nhật current_height từ controller
                current_height = height_ctrl.current_height
            else:
                # Simulate height adjustment (không có UART)
                if result.action != ClearanceAction.STOP:
                    height_diff = result.recommended_height - current_height
                    current_height += height_diff * 0.1  # Smooth 10%
            
            # Visualize
            vis = analyzer.visualize(color_frame, depth_frame, result)
            
            # Add depth visualization side by side
            depth_colored = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_frame, alpha=50), 
                cv2.COLORMAP_JET
            )
            
            # Draw zones on depth
            h, w = depth_colored.shape[:2]
            y_ceil_end = int(h * config.ceiling_zone_bottom)
            y_ground_start = int(h * config.ground_zone_top)
            cv2.line(depth_colored, (0, y_ceil_end), (w, y_ceil_end), (255, 255, 255), 2)
            cv2.line(depth_colored, (0, y_ground_start), (w, y_ground_start), (255, 255, 255), 2)
            
            # Combine views
            combined = cv2.hconcat([vis, depth_colored])
            
            # FPS
            if time.time() - fps_time > 1.0:
                fps = frame_count / (time.time() - fps_time)
                fps_time = time.time()
                frame_count = 0
            
            cv2.putText(combined, f"FPS: {fps:.1f}", (10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Current height indicator
            height_text = f"Current Height: {current_height*100:.1f}cm"
            cv2.putText(combined, height_text, (combined.shape[1] - 250, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Config info overlay (top-left)
            config_info = calib.get_info_text()
            y_offset = 50
            for i, line in enumerate(config_info.split('\n')):
                cv2.putText(combined, line, (10, y_offset + i*20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            
            cv2.imshow("Terrain Analyzer", combined)
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                analyzer.reset()
                current_height = config.normal_ground_clearance
                if buzzer:
                    buzzer.stop_alarm()
                print("Reset!")
            elif key == ord('s'):
                filename = f"terrain_{int(time.time())}.jpg"
                cv2.imwrite(filename, combined)
                print(f"Saved: {filename}")
            elif key == ord('b'):
                # Test beep
                if buzzer:
                    buzzer.single_beep()
                    print("Test beep!")
            elif key == ord('c'):
                # Save calibration
                calib.save_config()
            elif key == ord('l'):
                # Load calibration
                calib.load_config()
                analyzer.reset()  # Reset analyzer với config mới
            elif key == ord('d'):
                # Reset to defaults
                calib.reset_to_default()
                analyzer.reset()
    
    finally:
        # Cleanup
        if height_ctrl:
            height_ctrl.disable()  # E0 - disable motor control
        if buzzer:
            buzzer.stop_alarm()
        if uart:
            uart.disconnect()
        camera.stop()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
