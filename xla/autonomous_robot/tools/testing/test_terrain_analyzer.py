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
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception import RealSenseCamera
from src.perception.terrain_analyzer import TerrainAnalyzer, TerrainConfig, ClearanceAction
from src.communication import UARTController, MockUARTController


def nothing(x):
    pass


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
    print("  Trackbars - Điều chỉnh thông số")
    print("-" * 60)
    print("Buzzer behavior:")
    print("  🔔 Trần thấp (LOWER) → Kêu 1 tiếng")
    print("  🚨 Vật cản (STOP) → Kêu liên tục")
    print("=" * 60)
    
    # Initialize UART
    uart = None
    buzzer = None
    if not args.no_uart:
        try:
            if args.mock_uart:
                uart = MockUARTController()
                print("Using MOCK UART (no hardware)")
            else:
                uart = UARTController()
            
            if uart.connect():
                buzzer = BuzzerController(uart)
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
        normal_ground_clearance=0.05,   # Gầm bình thường 5cm
        
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
    cv2.namedWindow("Terrain Analyzer", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Terrain Analyzer", 1280, 720)
    
    # Trackbars
    cv2.createTrackbar("Ceil Warning (cm)", "Terrain Analyzer", 
                       int(config.ceiling_warning_distance * 100), 200, nothing)
    cv2.createTrackbar("Max Step (cm)", "Terrain Analyzer", 
                       int(config.max_step_height * 100), 20, nothing)
    cv2.createTrackbar("Obstacle Thresh (cm)", "Terrain Analyzer", 
                       int(config.obstacle_threshold * 100), 50, nothing)
    
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
            config.ceiling_warning_distance = cv2.getTrackbarPos("Ceil Warning (cm)", "Terrain Analyzer") / 100.0
            config.max_step_height = cv2.getTrackbarPos("Max Step (cm)", "Terrain Analyzer") / 100.0
            config.obstacle_threshold = cv2.getTrackbarPos("Obstacle Thresh (cm)", "Terrain Analyzer") / 100.0
            
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
            
            # Simulate height adjustment
            if result.action != ClearanceAction.STOP:
                # Smooth transition to recommended height
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
    
    finally:
        # Cleanup
        if buzzer:
            buzzer.stop_alarm()
        if uart:
            uart.disconnect()
        camera.stop()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
