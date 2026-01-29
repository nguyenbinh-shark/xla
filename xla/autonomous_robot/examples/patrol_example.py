#!/usr/bin/env python3
"""
Patrol Mode Example - Chế độ tuần tra phát hiện người lạ.

Usage:
    python examples/patrol_example.py                    # Chạy thật với robot
    python examples/patrol_example.py --sim              # Simulation (không UART)
    python examples/patrol_example.py --no-robot         # Chỉ test camera, không robot
"""

import cv2
import time
import argparse
import logging
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.modes import PatrolMode, PatrolConfig
from src.perception import RealSenseCamera
from src.communication import UARTController

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def alert_callback(intruder):
    """Callback khi phát hiện người lạ."""
    print(f"\n{'='*50}")
    print(f"🚨 CẢNH BÁO: PHÁT HIỆN NGƯỜI LẠ!")
    print(f"   Khoảng cách: {intruder.distance:.1f}m")
    print(f"   Độ tin cậy: {intruder.confidence:.1%}")
    print(f"   Thời gian: {time.strftime('%H:%M:%S')}")
    print(f"{'='*50}\n")


def main():
    parser = argparse.ArgumentParser(description='Patrol Mode Demo')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                        help='UART port')
    parser.add_argument('--sim', action='store_true',
                        help='Simulation mode - không gửi lệnh UART')
    parser.add_argument('--no-robot', action='store_true',
                        help='Chỉ test camera, không điều khiển robot')
    args = parser.parse_args()
    
    print("=" * 60)
    print("🛡️  PATROL MODE - CHẾ ĐỘ TUẦN TRA AI")
    print("=" * 60)
    
    # Configure patrol
    # LƯU Ý: Các thông số này ảnh hưởng đến tỷ lệ miss detection
    config = PatrolConfig(
        patrol_velocity=0.25,          # Tốc độ tuần tra 0.25 m/s
        rotate_yaw_rate=0.4,           # Tốc độ xoay 0.4 rad/s
        patrol_forward_time=10.0,      # Tiến 10 giây
        patrol_rotate_time=2.5,        # Xoay 2.5 giây
        detect_class="person",         # Phát hiện người
        min_confidence=0.35,           # Giảm từ 0.5 xuống 0.35 để bắt nhiều detection hơn
        min_box_area=1500,             # Giảm từ 3000 xuống 1500 để detect người ở xa hơn
        alert_distance=6.0,            # Tăng từ 4m lên 6m để phát hiện sớm hơn
        track_intruder=True,           # Theo dõi người lạ
        tracking_distance=2.0,         # Giữ khoảng cách 2m
        max_track_time=15.0,           # Theo dõi tối đa 15s
    )
    
    patrol = PatrolMode(config)
    patrol.set_alert_callback(alert_callback)
    
    # Initialize camera
    print("Khởi tạo camera...")
    camera = RealSenseCamera()
    if not camera.start():
        print("✗ Không thể khởi tạo camera!")
        return
    print("✓ Camera OK")
    
    # Initialize UART if not simulation
    uart = None
    if not args.sim and not args.no_robot:
        try:
            uart = UARTController(port=args.port)
            if not uart.connect():
                print(f"✗ Không thể kết nối {args.port}")
                uart = None
            else:
                print(f"✓ Kết nối UART: {args.port}")
                if not uart.enable_control():
                    print("✗ Không thể enable robot")
                    uart = None
                else:
                    print("✓ Robot ENABLED")
        except Exception as e:
            print(f"✗ Lỗi UART: {e}")
            uart = None
    
    if args.sim:
        print("⚠ CHẾ ĐỘ SIMULATION")
    if args.no_robot:
        print("⚠ CHẾ ĐỘ CHỈ CAMERA")
    
    print("\n" + "-" * 40)
    print("Nhấn 'q' để thoát")
    print("Nhấn 'r' để reset")
    print("Nhấn 'p' để pause/resume")
    print("-" * 40 + "\n")
    
    # Enable patrol mode
    patrol.enable()
    
    cv2.namedWindow("Patrol Mode", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Patrol Mode", 1280, 720)
    
    paused = False
    
    try:
        while True:
            # Get frame
            color_frame, depth_frame = camera.get_frames()
            
            if color_frame is None:
                time.sleep(0.01)
                continue
            
            # Process
            if not paused:
                output = patrol.process(color_frame, depth_frame)
                
                # Send to robot
                if uart and output.velocity != 0 or output.yaw_rate != 0:
                    uart.send_motion_command(output.velocity, output.yaw_rate)
                
                # Display frame with overlay
                if output.viz_frame is not None:
                    display_frame = output.viz_frame
                else:
                    display_frame = color_frame
            else:
                display_frame = color_frame
                cv2.putText(display_frame, "PAUSED", (50, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)
            
            cv2.imshow("Patrol Mode", display_frame)
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                patrol.reset()
                patrol.enable()
                print("Reset patrol mode")
            elif key == ord('p'):
                paused = not paused
                if paused and uart:
                    uart.send_motion_command(0, 0)  # Stop robot
                print("Paused" if paused else "Resumed")
    
    except KeyboardInterrupt:
        print("\n\n⚠ Dừng bởi người dùng")
    
    finally:
        # Cleanup
        print("\nĐang dừng...")
        
        if uart:
            uart.send_emergency_stop()
            uart.disable_control()
            uart.disconnect()
            print("✓ Đã ngắt UART")
        
        camera.stop()
        cv2.destroyAllWindows()
        
        # Print summary
        history = patrol.get_intruder_history()
        print("\n" + "=" * 40)
        print("📊 TÓM TẮT TUẦN TRA")
        print(f"   Số người phát hiện: {len(history)}")
        if history:
            print("   Chi tiết:")
            for i, intruder in enumerate(history, 1):
                t = time.strftime('%H:%M:%S', time.localtime(intruder.timestamp))
                print(f"   {i}. {t} - {intruder.distance:.1f}m ({intruder.confidence:.0%})")
        print("=" * 40)


if __name__ == "__main__":
    main()
