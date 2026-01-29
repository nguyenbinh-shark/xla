#!/usr/bin/env python3
"""
Object Tracking Example - Gimbal-style tracking với khóa mục tiêu.

Tính năng:
- Lock target: Khóa vào một vật thể cụ thể
- Gimbal tracking: Xoay robot để giữ mục tiêu ở center màn hình
- Distance keeping: Tiến/lùi để giữ khoảng cách với mục tiêu

Usage:
    python examples/object_tracking_example.py                    # Track người
    python examples/object_tracking_example.py --target car       # Track xe
    python examples/object_tracking_example.py --sim              # Simulation mode
    python examples/object_tracking_example.py --distance 2.0     # Giữ khoảng cách 2m
"""

import cv2
import time
import argparse
import logging
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.modes import ObjectTrackingMode, ObjectTrackingConfig
from src.perception import RealSenseCamera
from src.communication import UARTController

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def nothing(x):
    """Callback for trackbar."""
    pass


def main():
    parser = argparse.ArgumentParser(description='Object Tracking - Gimbal Style')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                        help='UART port')
    parser.add_argument('--target', type=str, default='person',
                        help='Loại vật thể cần track (person, car, bottle, cell phone...)')
    parser.add_argument('--distance', type=float, default=1.5,
                        help='Khoảng cách mong muốn (m)')
    parser.add_argument('--sim', action='store_true',
                        help='Simulation mode - không gửi lệnh UART')
    parser.add_argument('--no-robot', action='store_true',
                        help='Chỉ test camera, không điều khiển robot')
    args = parser.parse_args()
    
    print("=" * 60)
    print("🎯 OBJECT TRACKING - GIMBAL STYLE")
    print("=" * 60)
    print(f"Target: {args.target}")
    print(f"Distance: {args.distance}m")
    print("=" * 60)
    
    # Configure tracking
    config = ObjectTrackingConfig(
        target_class=args.target,           # Loại vật thể cần track
        target_distance=args.distance,      # Khoảng cách mong muốn
        
        # Speed control
        max_speed=0.5,                      # Tốc độ tối đa khi tiến/lùi
        min_speed=0.0,
        approach_speed=0.3,                 # Tốc độ tiếp cận
        
        # Gimbal-style steering (quan trọng!)
        steering_gain=2.5,                  # Gain cao = xoay nhanh hơn để bám target
        max_yaw_rate=1.2,                   # Tốc độ xoay tối đa
        
        # Distance control
        distance_deadband=0.3,              # Vùng chết 30cm (không cần di chuyển)
        min_safe_distance=0.8,              # Khoảng cách an toàn tối thiểu
        max_tracking_distance=6.0,          # Track tối đa 6m
        
        # Search behavior
        search_yaw_rate=0.5,                # Tốc độ tìm kiếm khi mất target
        max_frames_lost=90,                 # ~3 giây ở 30fps
        
        # Detection
        min_confidence=0.4,                 # Confidence thấp hơn để dễ detect
        min_box_area=1500,                  # Box nhỏ hơn để track từ xa
    )
    
    tracker = ObjectTrackingMode(config)
    
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
    print("CONTROLS:")
    print("  Q       - Thoát")
    print("  SPACE   - Pause/Resume")
    print("  R       - Reset tracker")
    print("  +/-     - Tăng/giảm khoảng cách mục tiêu")
    print("  1-9     - Chọn target class nhanh:")
    print("            1=person, 2=car, 3=bottle")
    print("            4=cell phone, 5=cup, 6=chair")
    print("-" * 40 + "\n")
    
    # Create window with trackbars
    cv2.namedWindow("Object Tracking", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Object Tracking", 1280, 800)
    
    # Trackbars for realtime tuning
    cv2.createTrackbar("Steering Gain", "Object Tracking", 
                       int(config.steering_gain * 10), 50, nothing)
    cv2.createTrackbar("Target Dist (cm)", "Object Tracking", 
                       int(config.target_distance * 100), 500, nothing)
    cv2.createTrackbar("Max Speed", "Object Tracking", 
                       int(config.max_speed * 100), 100, nothing)
    
    # Enable tracking mode
    tracker.enable()
    
    paused = False
    last_send_time = time.time()
    
    # Target class shortcuts
    target_shortcuts = {
        ord('1'): 'person',
        ord('2'): 'car',
        ord('3'): 'bottle',
        ord('4'): 'cell phone',
        ord('5'): 'cup',
        ord('6'): 'chair',
        ord('7'): 'dog',
        ord('8'): 'cat',
        ord('9'): 'backpack',
    }
    
    try:
        while True:
            # Get frame
            color_frame, depth_frame = camera.get_frames()
            
            if color_frame is None:
                time.sleep(0.01)
                continue
            
            # Update config from trackbars
            tracker.config.steering_gain = cv2.getTrackbarPos("Steering Gain", "Object Tracking") / 10.0
            tracker.config.target_distance = cv2.getTrackbarPos("Target Dist (cm)", "Object Tracking") / 100.0
            tracker.config.max_speed = cv2.getTrackbarPos("Max Speed", "Object Tracking") / 100.0
            
            # Process
            if not paused:
                output = tracker.process(color_frame, depth_frame)
                
                # Send to robot (rate limited to 20Hz)
                current_time = time.time()
                if uart and (current_time - last_send_time >= 0.05):
                    uart.send_motion_command(output.velocity, output.yaw_rate)
                    last_send_time = current_time
                
                # Display frame with overlay
                if output.viz_frame is not None:
                    display_frame = output.viz_frame
                else:
                    display_frame = color_frame
                
                # Add extra status info
                h, w = display_frame.shape[:2]
                
                # Draw target distance indicator
                cv2.putText(display_frame, 
                           f"Target Dist: {tracker.config.target_distance:.1f}m | Gain: {tracker.config.steering_gain:.1f}",
                           (w - 350, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                # Draw velocity/yaw bar visualization
                bar_y = h - 70
                bar_center = w // 2
                
                # Velocity bar (vertical)
                vel_height = int(output.velocity * 100)
                vel_color = (0, 255, 0) if output.velocity >= 0 else (0, 0, 255)
                cv2.rectangle(display_frame, 
                             (bar_center - 20, bar_y - vel_height), 
                             (bar_center + 20, bar_y), vel_color, -1)
                cv2.putText(display_frame, f"V:{output.velocity:.2f}", 
                           (bar_center - 30, bar_y + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Yaw bar (horizontal)
                yaw_width = int(output.yaw_rate * 50)
                yaw_color = (255, 255, 0)
                if yaw_width > 0:
                    cv2.rectangle(display_frame, 
                                 (bar_center, bar_y - 40), 
                                 (bar_center + yaw_width, bar_y - 25), yaw_color, -1)
                else:
                    cv2.rectangle(display_frame, 
                                 (bar_center + yaw_width, bar_y - 40), 
                                 (bar_center, bar_y - 25), yaw_color, -1)
                cv2.putText(display_frame, f"Y:{output.yaw_rate:+.2f}", 
                           (bar_center + 50, bar_y - 28), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
            else:
                display_frame = color_frame.copy()
                cv2.putText(display_frame, "PAUSED", (50, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)
            
            cv2.imshow("Object Tracking", display_frame)
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                paused = not paused
                if paused and uart:
                    uart.send_motion_command(0, 0)
                print("⏸️ PAUSED" if paused else "▶️ RESUMED")
            elif key == ord('r'):
                tracker.reset()
                tracker.enable()
                print("🔄 Reset tracker")
            elif key == ord('+') or key == ord('='):
                tracker.config.target_distance = min(5.0, tracker.config.target_distance + 0.2)
                cv2.setTrackbarPos("Target Dist (cm)", "Object Tracking", 
                                  int(tracker.config.target_distance * 100))
                print(f"📏 Target distance: {tracker.config.target_distance:.1f}m")
            elif key == ord('-'):
                tracker.config.target_distance = max(0.5, tracker.config.target_distance - 0.2)
                cv2.setTrackbarPos("Target Dist (cm)", "Object Tracking", 
                                  int(tracker.config.target_distance * 100))
                print(f"📏 Target distance: {tracker.config.target_distance:.1f}m")
            elif key in target_shortcuts:
                new_target = target_shortcuts[key]
                tracker.set_target_class(new_target)
                print(f"🎯 Tracking: {new_target}")
    
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
        
        # Print statistics
        stats = tracker.get_statistics()
        print("\n" + "=" * 40)
        print("📊 TRACKING STATISTICS")
        print(f"   Total frames: {stats['total_frames']}")
        print(f"   Tracking frames: {stats['tracking_frames']}")
        print(f"   Tracking rate: {stats['tracking_rate']:.1%}")
        print(f"   Target class: {stats['target_class']}")
        print("=" * 40)


if __name__ == "__main__":
    main()
