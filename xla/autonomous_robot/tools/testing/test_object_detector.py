#!/usr/bin/env python3
"""
Test Object Detector với hiển thị trực quan.

Usage:
    python tools/testing/test_object_detector.py
"""

import cv2
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception import RealSenseCamera, ObjectDetector
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
    print("  TEST: OBJECT DETECTOR (YOLOv8)")
    print("=" * 60)
    print("Controls:")
    print("  Q - Quit")
    print("  S - Save frame") 
    print("  D - Toggle depth view")
    print("  P - Pause/Resume")
    print("  R - Reset detector stats")
    print("  I - Show detection info")
    print("  Trackbars - Adjust thresholds")
    print("=" * 60)
    
    camera = RealSenseCamera()
    detector = ObjectDetector()
    
    if not camera.start():
        print("ERROR: Cannot start camera!")
        return
    
    # Create window and trackbars for tuning
    cv2.namedWindow("Object Detector Test")
    
    # Trackbars for real-time parameter adjustment
    # Confidence: 30% mặc định để detect nhiều hơn (range 0-100%)
    cv2.createTrackbar("Confidence", "Object Detector Test", 
                       30, 100, nothing)
    # NMS: 45% để cân bằng giữa loại trùng và giữ detection
    cv2.createTrackbar("NMS Threshold", "Object Detector Test", 
                       45, 100, nothing)
    # Safe Distance: 3m để phát hiện obstacle từ xa
    cv2.createTrackbar("Safe Distance", "Object Detector Test", 
                       300, 500, nothing)  # 0-5m range
    
    frame_count = 0
    fps_time = time.time() 
    fps = 0
    show_depth = True
    paused = False
    detection_times = []
    
    try:
        while True:
            color_frame, depth_frame = camera.get_frames()
            
            if color_frame is None:
                continue
                
            # Handle pause
            if paused:
                cv2.waitKey(30)
                continue
            
            frame_count += 1
            
            # Update config from trackbars
            confidence = cv2.getTrackbarPos("Confidence", "Object Detector Test") / 100.0
            nms_threshold = cv2.getTrackbarPos("NMS Threshold", "Object Detector Test") / 100.0  
            safe_distance = cv2.getTrackbarPos("Safe Distance", "Object Detector Test") / 100.0
            
            config.YOLO_CONFIDENCE_THRESHOLD = confidence
            config.YOLO_NMS_THRESHOLD = nms_threshold
            config.D_SAFE = safe_distance
            
            # Measure detection time
            start_time = time.time()
            result = detector.detect(color_frame, depth_frame)
            detection_time = time.time() - start_time
            detection_times.append(detection_time)
            
            # Keep only last 30 measurements for average
            if len(detection_times) > 30:
                detection_times = detection_times[-30:]
            
            # Calculate FPS
            if time.time() - fps_time > 1.0:
                fps = frame_count / (time.time() - fps_time)
                fps_time = time.time()
                frame_count = 0
        
            # Choose visualization
            if show_depth and depth_frame is not None:
                # Show depth visualization
                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_frame, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                vis = cv2.hconcat([detector.visualize(color_frame, result), depth_vis])
            else:
                vis = detector.visualize(color_frame, result)
            
            # Add comprehensive info panel
            h, w = vis.shape[:2]
            
            # Status color
            emergency = result.emergency_stop
            status_color = (0, 0, 255) if emergency else (0, 255, 0)
            status_text = "EMERGENCY STOP!" if emergency else "OK"
            
            # Performance metrics
            avg_detection_time = sum(detection_times) / len(detection_times) if detection_times else 0
            
            # Draw comprehensive info
            info = [
                f"Frame: {frame_count}  FPS: {fps:.1f}",
                f"Status: {status_text}",
                f"Objects: {len(result.objects)} | Obstacles: {len(result.obstacles)}",
                f"Detection: {avg_detection_time*1000:.1f}ms",
                f"Config: C:{confidence:.2f} N:{nms_threshold:.2f} D:{safe_distance:.2f}m",
            ]
            
            if result.closest_obstacle:
                obj = result.closest_obstacle
                info.append(f"Closest: {obj.class_name} @ {obj.depth:.2f}m")

            y = 30
            for i, text in enumerate(info):
                color = status_color if i == 1 else (255, 255, 255)
                cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                y += 25

            # Object list
            y = h - 20 - len(result.objects) * 20
            for obj in result.objects:
                if obj.depth > 0:
                    text = f"{obj.class_name}: {obj.depth:.2f}m"
                else:
                    text = f"{obj.class_name}: N/A"
                cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                y += 20

            cv2.imshow("Object Detector Test", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"object_test_{frame_count}.jpg"
                cv2.imwrite(filename, vis)
                print(f"Saved: {filename}")
            elif key == ord('d'):
                show_depth = not show_depth
                print(f"Depth view: {'ON' if show_depth else 'OFF'}")
            elif key == ord('p'):
                paused = not paused
                print(f"{'PAUSED' if paused else 'RESUMED'}")
            elif key == ord('r'):
                detection_times = []
                frame_count = 0
                fps_time = time.time()
                print("Stats reset!")
            elif key == ord('i'):
                print("\n" + "="*50)
                print("DETECTION INFO:")
                print(f"Total objects: {len(result.objects)}")
                for obj in result.objects:
                    print(f"  {obj.class_name}: conf={obj.confidence:.3f}, depth={obj.depth:.2f}m")
                print("="*50 + "\n")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Test completed.")


if __name__ == "__main__":
    main()
