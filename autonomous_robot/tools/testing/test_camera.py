#!/usr/bin/env python3
"""
Test Camera (RealSense D435i) với hiển thị trực quan.

Usage:
    python tools/testing/test_camera.py
"""

import cv2
import numpy as np
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.perception import RealSenseCamera


def main():
    print("=" * 60)
    print("  TEST: REALSENSE CAMERA (D435i)")
    print("=" * 60)
    print("Controls:")
    print("  Q - Quit")
    print("  S - Save frame")
    print("  D - Toggle depth view")
    print("  I - Show camera info")
    print("=" * 60)
    
    camera = RealSenseCamera()
    
    if not camera.start():
        print("ERROR: Cannot start camera!")
        return
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    show_depth = True
    
    try:
        while True:
            color_frame, depth_frame = camera.get_frames()
            
            if color_frame is None:
                continue
            
            frame_count += 1
            
            # Calculate FPS
            if frame_count % 30 == 0:
                now = time.time()
                fps = 30 / (now - fps_time)
                fps_time = now
            
            h, w = color_frame.shape[:2]
            
            if show_depth and depth_frame is not None:
                # Create side-by-side view
                depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
                
                vis = np.hstack([color_frame, depth_colored])
                
                # Labels
                cv2.putText(vis, "RGB", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(vis, "DEPTH", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            else:
                vis = color_frame.copy()
            
            # Info panel
            info = [
                f"Frame: {frame_count}",
                f"FPS: {fps:.1f}",
                f"Resolution: {w}x{h}",
                f"Depth: {'ON' if show_depth else 'OFF'} (press D)",
            ]
            
            vis_h = vis.shape[0]
            y = vis_h - 100
            for text in info:
                cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y += 25
            
            cv2.imshow("Camera Test", vis)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv2.imwrite(f"camera_rgb_{frame_count}.jpg", color_frame)
                if depth_frame is not None:
                    depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
                    depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
                    cv2.imwrite(f"camera_depth_{frame_count}.jpg", depth_colored)
                print(f"Saved frame {frame_count}")
            elif key == ord('d'):
                show_depth = not show_depth
                print(f"Depth view: {'ON' if show_depth else 'OFF'}")
            elif key == ord('i'):
                print(f"\nCamera Info:")
                print(f"  Resolution: {w}x{h}")
                print(f"  FPS: {fps:.1f}")
                print(f"  Frame count: {frame_count}")
    
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Test completed.")


if __name__ == "__main__":
    main()
