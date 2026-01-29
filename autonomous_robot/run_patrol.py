#!/usr/bin/env python3
"""
Patrol Surveillance Robot - Main Control Script.

Robot tuần tra tự động với khả năng:
- Phát hiện người xâm nhập bằng YOLOv8
- Đo khoảng cách với Intel RealSense
- Theo dõi (tracking) người lạ
- Cảnh báo qua buzzer và UART

Usage:
    python run_patrol.py [options]

Options:
    --mock-uart     Use mock UART for testing without hardware
    --no-uart       Disable UART completely
    --no-viz        Disable visualization window
    --debug         Enable debug logging
    --port PORT     UART port (default: /dev/ttyACM0)

Controls:
    q       - Quit
    r       - Reset patrol
    p       - Pause/Resume
    b       - Test buzzer
    Space   - Stop robot immediately
"""

import cv2
import logging
import signal
import sys
import time
import argparse
from pathlib import Path
from datetime import datetime

# Project root path
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.core import config
from src.perception import RealSenseCamera
from src.modes import PatrolMode, PatrolConfig
from src.communication import UARTController, MockUARTController

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# =============================================================================
# PATROL PARAMETERS
# =============================================================================

# Patrol movement
PATROL_VELOCITY = 0.25        # Forward speed while patrolling (m/s)
ROTATE_YAW_RATE = 0.4         # Rotation speed (rad/s)
PATROL_FORWARD_TIME = 8.0     # Seconds to move forward before rotating
PATROL_ROTATE_TIME = 2.5      # Seconds to rotate

# Detection settings
DETECT_CLASS = "person"       # Object class to detect as intruder
MIN_CONFIDENCE = 0.40         # Minimum detection confidence (lower = more sensitive)
MIN_BOX_AREA = 2000           # Minimum bounding box area in pixels²

# Alert settings
ALERT_DISTANCE = 5.0          # Maximum distance to trigger alert (meters)
ALERT_DURATION = 3.0          # How long to stay in alert state (seconds)
CLOSE_DISTANCE = 2.5          # Distance considered "close" - strong alarm (meters)
DANGER_DISTANCE = 1.5         # Distance considered "danger" - emergency (meters)

# Tracking settings
TRACK_INTRUDER = True         # Enable following intruder
TRACKING_DISTANCE = 2.0       # Distance to maintain from intruder (meters)
TRACKING_GAIN = 1.5           # Steering gain when tracking
MAX_TRACK_TIME = 15.0         # Maximum tracking time (seconds)

# Buzzer settings
ENABLE_BUZZER = True          # Enable buzzer alerts


class BuzzerController:
    """Điều khiển còi qua UART."""
    
    def __init__(self, uart):
        self.uart = uart
        self._last_beep_time = 0
        self._beep_cooldown = 2.0  # Không beep lại trong 2s
    
    def beep(self):
        """Single beep."""
        if self.uart and time.time() - self._last_beep_time > self._beep_cooldown:
            self.uart._send_command_direct("B1")
            self._last_beep_time = time.time()
            logger.info("🔔 BEEP!")
    
    def alarm(self):
        """Continuous alarm."""
        if self.uart:
            self.uart._send_command_direct("B2")
            logger.warning("🚨 ALARM!")
    
    def stop(self):
        """Stop buzzer."""
        if self.uart:
            self.uart._send_command_direct("B0")


class PatrolRobot:
    """
    Main patrol surveillance robot controller.
    
    Pipeline:
        Camera → YOLO Detection → Depth Measurement → Patrol Logic → UART → STM32
    
    States:
        PATROLLING → ROTATING → ALERT → TRACKING → RETURNING
    """

    def __init__(self, use_mock_uart: bool = False, no_uart: bool = False, 
                 enable_viz: bool = True, uart_port: str = '/dev/ttyACM0'):
        """
        Initialize patrol robot.
        
        Args:
            use_mock_uart: Use mock UART for testing
            no_uart: Disable UART completely
            enable_viz: Show visualization window
            uart_port: UART port path
        """
        self.enable_viz = enable_viz
        self.no_uart = no_uart
        self._running = False
        self._paused = False
        self._frame_count = 0
        self._start_time = 0.0
        self._intruder_log = []
        
        logger.info("=" * 60)
        logger.info("🛡️  PATROL SURVEILLANCE ROBOT - INITIALIZING")
        logger.info("=" * 60)
        
        # Initialize camera
        logger.info("Initializing camera...")
        self.camera = RealSenseCamera()
        
        # Initialize patrol mode
        logger.info("Initializing patrol mode...")
        patrol_config = PatrolConfig(
            patrol_velocity=PATROL_VELOCITY,
            rotate_yaw_rate=ROTATE_YAW_RATE,
            patrol_forward_time=PATROL_FORWARD_TIME,
            patrol_rotate_time=PATROL_ROTATE_TIME,
            detect_class=DETECT_CLASS,
            min_confidence=MIN_CONFIDENCE,
            min_box_area=MIN_BOX_AREA,
            alert_distance=ALERT_DISTANCE,
            alert_duration=ALERT_DURATION,
            track_intruder=TRACK_INTRUDER,
            tracking_distance=TRACKING_DISTANCE,
            tracking_gain=TRACKING_GAIN,
            max_track_time=MAX_TRACK_TIME,
            enable_sound_alert=ENABLE_BUZZER
        )
        self.patrol = PatrolMode(patrol_config)
        self.patrol.set_alert_callback(self._on_intruder_detected)
        
        # Initialize UART
        self.uart = None
        self.buzzer = None
        
        if no_uart:
            logger.info("→ UART disabled")
        elif use_mock_uart:
            self.uart = MockUARTController()
            logger.info("→ Using MOCK UART (no hardware)")
        else:
            logger.info(f"Initializing UART on {uart_port}...")
            self.uart = UARTController(port=uart_port)
        
        if self.uart:
            self.buzzer = BuzzerController(self.uart)
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._on_shutdown_signal)
        signal.signal(signal.SIGTERM, self._on_shutdown_signal)
        
        logger.info("Initialization complete!")
        logger.info("-" * 60)

    def _on_intruder_detected(self, intruder):
        """Callback when intruder is detected."""
        timestamp = datetime.now().strftime('%H:%M:%S')
        distance = intruder.distance
        
        # Determine alert level based on distance
        if distance <= DANGER_DISTANCE:
            alert_level = "🔴 NGUY HIỂM"
            alert_type = "danger"
        elif distance <= CLOSE_DISTANCE:
            alert_level = "🟠 GẦN"
            alert_type = "close"
        else:
            alert_level = "🟡 PHÁT HIỆN"
            alert_type = "far"
        
        logger.warning("=" * 50)
        logger.warning(f"🚨 {alert_level}: PHÁT HIỆN NGƯỜI XÂM NHẬP!")
        logger.warning(f"   Khoảng cách: {distance:.1f}m")
        logger.warning(f"   Độ tin cậy: {intruder.confidence:.1%}")
        logger.warning(f"   Thời gian: {timestamp}")
        logger.warning("=" * 50)
        
        # Log intruder
        self._intruder_log.append({
            'timestamp': timestamp,
            'distance': distance,
            'confidence': intruder.confidence,
            'position': (intruder.center_x, intruder.center_y),
            'alert_type': alert_type
        })
        
        # Sound alarm based on distance
        if self.buzzer and ENABLE_BUZZER:
            if distance <= DANGER_DISTANCE:
                # Very close - continuous alarm + emergency stop
                logger.warning("⚠️ DỪNG KHẨN CẤP - NGƯỜI Ở QUÁ GẦN!")
                self.buzzer.alarm()  # B2 - continuous alarm
                if self.uart:
                    self.uart._send_command_direct("V0")
                    self.uart._send_command_direct("Y0")
            elif distance <= CLOSE_DISTANCE:
                # Close - continuous alarm
                self.buzzer.alarm()  # B2 - continuous alarm
            else:
                # Far - single beep
                self.buzzer.beep()   # B1 - single beep

    def _on_shutdown_signal(self, signum, frame):
        """Handle shutdown signal."""
        logger.info("\n\n⚠ Received shutdown signal")
        self._running = False

    def start(self) -> bool:
        """Start the robot."""
        logger.info("-" * 60)
        logger.info("🚀 STARTING PATROL ROBOT")
        logger.info("-" * 60)
        
        # Start camera
        if not self.camera.start():
            logger.error("✗ Failed to start camera!")
            return False
        logger.info("✓ Camera started")
        
        # Connect UART
        if self.uart and not self.no_uart:
            if not self.uart.connect():
                logger.error("✗ Failed to connect UART!")
                # Continue without UART
            else:
                logger.info("✓ UART connected")
                if hasattr(self.uart, 'enable_control'):
                    self.uart.enable_control()
                    logger.info("✓ Robot control enabled")
                # Send E1 to enable
                self.uart._send_command_direct("E1")
        
        # Enable patrol
        self.patrol.enable()
        logger.info("✓ Patrol mode enabled")
        
        self._running = True
        self._start_time = time.time()
        
        return True

    def stop(self):
        """Stop the robot."""
        logger.info("-" * 60)
        logger.info("🛑 STOPPING PATROL ROBOT")
        logger.info("-" * 60)
        
        self._running = False
        
        # Stop buzzer
        if self.buzzer:
            self.buzzer.stop()
        
        # Stop robot motion
        if self.uart:
            try:
                self.uart._send_command_direct("V0")
                self.uart._send_command_direct("Y0")
                self.uart._send_command_direct("E0")
                self.uart.disconnect()
                logger.info("✓ UART disconnected")
            except:
                pass
        
        # Stop camera
        self.camera.stop()
        logger.info("✓ Camera stopped")
        
        # Destroy windows
        cv2.destroyAllWindows()
        
        # Print summary
        self._print_summary()

    def _print_summary(self):
        """Print patrol session summary."""
        elapsed = time.time() - self._start_time
        
        print("\n" + "=" * 60)
        print("📊 TÓM TẮT PHIÊN TUẦN TRA")
        print("=" * 60)
        print(f"   Thời gian chạy: {elapsed/60:.1f} phút")
        print(f"   Số frame xử lý: {self._frame_count}")
        print(f"   FPS trung bình: {self._frame_count/elapsed:.1f}")
        print(f"   Số người phát hiện: {len(self._intruder_log)}")
        
        if self._intruder_log:
            print("\n   📋 Chi tiết phát hiện:")
            for i, entry in enumerate(self._intruder_log, 1):
                print(f"      {i}. {entry['timestamp']} - "
                      f"{entry['distance']:.1f}m ({entry['confidence']:.0%})")
        
        print("=" * 60)

    def run(self):
        """Main control loop."""
        if not self.start():
            return
        
        # Create window
        if self.enable_viz:
            cv2.namedWindow("Patrol Surveillance", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Patrol Surveillance", 1280, 720)
        
        logger.info("\n" + "-" * 40)
        logger.info("🎮 CONTROLS:")
        logger.info("   q     - Quit")
        logger.info("   r     - Reset patrol")
        logger.info("   p     - Pause/Resume")
        logger.info("   b     - Test buzzer")
        logger.info("   Space - Emergency stop")
        logger.info("-" * 40 + "\n")
        
        try:
            while self._running:
                loop_start = time.time()
                
                # Get frames
                color_frame, depth_frame = self.camera.get_frames()
                
                if color_frame is None:
                    time.sleep(0.01)
                    continue
                
                self._frame_count += 1
                
                # Process frame (if not paused)
                if not self._paused:
                    output = self.patrol.process(color_frame, depth_frame)
                    
                    # Send commands to robot
                    if self.uart and not self.no_uart:
                        v_cmd = int(output.velocity * 1000)
                        y_cmd = int(output.yaw_rate * 1000)
                        self.uart._send_command_direct(f"V{v_cmd}")
                        self.uart._send_command_direct(f"Y{y_cmd}")
                    
                    # Get visualization
                    display_frame = output.viz_frame if output.viz_frame is not None else color_frame
                else:
                    display_frame = color_frame.copy()
                    cv2.putText(display_frame, "⏸ PAUSED", (50, 100),
                               cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)
                
                # Add status bar
                self._draw_status_bar(display_frame)
                
                # Show
                if self.enable_viz:
                    cv2.imshow("Patrol Surveillance", display_frame)
                
                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF
                if not self._handle_key(key):
                    break
                
                # Maintain loop rate
                elapsed = time.time() - loop_start
                if elapsed < 0.033:  # 30fps
                    time.sleep(0.033 - elapsed)
        
        except Exception as e:
            logger.error(f"Error in main loop: {e}")
        
        finally:
            self.stop()

    def _draw_status_bar(self, frame):
        """Draw status information on frame."""
        h, w = frame.shape[:2]
        
        # Background
        cv2.rectangle(frame, (0, h-60), (w, h), (40, 40, 40), -1)
        
        # Status text
        elapsed = time.time() - self._start_time
        fps = self._frame_count / elapsed if elapsed > 0 else 0
        
        status = f"Patrol | Time: {elapsed/60:.1f}m | FPS: {fps:.1f} | Detections: {len(self._intruder_log)}"
        if self._paused:
            status = "⏸ PAUSED | " + status
        
        cv2.putText(frame, status, (10, h-25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Mode indicator
        mode_text = f"[Mode: {'MOCK UART' if isinstance(self.uart, MockUARTController) else 'UART' if self.uart else 'NO UART'}]"
        cv2.putText(frame, mode_text, (w-250, h-25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    def _handle_key(self, key) -> bool:
        """Handle keyboard input. Returns False to quit."""
        if key == ord('q'):
            logger.info("Quit requested")
            return False
        
        elif key == ord('r'):
            logger.info("Resetting patrol...")
            self.patrol.reset()
            self.patrol.enable()
            if self.buzzer:
                self.buzzer.stop()
        
        elif key == ord('p'):
            self._paused = not self._paused
            logger.info("Paused" if self._paused else "Resumed")
            if self._paused and self.uart:
                self.uart._send_command_direct("V0")
                self.uart._send_command_direct("Y0")
        
        elif key == ord('b'):
            logger.info("Testing buzzer...")
            if self.buzzer:
                self.buzzer.beep()
        
        elif key == ord(' '):
            logger.warning("EMERGENCY STOP!")
            if self.uart:
                self.uart._send_command_direct("V0")
                self.uart._send_command_direct("Y0")
            self._paused = True
        
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Patrol Surveillance Robot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python run_patrol.py                    # Run with real UART
    python run_patrol.py --mock-uart        # Test without hardware
    python run_patrol.py --no-uart          # Camera only, no robot
    python run_patrol.py --debug            # Enable debug logging
        """
    )
    parser.add_argument('--mock-uart', action='store_true',
                       help='Use mock UART for testing without hardware')
    parser.add_argument('--no-uart', action='store_true',
                       help='Disable UART completely')
    parser.add_argument('--no-viz', action='store_true',
                       help='Disable visualization window')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug logging')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                       help='UART port (default: /dev/ttyACM0)')
    
    args = parser.parse_args()
    
    # Set logging level
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Print banner
    print("=" * 60)
    print("🛡️  PATROL SURVEILLANCE ROBOT")
    print("    AI-Powered Intruder Detection System")
    print("=" * 60)
    print()
    
    # Create and run robot
    robot = PatrolRobot(
        use_mock_uart=args.mock_uart,
        no_uart=args.no_uart,
        enable_viz=not args.no_viz,
        uart_port=args.port
    )
    
    robot.run()


if __name__ == "__main__":
    main()
