#!/usr/bin/env python3
"""
Line Following Robot - Main Control Script.

Detects a line using camera and sends control commands via UART to STM32.
Uses centerline detection method for simple and robust line following.

Usage:
    python run_line_follower.py [options]

Options:
    --mock-uart     Use mock UART for testing without hardware
    --no-viz        Disable visualization window
    --debug         Enable debug logging
    --config PATH   Path to custom YAML config file
"""

import cv2
import logging
import signal
import sys
import time
import argparse
from pathlib import Path

# Project root path
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.core import config
from src.perception import RealSenseCamera, SimpleLineDetector, LineDetectionResult
from src.perception import TerrainAnalyzer, TerrainConfig, ClearanceAction
from src.communication import UARTController, MockUARTController

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# =============================================================================
# CONTROL PARAMETERS
# =============================================================================
# These can be tuned for your specific robot

# Speed control (giá trị được nhân 1000 khi gửi qua UART, ví dụ: 0.1 m/s = V100)
BASE_SPEED = 0.10         # Base forward speed (m/s) - V100
MAX_SPEED = 0.15          # Maximum speed when line is straight - V150
MIN_SPEED = 0.05          # Minimum speed during sharp turns - V50

# Steering control (converts position error to yaw rate)
STEERING_GAIN = 1.2       # How aggressively to steer (rad/s per unit error)  
HEADING_GAIN = 0.3        # How much heading error affects steering
MAX_YAW_RATE = 0.7        # Maximum yaw rate (rad/s) - Y700

# Line search behavior (when line is lost)
SEARCH_YAW_RATE = 0.5     # Yaw rate for searching (rad/s) - Y500
MAX_FRAMES_LOST = 30      # Stop completely after this many frames without line

# Terrain analyzer (ground clearance adjustment)
ENABLE_TERRAIN_ANALYZER = True   # Bật/tắt phân tích địa hình
TERRAIN_ANALYZE_INTERVAL = 3     # Phân tích mỗi N frame (tiết kiệm CPU)


class LineFollower:
    """
    Main line following robot controller.
    Pipeline:
        Camera → Line Detection → Control Calculation → UART → STM32
    
    The robot follows a single line by:
    1. Detecting the line centerline in each frame
    2. Calculating position error (how far line is from center)
    3. Calculating heading error (angle of line vs robot heading)
    4. Converting errors to velocity and yaw rate commands
    5. Sending commands to STM32 via UART
    """

    def __init__(self, use_mock_uart: bool = False, enable_viz: bool = True):
        """
        Initialize line follower.
        
        Args:
            use_mock_uart: Use mock UART for testing without hardware
            enable_viz: Show visualization window
        """
        self.enable_viz = enable_viz
        self._running = False
        self._frame_count = 0
        self._start_time = 0.0
        
        logger.info("=" * 50)
        logger.info("INITIALIZING LINE FOLLOWER")
        logger.info("=" * 50)
        
        # Initialize camera
        logger.info("Initializing camera...")
        self.camera = RealSenseCamera()
        
        # Initialize line detector
        logger.info("Initializing line detector...")
        self.line_detector = SimpleLineDetector()
        
        # Initialize terrain analyzer
        if ENABLE_TERRAIN_ANALYZER:
            logger.info("Initializing terrain analyzer...")
            terrain_config = TerrainConfig(
                camera_height=0.20,
                camera_tilt_angle=15.0,
                min_ground_clearance=0.065,  # Hạ gầm: 6.5cm
                max_ground_clearance=0.18,   # Nâng gầm: 18cm
                normal_ground_clearance=0.08 # Bình thường: 8cm
            )
            self.terrain_analyzer = TerrainAnalyzer(terrain_config)
            self._current_clearance = terrain_config.normal_ground_clearance
            logger.info(f"→ Terrain analyzer enabled, default clearance: {self._current_clearance*100:.0f}cm")
        else:
            self.terrain_analyzer = None
            self._current_clearance = 0.08  # Bình thường: 8cm
        
        # Initialize UART
        logger.info("Initializing UART...")
        if use_mock_uart:
            self.uart = MockUARTController()
            logger.info("→ Using MOCK UART (no hardware)")
        else:
            self.uart = UARTController()
            logger.info(f"→ Using real UART on {config.UART_PORT}")
        
        # Control loop timing
        self._loop_period = 1.0 / config.MAIN_LOOP_RATE_HZ
        
        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._on_shutdown_signal)
        signal.signal(signal.SIGTERM, self._on_shutdown_signal)
        
        logger.info("Initialization complete!")

    def start(self) -> bool:
        """
        Start the robot.
        
        Returns:
            True if started successfully, False otherwise
        """
        logger.info("-" * 50)
        logger.info("STARTING ROBOT")
        logger.info("-" * 50)
        
        # Start camera
        if not self.camera.start():
            logger.error("✗ Failed to start camera!")
            return False
        logger.info("✓ Camera started")
        
        # Connect UART
        if not self.uart.connect():
            if isinstance(self.uart, MockUARTController):
                logger.warning("⚠ Mock UART - continuing without hardware")
            else:
                logger.error("✗ Failed to connect UART!")
                self.camera.stop()
                return False
        logger.info("✓ UART connected")
        
        # Enable motor control (quan trọng - xe chỉ phản ứng khi enabled!)
        if self.uart.is_connected:
            success = self.uart.enable_control()
            if success:
                logger.info("✓ Motor control ENABLED - xe sẽ phản ứng")
            else:
                logger.error("✗ FAILED to enable motor control!")
                logger.error("  → Xe sẽ KHÔNG phản ứng cho đến khi enable thành công")
        else:
            logger.warning("⚠ UART not connected - motor control NOT enabled!")
        
        self._running = True
        self._start_time = time.time()
        
        logger.info("-" * 50)
        logger.info("ROBOT READY - Press 'q' to quit")
        logger.info("-" * 50)
        
        return True

    def stop(self):
        """Stop the robot safely."""
        logger.info("-" * 50)
        logger.info("STOPPING ROBOT")
        logger.info("-" * 50)
        
        self._running = False
        
        # Stop motors
        if self.uart.is_connected:
            logger.info("Stopping motors...")
            self.uart.send_emergency_stop()
            self.uart.disconnect()
        
        # Stop camera
        self.camera.stop()
        
        # Close windows
        if self.enable_viz:
            cv2.destroyAllWindows()
        
        # Print statistics
        elapsed = time.time() - self._start_time
        if elapsed > 0 and self._frame_count > 0:
            fps = self._frame_count / elapsed
            logger.info(f"Statistics: {self._frame_count} frames, {fps:.1f} FPS")
        
        logger.info("Robot stopped safely.")

    def run(self):
        """
        Main control loop.
        
        Runs until 'q' is pressed or signal received.
        """
        logger.info("Entering main control loop...")
        
        while self._running:
            loop_start = time.time()
            
            try:
                # Execute one control cycle
                self._control_step()
                
            except Exception as e:
                logger.error(f"Control error: {e}")
                self._send_stop_command()
            
            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = self._loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            self._frame_count += 1
            
            # Handle keyboard input
            if self.enable_viz:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    logger.info("Quit requested by user")
                    self._running = False
                elif key == ord('r'):
                    logger.info("Reset line detector")
                    self.line_detector.reset()
        
        # Clean shutdown
        self.stop()

    def _control_step(self):
        """
        Execute one control step.
        
        1. Get camera frame
        2. Detect line
        3. Calculate control commands
        4. Send commands to robot
        5. Update visualization
        """
        # Get frame from camera
        color_frame, depth_frame = self.camera.get_frames()
        
        if color_frame is None:
            logger.warning("No camera frame!")
            self._send_stop_command()
            return
        
        # Detect line
        result = self.line_detector.detect(color_frame)
        
        # Analyze terrain for ground clearance (every N frames)
        terrain_result = None
        if self.terrain_analyzer and depth_frame is not None:
            if self._frame_count % TERRAIN_ANALYZE_INTERVAL == 0:
                terrain_result = self.terrain_analyzer.analyze(depth_frame)
                self._process_terrain(terrain_result)
        
        # Log detection status periodically
        if self._frame_count % 30 == 0:  # Every 30 frames (~1 second)
            logger.info(f"Line detected: {result.line_detected}, UART connected: {self.uart.is_connected}, Enabled: {self.uart._is_enabled if hasattr(self.uart, '_is_enabled') else 'N/A'}")
        
        # Calculate and send control commands
        self._process_detection(result)
        
        # Update visualization
        if self.enable_viz:
            self._update_display(color_frame, result, depth_frame, terrain_result)

    def _process_detection(self, result: LineDetectionResult):
        """
        Process line detection result and send control commands.
        
        Args:
            result: Line detection result from SimpleLineDetector
        """
        if not self.uart.is_connected:
            logger.warning("UART not connected - cannot send commands!")
            return
        
        if not self.uart.is_enabled:
            logger.warning("Motor control NOT ENABLED - call enable_control() first!")
            return
        
        if result.line_detected:
            # Line found - calculate control
            velocity, yaw_rate = self._calculate_control(result)
            # Log với format giống UART command (V và Y nhân 1000)
            logger.info(f">>> SENDING: V{int(velocity*1000)} Y{int(yaw_rate*1000)} (v={velocity:.3f} m/s, y={yaw_rate:.3f} rad/s)")
            self.uart.send_motion_command(velocity, yaw_rate)
            
        elif result.frames_lost < MAX_FRAMES_LOST:
            # Line recently lost - search mode
            self._search_for_line(result)
            
        else:
            # Line lost too long - stop
            logger.warning(f"Line lost for {result.frames_lost} frames - stopping")
            self._send_stop_command()

    def _calculate_control(self, result: LineDetectionResult) -> tuple:
        """
        Calculate velocity and yaw rate from detection result.
        
        Args:
            result: Line detection result
            
        Returns:
            (velocity, yaw_rate) tuple
        """
        # Position error: negative = line is left, positive = line is right
        pos_error = result.position_error  # Range: -1 to 1
        
        # Heading error: angle of line relative to vertical
        heading_error = result.heading_error  # Radians
        
        # Calculate yaw rate (steering)
        # Combine position error and heading error
        yaw_rate = STEERING_GAIN * pos_error + HEADING_GAIN * heading_error
        
        # Clamp yaw rate to MAX_YAW_RATE (default ±0.7 rad/s = Y700)
        yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, yaw_rate))
        
        # Calculate velocity - fixed speed for testing
        velocity = BASE_SPEED  # Use constant speed (V20 = 0.02 m/s)
        
        logger.info(f"Control calc: pos_err={pos_error:+.3f}, heading_err={heading_error:+.3f} → V={velocity:.3f}, Y={yaw_rate:+.3f}")
        
        return velocity, yaw_rate

    def _search_for_line(self, result: LineDetectionResult):
        """
        Search for lost line by rotating in place (V=0, Y≠0).
        
        Chiến lược: Khi mất line, xe dừng lại và chỉ quay tại chỗ 
        để tìm line, thay vì di chuyển tiếp.
        
        Args:
            result: Line detection result (contains search direction)
        """
        # Dừng hẳn velocity, chỉ quay tại chỗ
        velocity = 0.0  # DỪNG LẠI - không di chuyển
        
        # Quay theo hướng đã thấy line lần cuối
        search_yaw = result.search_direction * SEARCH_YAW_RATE
        
        # Gửi lệnh: V=0 (đứng yên), Y≠0 (quay tìm)
        self.uart.send_motion_command(velocity, search_yaw)
        
        if result.frames_lost % 10 == 0:  # Log every 10 frames
            direction = "trái" if result.search_direction < 0 else "phải"
            logger.info(f"Tìm line... quay tại chỗ về {direction} (V=0, Y={search_yaw:+.2f})")

    def _send_stop_command(self):
        """Send stop command to robot."""
        if self.uart.is_connected:
            self.uart.send_motion_command(0.0, 0.0)

    def _process_terrain(self, terrain_result):
        """
        Process terrain analysis and adjust ground clearance.
        
        Args:
            terrain_result: TerrainAnalysisResult from terrain analyzer
        """
        if terrain_result is None:
            return
        
        action = terrain_result.action
        new_clearance = terrain_result.recommended_height
        
        # Only act if clearance needs to change significantly
        clearance_diff = abs(new_clearance - self._current_clearance)
        if clearance_diff < 0.01:  # Less than 1cm change - ignore
            return
        
        if action == ClearanceAction.RAISE:
            # Nâng gầm để vượt chướng ngại vật
            logger.info(f"🔼 TERRAIN: Nâng gầm {self._current_clearance*100:.0f}cm → {new_clearance*100:.0f}cm")
            logger.info(f"   Lý do: {terrain_result.message}")
            self._current_clearance = new_clearance
            self._send_clearance_command(new_clearance)
            
        elif action == ClearanceAction.LOWER:
            # Hạ gầm để qua trần thấp
            logger.info(f"🔽 TERRAIN: Hạ gầm {self._current_clearance*100:.0f}cm → {new_clearance*100:.0f}cm")
            logger.info(f"   Lý do: {terrain_result.message}")
            self._current_clearance = new_clearance
            self._send_clearance_command(new_clearance)
        elif action == ClearanceAction.STOP:
            # Không thể qua - dừng lại
            logger.warning(f"⛔ TERRAIN: DỪNG LẠI! {terrain_result.message}")
            self._send_stop_command()
            
        elif action == ClearanceAction.NORMAL:
            # Trở về bình thường nếu đang ở trạng thái khác
            if self._current_clearance != 0.08:  # Default clearance = 8cm
                logger.info(f"↩️ TERRAIN: Trở về bình thường {new_clearance*100:.0f}cm")
                self._current_clearance = new_clearance
                self._send_clearance_command(new_clearance)

    def _send_clearance_command(self, clearance: float):
        """
        Send ground clearance command to robot.
        
        Args:
            clearance: Target ground clearance in meters
        """
        if self.uart.is_connected:
            # Gửi lệnh điều chỉnh độ cao gầm
            # Format: HEIGHT <clearance_cm>
            clearance_cm = clearance * 100
            command = f"HEIGHT {clearance_cm:.1f}"
            logger.debug(f"Sending clearance command: {command}")
            # self.uart.send_raw_command(command)  # Uncomment when UART supports this
            pass  # TODO: Implement in UART controller

    def _update_display(self, frame, result: LineDetectionResult, 
                         depth_frame=None, terrain_result=None):
        """
        Update visualization window.
        
        Args:
            frame: Camera frame
            result: Line detection result
            depth_frame: Depth frame (optional)
            terrain_result: Terrain analysis result (optional)
        """
        # Get visualization from line detector
        vis = self.line_detector.visualize(frame, result)
        
        h, w = vis.shape[:2]
        
        # Add terrain overlay if available
        if self.terrain_analyzer and terrain_result:
            vis = self._draw_terrain_overlay(vis, terrain_result)
        
        # Add status bar at bottom
        status_bar = 30
        vis_with_bar = cv2.copyMakeBorder(
            vis, 0, status_bar, 0, 0, 
            cv2.BORDER_CONSTANT, value=(40, 40, 40)
        )
        
        # UART status
        if self.uart.is_connected:
            status = "UART: Connected"
            color = (0, 255, 0)
        else:
            status = "UART: Disconnected"
            color = (0, 0, 255)
        
        cv2.putText(vis_with_bar, status, (10, h + 22),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # FPS
        if self._frame_count > 0:
            elapsed = time.time() - self._start_time
            fps = self._frame_count / elapsed if elapsed > 0 else 0
            cv2.putText(vis_with_bar, f"FPS: {fps:.1f}", (w - 80, h + 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Line status
        if result.line_detected:
            line_status = f"Line OK | Err: {result.position_error:+.2f}"
            line_color = (0, 255, 0)
        else:
            line_status = f"Line LOST ({result.frames_lost})"
            line_color = (0, 0, 255)
        
        cv2.putText(vis_with_bar, line_status, (150, h + 22),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, line_color, 1)
        
        cv2.imshow("Line Follower", vis_with_bar)

    def _draw_terrain_overlay(self, vis, terrain_result):
        """
        Draw terrain analysis overlay on visualization.
        
        Args:
            vis: Visualization frame
            terrain_result: Terrain analysis result
            
        Returns:
            Frame with terrain overlay
        """
        h, w = vis.shape[:2]
        
        # Color based on action
        action_colors = {
            ClearanceAction.NORMAL: (0, 255, 0),    # Green
            ClearanceAction.RAISE: (0, 165, 255),   # Orange
            ClearanceAction.LOWER: (255, 165, 0),   # Light blue
            ClearanceAction.STOP: (0, 0, 255),      # Red
        }
        color = action_colors.get(terrain_result.action, (255, 255, 255))
        
        # Draw terrain info panel (top-right)
        panel_x = w - 200
        panel_y = 10
        # Background
        cv2.rectangle(vis, (panel_x - 5, panel_y - 5), 
                     (w - 5, panel_y + 75), (0, 0, 0), -1)
        cv2.rectangle(vis, (panel_x - 5, panel_y - 5), 
                     (w - 5, panel_y + 75), color, 2)
        # Title
        cv2.putText(vis, "TERRAIN", (panel_x, panel_y + 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        # Clearance
        cv2.putText(vis, f"Gam: {self._current_clearance*100:.0f}cm", 
                   (panel_x, panel_y + 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        # Action
        action_names = {
            ClearanceAction.NORMAL: "OK",
            ClearanceAction.RAISE: "NANG GAM",
            ClearanceAction.LOWER: "HA GAM",
            ClearanceAction.STOP: "DUNG!",
        }
        action_text = action_names.get(terrain_result.action, "?")
        cv2.putText(vis, action_text, (panel_x, panel_y + 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        # Confidence
        cv2.putText(vis, f"Conf: {terrain_result.confidence:.0%}", 
                   (panel_x, panel_y + 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        return vis

    def _on_shutdown_signal(self, sig, frame):
        """Handle shutdown signals (Ctrl+C, etc.)."""
        logger.info(f"Shutdown signal received ({sig})")
        self._running = False
# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Line Following Robot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python run_line_follower.py                    # Normal operation
    python run_line_follower.py --mock-uart        # Test without hardware
    python run_line_follower.py --debug            # Enable debug logging
    python run_line_follower.py --no-viz           # Run headless
        """
    )
    parser.add_argument('--mock-uart', action='store_true', 
                       help='Use mock UART for testing')
    parser.add_argument('--no-viz', action='store_true', 
                       help='Disable visualization')
    parser.add_argument('--debug', action='store_true', 
                       help='Enable debug logging')
    parser.add_argument('--config', type=str, default=None, 
                       help='Path to YAML config file')
    
    args = parser.parse_args()
    
    # Set debug logging if requested
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Load configuration BEFORE creating robot
    config_file = args.config
    if config_file is None:
        config_file = PROJECT_ROOT / "configs" / "default_config.yaml"
    
    logger.info(f"Loading config: {config_file}")
    try:
        config.load_config(config_file)
    except Exception as e:
        logger.error(f"Failed to load config: {e}")
        sys.exit(1)
    # Create and run robot
    robot = LineFollower(
        use_mock_uart=args.mock_uart,
        enable_viz=not args.no_viz
    )
    if robot.start():
        robot.run()
    else:
        logger.error("Failed to start robot!")
        sys.exit(1)
if __name__ == "__main__":
    main()
