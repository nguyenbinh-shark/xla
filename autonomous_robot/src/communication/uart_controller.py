"""
UART Controller Module for STM32 Communication.
Implements the ASCII command protocol for robot control.
Includes watchdog/heartbeat mechanism for safety.
"""

import serial
import logging
import time
import threading
from typing import Optional, Callable
from queue import Queue, Empty
from dataclasses import dataclass

from src.core import config

logger = logging.getLogger(__name__)


@dataclass
class RobotFeedback:
    """Feedback data from STM32."""
    velocity: float = 0.0      # Filtered velocity (m/s)
    position: float = 0.0      # Estimated position (m)
    yaw: float = 0.0           # Total yaw angle (rad)
    yaw_rate: float = 0.0      # Yaw rate (rad/s)
    timestamp: float = 0.0     # Reception timestamp
    valid: bool = False        # Whether data is valid


class UARTController:
    """
    UART communication controller for STM32.
    
    Command Protocol (ASCII):
        E1 / E0     - Enable / Disable control
        Vxxx        - Forward velocity (m/s × 1000), e.g., V800 = 0.8 m/s
        Yxxx        - Yaw rate (rad/s × 1000), e.g., Y500 = 0.5 rad/s
        Hxxx        - Leg height (m × 1000), e.g., H80 = 0.08 m
        Rxxx        - Roll angle (rad × 1000), e.g., R100 = 0.1 rad
        Lxxx        - Line error (-1000 to 1000), e.g., L-500 = -0.5
        J1          - Trigger jump
        Cxxx        - PWM duty (debug)
        ?           - Heartbeat ping (expects '!' response)
    
    Format: <COMMAND><VALUE>\n
    All commands are UPPERCASE.
    """

    # Command constants
    CMD_ENABLE = "E1"
    CMD_DISABLE = "E0"
    CMD_VELOCITY = "V"
    CMD_YAW_RATE = "Y"
    CMD_LEG_HEIGHT = "H"
    CMD_ROLL = "R"
    CMD_JUMP = "J1"
    CMD_PWM = "C"
    CMD_HEARTBEAT = "?"

    # Watchdog settings
    HEARTBEAT_INTERVAL = 0.5  # Send heartbeat every 500ms
    HEARTBEAT_TIMEOUT = 1.5   # Consider disconnected after 1.5s without response
    MAX_MISSED_HEARTBEATS = 3  # Emergency stop after 3 missed heartbeats

    def __init__(
        self,
        port: str = config.UART_PORT,
        baudrate: int = config.UART_BAUDRATE,
        timeout: float = config.UART_TIMEOUT
    ):
        """
        Initialize UART controller.

        Args:
            port: Serial port path
            baudrate: Communication baud rate
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self._serial: Optional[serial.Serial] = None
        self._is_connected = False
        self._is_enabled = False

        # Command rate control
        self._command_period = 1.0 / config.UART_COMMAND_RATE_HZ
        self._last_command_time = 0.0

        # Thread-safe command queue
        self._command_queue: Queue = Queue()
        self._send_thread: Optional[threading.Thread] = None
        self._running = False

        # Watchdog/Heartbeat
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._last_heartbeat_response = 0.0
        self._missed_heartbeats = 0
        self._heartbeat_callback: Optional[Callable[[bool], None]] = None

        # Last sent values for change detection
        self._last_velocity = None
        self._last_yaw_rate = None
        self._last_roll = None
        self._last_leg_height = None

        # Feedback from STM32
        self._feedback = RobotFeedback()
        self._feedback_lock = threading.Lock()
        self._receive_thread: Optional[threading.Thread] = None
        self._feedback_callback: Optional[Callable[[RobotFeedback], None]] = None

    def connect(self) -> bool:
        """
        Establish serial connection.

        Returns:
            True if successful, False otherwise
        """
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=config.UART_BYTESIZE,
                parity=config.UART_PARITY,
                stopbits=config.UART_STOPBITS,
                timeout=self.timeout
            )

            # Flush buffers
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

            self._is_connected = True
            self._last_heartbeat_response = time.time()
            logger.info(f"UART connected: {self.port} @ {self.baudrate}")

            # Start send thread
            self._running = True
            self._send_thread = threading.Thread(
                target=self._send_loop,
                daemon=True
            )
            self._send_thread.start()

            # Start heartbeat thread
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop,
                daemon=True
            )
            self._heartbeat_thread.start()

            # Start receive thread for feedback
            self._receive_thread = threading.Thread(
                target=self._receive_loop,
                daemon=True
            )
            self._receive_thread.start()

            return True

        except serial.SerialException as e:
            logger.error(f"Failed to connect UART: {e}")
            self._is_connected = False
            return False

    def disconnect(self) -> None:
        """Close serial connection."""
        self._running = False

        # Wait for threads to finish
        if self._send_thread is not None:
            self._send_thread.join(timeout=1.0)
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=1.0)
        if self._receive_thread is not None:
            self._receive_thread.join(timeout=1.0)

        if self._serial is not None and self._serial.is_open:
            # Send disable command before disconnecting
            self._send_command_direct(self.CMD_DISABLE)
            self._serial.close()

        self._is_connected = False
        self._is_enabled = False
        logger.info("UART disconnected")

    def enable_control(self) -> bool:
        """
        Enable robot control.

        Returns:
            True if successful
        """
        if not self._is_connected:
            logger.warning("Cannot enable: UART not connected")
            return False

        # Wait for STM32 to be ready after connect
        time.sleep(0.1)
        
        # Send enable command with retry
        logger.info(f"Sending ENABLE command: {self.CMD_ENABLE}")
        success = False
        for attempt in range(3):  # Retry up to 3 times
            if self._send_command_direct(self.CMD_ENABLE):
                success = True
                logger.info(f">>> SENT: {self.CMD_ENABLE} (attempt {attempt + 1})")
                time.sleep(0.05)  # Small delay between commands
                break
            else:
                logger.warning(f"Failed to send E1, attempt {attempt + 1}")
                time.sleep(0.1)
        
        if success:
            # Set initial leg height
            height_cmd = self._format_leg_height(config.LEG_HEIGHT)
            self._send_command_direct(height_cmd)
            logger.info(f">>> SENT: {height_cmd}")
            self._is_enabled = True
            logger.info("Robot control ENABLED successfully")
        else:
            logger.error("FAILED to enable robot control after 3 attempts!")

        return success

    def disable_control(self) -> bool:
        """
        Disable robot control.

        Returns:
            True if successful
        """
        if not self._is_connected:
            return False

        # Stop motion first
        self._send_command_direct(self._format_velocity(0.0))
        self._send_command_direct(self._format_yaw_rate(0.0))

        success = self._send_command_direct(self.CMD_DISABLE)
        if success:
            self._is_enabled = False
            logger.info("Robot control disabled")

        return success

    def send_motion_command(
        self, 
        velocity: float, 
        yaw_rate: float,
        leg_height: float = None,
        roll: float = None
    ) -> None:
        """
        Queue motion command for sending.
        
        Commands are sent at the configured rate (~10Hz).

        Args:
            velocity: Linear velocity in m/s
            yaw_rate: Angular velocity in rad/s
            leg_height: Leg height in meters (optional)
            roll: Roll angle in rad (optional)
        """
        if not self._is_connected:
            logger.warning("send_motion_command: UART not connected!")
            return
        if not self._is_enabled:
            logger.warning("send_motion_command: Motor control not enabled! Call enable_control() first.")
            return

        # Format commands
        vel_cmd = self._format_velocity(velocity)
        yaw_cmd = self._format_yaw_rate(yaw_rate)

        # Queue commands
        self._command_queue.put(vel_cmd)
        self._command_queue.put(yaw_cmd)
        
        # Queue optional commands
        if leg_height is not None:
            height_cmd = self._format_leg_height(leg_height)
            self._command_queue.put(height_cmd)
        
        if roll is not None:
            roll_cmd = self._format_roll(roll)
            self._command_queue.put(roll_cmd)

    def send_emergency_stop(self) -> None:
        """Send emergency stop commands immediately."""
        logger.warning("Sending emergency stop")

        # Clear queue
        while not self._command_queue.empty():
            try:
                self._command_queue.get_nowait()
            except Empty:
                break

        # Send stop commands directly
        self._send_command_direct(self._format_velocity(0.0))
        self._send_command_direct(self._format_yaw_rate(0.0))
        self._send_command_direct(self.CMD_DISABLE)

        self._is_enabled = False

    def send_jump(self) -> None:
        """Send jump command."""
        if not self._is_connected or not self._is_enabled:
            logger.warning("Cannot send jump: not connected or not enabled")
            return
        
        logger.info("Sending jump command")
        self._send_command_direct(self.CMD_JUMP)

    def send_roll(self, roll: float) -> None:
        """
        Send roll command.
        
        Args:
            roll: Roll angle in rad
        """
        if not self._is_connected or not self._is_enabled:
            logger.warning("Cannot send roll: not connected or not enabled")
            return
        
        roll_cmd = self._format_roll(roll)
        self._command_queue.put(roll_cmd)

    def send_pwm(self, duty: int) -> None:
        """
        Send PWM duty command (debug).
        
        Args:
            duty: PWM duty value
        """
        if not self._is_connected:
            logger.warning("Cannot send PWM: not connected")
            return
        
        pwm_cmd = self._format_pwm(duty)
        self._send_command_direct(pwm_cmd)

    def _send_loop(self) -> None:
        """Background thread for sending commands at controlled rate."""
        while self._running:
            current_time = time.time()
            elapsed = current_time - self._last_command_time

            if elapsed >= self._command_period:
                try:
                    # Get command from queue (non-blocking)
                    command = self._command_queue.get(timeout=0.01)
                    self._send_command_direct(command)
                    self._last_command_time = current_time
                except Empty:
                    pass
            else:
                # Sleep for remaining time
                time.sleep(self._command_period - elapsed)

    def _heartbeat_loop(self) -> None:
        """
        Background thread for sending heartbeat and checking connection health.
        Triggers emergency stop if STM32 becomes unresponsive.
        """
        logger.info("Heartbeat watchdog started")
        
        while self._running:
            time.sleep(self.HEARTBEAT_INTERVAL)
            
            if not self._is_connected:
                continue
            
            # Send heartbeat ping
            try:
                self._send_command_direct(self.CMD_HEARTBEAT)
                
                # Try to read response (non-blocking with short timeout)
                if self._serial and self._serial.is_open:
                    self._serial.timeout = 0.1
                    response = self._serial.readline().decode('ascii', errors='ignore').strip()
                    self._serial.timeout = self.timeout
                    
                    if response == '!' or response.startswith('!'):
                        # Valid heartbeat response
                        self._last_heartbeat_response = time.time()
                        self._missed_heartbeats = 0
                    elif response.startswith('R=') or response.startswith('OK') or response == 'ERR':
                        # Valid robot response (your robot's format)
                        self._last_heartbeat_response = time.time()
                        self._missed_heartbeats = 0
                    elif response:
                        # Any non-empty response counts as heartbeat
                        self._last_heartbeat_response = time.time()
                        self._missed_heartbeats = 0
                    else:
                        self._missed_heartbeats += 1
                        # Comment out warning to reduce noise since this robot uses different protocol
                        # logger.warning(f"Invalid heartbeat response: '{response}' (missed: {self._missed_heartbeats})")
                        
            except Exception as e:
                self._missed_heartbeats += 1
                logger.warning(f"Heartbeat error: {e} (missed: {self._missed_heartbeats})")
            
            # Check if connection is lost
            time_since_response = time.time() - self._last_heartbeat_response
            
            if time_since_response > self.HEARTBEAT_TIMEOUT:
                logger.error(f"Heartbeat timeout! Last response: {time_since_response:.1f}s ago")
                
                if self._missed_heartbeats >= self.MAX_MISSED_HEARTBEATS:
                    logger.critical("CONNECTION LOST - Triggering emergency stop")
                    self._handle_connection_lost()
            
            # Notify callback if set
            if self._heartbeat_callback:
                is_healthy = self._missed_heartbeats == 0
                self._heartbeat_callback(is_healthy)

    def _receive_loop(self) -> None:
        """
        Background thread for receiving feedback from STM32.
        Parses feedback messages in format: F<v>,<x>,<yaw>,<yaw_rate>
        """
        logger.info("Feedback receive thread started")
        
        while self._running:
            if not self._is_connected or self._serial is None:
                time.sleep(0.01)
                continue
                
            try:
                if self._serial.in_waiting > 0:
                    line = self._serial.readline().decode('ascii', errors='ignore').strip()
                    
                    if line.startswith('F'):
                        self._parse_feedback(line)
                    elif line.startswith('OK') or line == 'ERR' or line == '!':
                        # Response to commands, count as heartbeat
                        self._last_heartbeat_response = time.time()
                        self._missed_heartbeats = 0
                else:
                    time.sleep(0.005)  # Small sleep to prevent busy loop
                    
            except Exception as e:
                logger.debug(f"Receive error: {e}")
                time.sleep(0.01)
    
    def _parse_feedback(self, line: str) -> None:
        """
        Parse feedback message from STM32.
        Format: F<velocity>,<position>,<yaw>,<yaw_rate>
        Example: F0.523,1.234,0.785,0.100
        
        Args:
            line: Feedback line starting with 'F'
        """
        try:
            # Remove 'F' prefix and split
            data = line[1:].split(',')
            
            if len(data) >= 4:
                with self._feedback_lock:
                    self._feedback.velocity = float(data[0])
                    self._feedback.position = float(data[1])
                    self._feedback.yaw = float(data[2])
                    self._feedback.yaw_rate = float(data[3])
                    self._feedback.timestamp = time.time()
                    self._feedback.valid = True
                
                # Count as valid communication
                self._last_heartbeat_response = time.time()
                self._missed_heartbeats = 0
                
                # Call feedback callback if set
                if self._feedback_callback:
                    self._feedback_callback(self._feedback)
                    
                logger.debug(f"Feedback: v={self._feedback.velocity:.3f}, "
                            f"x={self._feedback.position:.3f}, "
                            f"yaw={self._feedback.yaw:.3f}")
                            
        except (ValueError, IndexError) as e:
            logger.warning(f"Failed to parse feedback '{line}': {e}")
    
    def get_feedback(self) -> RobotFeedback:
        """
        Get latest feedback from STM32.
        
        Returns:
            RobotFeedback dataclass with latest values
        """
        with self._feedback_lock:
            return RobotFeedback(
                velocity=self._feedback.velocity,
                position=self._feedback.position,
                yaw=self._feedback.yaw,
                yaw_rate=self._feedback.yaw_rate,
                timestamp=self._feedback.timestamp,
                valid=self._feedback.valid
            )
    
    def set_feedback_callback(self, callback: Callable[['RobotFeedback'], None]) -> None:
        """
        Set callback function for new feedback data.
        
        Args:
            callback: Function that takes RobotFeedback
        """
        self._feedback_callback = callback
    
    def reset_position(self) -> None:
        """Reset position estimate on STM32 (send special command)."""
        # Could add a reset command if needed
        with self._feedback_lock:
            self._feedback.position = 0.0

    def _handle_connection_lost(self) -> None:
        """Handle lost connection to STM32."""
        self._is_enabled = False
        
        # Try to send stop commands anyway
        try:
            self._send_command_direct(self._format_velocity(0.0))
            self._send_command_direct(self._format_yaw_rate(0.0))
            self._send_command_direct(self.CMD_DISABLE)
        except:
            pass
        
        logger.error("Robot disabled due to connection loss")

    def set_heartbeat_callback(self, callback: Callable[[bool], None]) -> None:
        """
        Set callback function for heartbeat status updates.
        
        Args:
            callback: Function that takes bool (True=healthy, False=issues)
        """
        self._heartbeat_callback = callback

    def get_connection_health(self) -> dict:
        """
        Get current connection health status.
        
        Returns:
            Dict with health metrics
        """
        return {
            'is_connected': self._is_connected,
            'is_enabled': self._is_enabled,
            'missed_heartbeats': self._missed_heartbeats,
            'last_response_age': time.time() - self._last_heartbeat_response,
            'healthy': self._missed_heartbeats == 0 and self._is_connected
        }

    def _send_command_direct(self, command: str) -> bool:
        """
        Send command directly to serial port.

        Args:
            command: Command string (without newline)

        Returns:
            True if successful
        """
        if self._serial is None or not self._serial.is_open:
            return False

        try:
            # Add newline and encode
            data = (command + "\n").encode('ascii')
            bytes_written = self._serial.write(data)
            self._serial.flush()
            logger.info(f"UART TX: '{command}' ({bytes_written} bytes)")
            return True

        except serial.SerialException as e:
            logger.error(f"UART send error: {e}")
            return False

    def _format_velocity(self, velocity: float) -> str:
        """
        Format velocity command.

        Args:
            velocity: Velocity in m/s

        Returns:
            Formatted command string (e.g., V800 for 0.8 m/s)
        """
        # Scale by 1000 to match STM32 protocol (value * 0.001)
        scaled = int(velocity * 1000)
        return f"{self.CMD_VELOCITY}{scaled}"

    def _format_yaw_rate(self, yaw_rate: float) -> str:
        """
        Format yaw rate command.

        Args:
            yaw_rate: Yaw rate in rad/s

        Returns:
            Formatted command string (e.g., Y500 for 0.5 rad/s)
        """
        # Scale by 1000 to match STM32 protocol (value * 0.001)
        scaled = int(yaw_rate * 1000)
        return f"{self.CMD_YAW_RATE}{scaled}"

    def _format_leg_height(self, height: float) -> str:
        """
        Format leg height command.

        Args:
            height: Height in meters

        Returns:
            Formatted command string (e.g., H80 for 0.08m)
        """
        # Scale by 1000 to match STM32 protocol (value * 0.001)
        scaled = int(height * 1000)
        return f"{self.CMD_LEG_HEIGHT}{scaled}"

    def _format_roll(self, roll: float) -> str:
        """
        Format roll command.

        Args:
            roll: Roll angle in rad

        Returns:
            Formatted command string (e.g., R100 for 0.1 rad)
        """
        # Scale by 1000 to match STM32 protocol (value * 0.001)
        scaled = int(roll * 1000)
        return f"{self.CMD_ROLL}{scaled}"

    def _format_pwm(self, duty: int) -> str:
        """
        Format PWM duty command.

        Args:
            duty: PWM duty value

        Returns:
            Formatted command string
        """
        return f"{self.CMD_PWM}{duty}"

    @property
    def is_connected(self) -> bool:
        """Check if UART is connected."""
        return self._is_connected

    @property
    def is_enabled(self) -> bool:
        """Check if robot control is enabled."""
        return self._is_enabled

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False


class MockUARTController(UARTController):
    """
    Mock UART controller for testing without hardware.
    Logs commands instead of sending them.
    Simulates heartbeat responses.
    """

    def __init__(self, *args, **kwargs):
        """Initialize mock controller."""
        super().__init__(*args, **kwargs)
        self._command_log = []
        self._simulate_disconnect = False  # Set True to simulate connection loss

    def connect(self) -> bool:
        """Simulate connection."""
        logger.info("Mock UART connected (simulation mode)")
        self._is_connected = True
        self._last_heartbeat_response = time.time()

        self._running = True
        self._send_thread = threading.Thread(
            target=self._send_loop,
            daemon=True
        )
        self._send_thread.start()

        # Mock heartbeat thread (always responds)
        self._heartbeat_thread = threading.Thread(
            target=self._mock_heartbeat_loop,
            daemon=True
        )
        self._heartbeat_thread.start()

        return True

    def disconnect(self) -> None:
        """Simulate disconnection."""
        self._running = False
        if self._send_thread is not None:
            self._send_thread.join(timeout=1.0)
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=1.0)
        self._is_connected = False
        self._is_enabled = False
        logger.info("Mock UART disconnected")

    def _mock_heartbeat_loop(self) -> None:
        """Simulate heartbeat responses."""
        while self._running:
            time.sleep(self.HEARTBEAT_INTERVAL)
            
            if not self._simulate_disconnect:
                # Simulate successful heartbeat
                self._last_heartbeat_response = time.time()
                self._missed_heartbeats = 0
            else:
                # Simulate connection loss
                self._missed_heartbeats += 1
                if self._missed_heartbeats >= self.MAX_MISSED_HEARTBEATS:
                    self._handle_connection_lost()

    def simulate_disconnect(self, disconnect: bool = True) -> None:
        """
        Simulate connection loss for testing.
        
        Args:
            disconnect: True to simulate disconnect, False to restore
        """
        self._simulate_disconnect = disconnect
        if disconnect:
            logger.warning("Mock: Simulating connection loss")
        else:
            logger.info("Mock: Connection restored")
            self._missed_heartbeats = 0

    def _send_command_direct(self, command: str) -> bool:
        """Log command instead of sending."""
        self._command_log.append((time.time(), command))
        logger.debug(f"Mock sent: {command}")

        # Keep log size manageable
        if len(self._command_log) > 1000:
            self._command_log = self._command_log[-500:]

        return True

    def get_command_log(self) -> list:
        """Get logged commands."""
        return self._command_log.copy()
