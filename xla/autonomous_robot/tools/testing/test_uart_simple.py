#!/usr/bin/env python3
"""
Simple UART Command Testing Tool - Công cụ Kiểm tra UART Đơn Giản

Simple command-line interface for testing UART communication with STM32 robot.
Supports direct command sending following the protocol table.

Usage:
    python tools/testing/test_uart_simple.py [--port /dev/ttyACM0] [--mock]

Examples:
    E1           - Enable control
    E0           - Disable control
    V800         - Set velocity to 0.8 m/s
    V-500        - Set velocity to -0.5 m/s (backward)
    Y500         - Set yaw rate to 0.5 rad/s
    Y-300        - Set yaw rate to -0.3 rad/s (turn left)
    H100         - Set leg height to 0.1 m
    R100         - Set roll angle to 0.1 rad
    R-50         - Set roll angle to -0.05 rad
    J1           - Trigger jump
    ?            - Send heartbeat
    help         - Show this help
    exit         - Exit program
"""

import sys
import time
import argparse
import logging
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.communication import UARTController, MockUARTController
from src.core import config

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Command reference table
COMMAND_TABLE = """
╔════════════════════════════════════════════════════════════════════════════════╗
║                    BẢNG LỆNH UART / UART COMMAND TABLE                        ║
╚════════════════════════════════════════════════════════════════════════════════╝

| Lệnh    | Chức năng              | Đơn vị gốc | Hệ số  | Ví dụ     | Kết quả               |
|---------|------------------------|------------|--------|-----------|------------------------|
| **E1**  | Bật điều khiển         | -          | -      | E1\n      | Enable control mode   |
| **E0**  | Tắt điều khiển         | -          | -      | E0\n      | Disable (hold pose)   |
| **Vxxx**| Vận tốc tiến/lùi       | m/s        | ×1000  | V800\n    | 0.8 m/s forward       |
| **V-xx**| Vận tốc lùi            | m/s        | ×1000  | V-500\n   | -0.5 m/s backward     |
| **Yxxx**| Tốc độ quay (yaw rate) | rad/s      | ×1000  | Y500\n    | 0.5 rad/s turn right  |
| **Y-xx**| Quay trái              | rad/s      | ×1000  | Y-300\n   | -0.3 rad/s turn left  |
| **Hxxx**| Chiều cao chân         | m          | ×1000  | H100\n    | 0.1 m = 10 cm         |
| **Rxxx**| Góc roll (nghiêng)     | rad        | ×1000  | R100\n    | 0.1 rad tilt          |
| **R-xx**| Roll âm                | rad        | ×1000  | R-50\n    | -0.05 rad             |
| **J1**  | Nhảy                   | -          | -      | J1\n      | Trigger jump          |
| **?**   | Heartbeat              | -          | -      | ?\n       | Check connection      |

Ghi chú / Notes:
  - Tất cả giá trị số được mã hóa ở định dạng ASCII
  - Kết thúc lệnh bằng \\n (Commands end with \\n)
  - Để gửi giá trị âm, sử dụng ký tự '-' (Use '-' for negative values)
  - Robot phải ở trạng thái enabled (E1) để nhận lệnh chuyển động
╚════════════════════════════════════════════════════════════════════════════════╝
"""


class SimpleUARTTester:
    """Simple command-line UART testing interface."""
    
    def __init__(self, uart_controller):
        self.uart = uart_controller
        self.is_connected = False
        self.is_enabled = False
        self.cmd_count = 0
        self.error_count = 0
        self.start_time = None
    
    def print_welcome(self):
        """Print welcome message."""
        print("\n" + "="*80)
        print("  SIMPLE UART TESTING TOOL - Công cụ Kiểm tra UART Đơn Giản")
        print("="*80)
        if hasattr(self.uart, 'port'):
            print(f"Port: {self.uart.port} | Baudrate: {self.uart.baudrate}")
        print("Type 'help' for help or 'table' to see command reference")
        print("="*80 + "\n")
    
    def connect(self) -> bool:
        """Connect to UART."""
        print(f"🔌 Connecting...")
        if not self.uart.connect():
            logger.error("❌ Connection failed!")
            self.error_count += 1
            return False
        
        print("✅ Connected!")
        self.is_connected = True
        self.start_time = time.time()
        return True
    
    def disconnect(self) -> bool:
        """Disconnect from UART."""
        if not self.is_connected:
            logger.warning("Already disconnected")
            return True
        
        print("🔌 Disconnecting...")
        if self.is_enabled:
            self.send_command("E0")
        
        self.uart.disconnect()
        self.is_connected = False
        self.is_enabled = False
        print("✅ Disconnected!")
        return True
    
    def send_command(self, command: str) -> bool:
        """Send raw UART command."""
        if not self.is_connected:
            logger.error("❌ Not connected!")
            self.error_count += 1
            return False
        
        try:
            cmd = command.strip().upper()
            
            # Parse command and provide feedback
            if cmd == "E1":
                self.uart.enable_control()
                self.is_enabled = True
                print("✅ Control enabled (E1)")
            
            elif cmd == "E0":
                self.uart.disable_control()
                self.is_enabled = False
                print("✅ Control disabled (E0)")
            
            elif cmd.startswith("V"):
                # Velocity command
                try:
                    value = int(cmd[1:])
                    velocity = value / 1000.0  # Convert from scaled value
                    self.uart._send_command_direct(cmd)
                    print(f"📤 Velocity: {velocity:+.3f} m/s (raw: {cmd})")
                except (ValueError, IndexError):
                    print(f"❌ Invalid velocity format: {cmd}")
                    self.error_count += 1
                    return False
            
            elif cmd.startswith("Y"):
                # Yaw rate command
                try:
                    value = int(cmd[1:])
                    yaw = value / 1000.0  # Convert from scaled value
                    self.uart._send_command_direct(cmd)
                    print(f"📤 Yaw rate: {yaw:+.3f} rad/s (raw: {cmd})")
                except (ValueError, IndexError):
                    print(f"❌ Invalid yaw format: {cmd}")
                    self.error_count += 1
                    return False
            
            elif cmd.startswith("H"):
                # Height command
                try:
                    value = int(cmd[1:])
                    height = value / 1000.0  # Convert from scaled value
                    self.uart._send_command_direct(cmd)
                    print(f"📤 Leg height: {height:.3f} m (raw: {cmd})")
                except (ValueError, IndexError):
                    print(f"❌ Invalid height format: {cmd}")
                    self.error_count += 1
                    return False
            
            elif cmd.startswith("R"):
                # Roll command
                try:
                    value = int(cmd[1:])
                    roll = value / 1000.0  # Convert from scaled value
                    self.uart._send_command_direct(cmd)
                    print(f"📤 Roll angle: {roll:+.3f} rad (raw: {cmd})")
                except (ValueError, IndexError):
                    print(f"❌ Invalid roll format: {cmd}")
                    self.error_count += 1
                    return False
            
            elif cmd == "J1":
                self.uart.send_jump()
                print("📤 Jump triggered (J1)")
            
            elif cmd == "?":
                self.uart._send_command_direct("?")
                print("📤 Heartbeat sent (?)")
            
            else:
                print(f"❌ Unknown command: {cmd}")
                self.error_count += 1
                return False
            
            self.cmd_count += 1
            return True

        except Exception as e:
            logger.error(f"❌ Error sending command: {e}")
            self.error_count += 1
            return False

    def print_status(self):
        """Print current status."""
        print("\n" + "-"*60)
        print("STATUS:")
        print(f"  Connected:       {'✅ YES' if self.is_connected else '❌ NO'}")
        print(f"  Control Enabled: {'✅ YES' if self.is_enabled else '❌ NO'}")
        
        if self.is_connected and self.start_time:
            uptime = time.time() - self.start_time
            print(f"  Uptime:          {uptime:.1f}s")
        
        print(f"  Commands sent:   {self.cmd_count}")
        print(f"  Errors:          {self.error_count}")
        print("-"*60 + "\n")
    
    def print_help(self):
        """Print help message."""
        help_text = """
╔═══════════════════════════════════════════════════════════════════════════════╗
║                        AVAILABLE COMMANDS / LỆNH CÓ SẴN                      ║
╚═══════════════════════════════════════════════════════════════════════════════╝

CONNECTION:
  connect              Kết nối / Connect to UART
  disconnect           Ngắt kết nối / Disconnect
  status               Hiển thị trạng thái / Show status

CONTROL:
  E1                   Bật điều khiển / Enable control
  E0                   Tắt điều khiển / Disable control

MOTION (require E1 enabled):
  V<value>             Vận tốc (m/s × 1000) / Velocity
                       Examples: V800, V-500
  Y<value>             Tốc độ quay (rad/s × 1000) / Yaw rate
                       Examples: Y500, Y-300
  H<value>             Chiều cao chân (m × 1000) / Leg height
                       Examples: H100, H150
  R<value>             Góc roll (rad × 1000) / Roll angle
                       Examples: R100, R-50
  J1                   Nhảy / Jump

TESTING:
  ?                    Heartbeat ping
  table                Hiển thị bảng lệnh / Show command reference
  help                 Hiển thị trợ giúp này / Show this help

EXIT:
  exit, quit           Thoát chương trình / Exit

═══════════════════════════════════════════════════════════════════════════════

EXAMPLES / VÍ DỤ:
  > connect
  > E1
  > V800               (0.8 m/s forward)
  > Y500               (0.5 rad/s turn)
  > H100               (0.1 m height)
  > R50                (0.05 rad roll)
  > J1                 (Jump)
  > E0                 (Disable control)
  > disconnect

═══════════════════════════════════════════════════════════════════════════════
"""
        print(help_text)


def main():
    parser = argparse.ArgumentParser(
        description="Simple UART Command Testing Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python tools/testing/test_uart_simple.py --mock
  python tools/testing/test_uart_simple.py --port /dev/ttyACM0
        """
    )
    parser.add_argument('--port', type=str, default=config.UART_PORT,
                       help=f'Serial port (default: {config.UART_PORT})')
    parser.add_argument('--mock', action='store_true',
                       help='Use mock UART (no hardware required)')
    parser.add_argument('--baudrate', type=int, default=config.UART_BAUDRATE,
                       help=f'Baud rate (default: {config.UART_BAUDRATE})')
    
    args = parser.parse_args()
    
    # Create UART controller
    if args.mock:
        logger.info("Using MOCK UART")
        uart = MockUARTController()
    else:
        logger.info(f"Using REAL UART: {args.port}")
        uart = UARTController(port=args.port, baudrate=args.baudrate)
    
    # Create tester
    tester = SimpleUARTTester(uart)
    tester.print_welcome()
    
    try:
        while True:
            try:
                # Read input
                user_input = input(">> ").strip()
                
                if not user_input:
                    continue
                
                # Parse command
                parts = user_input.split()
                cmd = parts[0].lower()
                
                # Connection commands
                if cmd == 'connect':
                    tester.connect()
                
                elif cmd == 'disconnect':
                    tester.disconnect()
                
                elif cmd == 'status':
                    tester.print_status()
                
                # Command commands (pass through)
                elif cmd in ['e0', 'e1', 'j1', '?']:
                    tester.send_command(cmd.upper())
                
                elif cmd.startswith('v') or cmd.startswith('y') or cmd.startswith('h') or cmd.startswith('r'):
                    tester.send_command(cmd.upper())
                
                # Info commands
                elif cmd == 'help':
                    tester.print_help()
                
                elif cmd == 'table':
                    print(COMMAND_TABLE)
                
                # Exit
                elif cmd in ['exit', 'quit']:
                    print("👋 Goodbye!")
                    break
                
                else:
                    print(f"❌ Unknown command: '{user_input}'")
                    print("   Type 'help' for available commands")
            
            except KeyboardInterrupt:
                print("\n👋 Interrupted by user")
                break
    
    finally:
        logger.info("Cleaning up...")
        if tester.is_connected:
            tester.disconnect()
        print("✅ Done!")


if __name__ == "__main__":
    main()
