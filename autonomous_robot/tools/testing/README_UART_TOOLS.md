# UART Interactive Testing Tools - Summary

## ✅ What's New

Tôi đã tạo **2 interactive testing tools** cho UART communication với STM32:

### 1. **CLI Tool** (Command-Line Interface)
📁 `tools/testing/test_uart_cli.py`

**Đặc điểm:**
- ✅ Terminal-based interface
- ✅ Dễ sử dụng, dễ debug
- ✅ Có thể chạy SSH/remote
- ✅ Mock mode support
- ✅ Status monitoring
- ✅ Command history

**Cách dùng:**
```bash
# With real hardware
python tools/testing/test_uart_cli.py --port /dev/ttyUSB0

# With mock (no hardware needed)
python tools/testing/test_uart_cli.py --mock
```

**Commands:**
```
connect              # Kết nối
status               # Xem trạng thái
enable               # Bật điều khiển
velocity 0.5         # Set vận tốc
yaw 0.2              # Set yaw rate
send                 # Gửi lệnh
heartbeat            # Test heartbeat
stop                 # Emergency stop
disconnect           # Ngắt kết nối
help                 # Xem help
exit                 # Thoát
```

### 2. **Interactive GUI Tool**
📁 `tools/testing/test_uart_interactive.py`

**Đặc điểm:**
- ✅ OpenCV-based GUI display
- ✅ Real-time status monitoring
- ✅ Keyboard controls
- ✅ Command history
- ✅ Visual feedback

**Cách dùng:**
```bash
python tools/testing/test_uart_interactive.py [--mock] [--port /dev/ttyUSB0]
```

**Keyboard Controls:**
```
Arrow Keys (↑↓) → Velocity control
Arrow Keys (←→) → Yaw rate control
W/S             → Leg height control
E               → Enable/Disable
SPACE           → Emergency stop
R               → Reset values
P               → Print status
Q               → Quit
```

## 🚀 Quick Start

### Test without hardware (Mock mode)
```bash
# CLI
cd autonomous_robot
python tools/testing/test_uart_cli.py --mock

# GUI
python tools/testing/test_uart_interactive.py --mock
```

### Test with real STM32
```bash
# Find serial port
ls /dev/ttyUSB*          # Linux
COM ports               # Windows

# CLI
python tools/testing/test_uart_cli.py --port /dev/ttyUSB0 --baudrate 115200

# GUI
python tools/testing/test_uart_interactive.py --port /dev/ttyUSB0
```

## 📊 Typical Testing Workflow

### Using CLI:
```bash
$ python tools/testing/test_uart_cli.py --mock

>> connect
🔌 Connecting...
✅ Connected!

>> enable
⚡ Enabling control...
✅ Control enabled!

>> velocity 0.5
📊 Velocity set to: +0.500 m/s

>> yaw 0.2
📊 Yaw rate set to: +0.200 rad/s

>> send
📤 Sending: V=+0.500m/s, Y=+0.200rad/s
✅ Sent!

>> status
[Shows connection status, values, statistics]

>> stop
🛑 EMERGENCY STOP!

>> disconnect
✅ Disconnected!

>> exit
```

## 📝 Protocol Reference

### Command Format
```
<COMMAND><VALUE>\n
```

### Supported Commands
| Cmd | Value | Example | Meaning |
|-----|-------|---------|---------|
| E1 | - | E1 | Enable control |
| E0 | - | E0 | Disable control |
| V | ±5000 | V+500 | Velocity (val/1000) |
| Y | ±3000 | Y+200 | Yaw rate (val/1000) |
| H | 0-100 | H050 | Leg height (val/1000) |
| ? | - | ? | Heartbeat ping |

## 🔍 Features

### CLI Tool Features
- **Connection Management**
  - Connect/disconnect with error handling
  - Heartbeat/watchdog monitoring
  - Auto-reconnect capability

- **Command Sending**
  - Velocity control (-2.0 to +2.0 m/s)
  - Yaw rate control (-3.0 to +3.0 rad/s)
  - Leg height adjustment (0.0 to 0.1 m)
  - Emergency stop
  - Heartbeat testing

- **Monitoring**
  - Real-time connection status
  - Command statistics (sent/errors)
  - Uptime tracking
  - Command history

### GUI Tool Features
- All CLI features plus:
- **Visual Display**
  - Status dashboard
  - Current values
  - Statistics
  - Command history
  - Color-coded status indicators

- **Keyboard Controls**
  - Arrow keys for analog control
  - Real-time value adjustment
  - Visual feedback

## 📚 Documentation

Detailed guide available in:
📄 `tools/testing/UART_TESTING_GUIDE.md`

Contains:
- Detailed command reference
- Troubleshooting guide
- Example sessions
- STM32 monitoring tips

## ✨ Benefits

1. **Easy Testing** - No need to write complex test code
2. **Hardware Validation** - Verify STM32 communication works
3. **Debug Capability** - Monitor what's being sent/received
4. **Non-Intrusive** - Separate from main codebase
5. **Mock Support** - Test without hardware
6. **Real-Time Feedback** - See results immediately

## 🎯 Use Cases

### 1. Initial Hardware Setup
```bash
python tools/testing/test_uart_cli.py --port /dev/ttyUSB0
>> connect
>> status
```

### 2. Verify STM32 Firmware
```bash
>> enable
>> heartbeat
>> send
```

### 3. Test Motion Control
```bash
>> velocity 0.3
>> yaw 0.1
>> send
```

### 4. Debug Communication Issues
```bash
# Monitor in another terminal
minicom -D /dev/ttyUSB0 -b 115200

# Send commands in another window
python tools/testing/test_uart_cli.py --port /dev/ttyUSB0
```

### 5. Calibrate Offsets
Use GUI version with offset tuning to verify motor response

## 🔧 Configuration

Default settings in [src/core/config.py](../../src/core/config.py):
```python
UART_PORT = "/dev/ttyUSB0"
UART_BAUDRATE = 115200
UART_TIMEOUT = 1.0
UART_BYTESIZE = 8
UART_PARITY = "N"
UART_STOPBITS = 1
```

Override via command-line:
```bash
python tools/testing/test_uart_cli.py --port COM3 --baudrate 9600
```

## 🐛 Troubleshooting

### Connection Issues
```bash
# Check if port exists
ls /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Verify with minicom
minicom -D /dev/ttyUSB0 -b 115200
```

### No Response
- Verify STM32 is powered on
- Check TX/RX connections
- Verify baud rate matches
- Test with debug firmware on STM32

### Heartbeat Timeout
- STM32 firmware may not support heartbeat response
- Verify response format: should be `!`

## 📦 Files Created

```
tools/testing/
├── test_uart_cli.py           # Main CLI tool
├── test_uart_interactive.py   # GUI tool
└── UART_TESTING_GUIDE.md      # Detailed documentation
```

## 🚀 Next Steps

1. ✅ Test with mock: `python tools/testing/test_uart_cli.py --mock`
2. ✅ Connect real STM32 and run tests
3. ✅ Monitor STM32 output with minicom
4. ✅ Verify motion commands work
5. ✅ Integrate into main control system

---

**Status:** Ready for use! ✨
