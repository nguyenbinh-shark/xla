# UART Testing Tools

Interactive tools để test giao tiếp UART với STM32 hardware.

## 📋 Available Tools

### 1. **Command-Line Interface (CLI)** - Recommended for beginners
```bash
python tools/testing/test_uart_cli.py [--mock] [--port /dev/ttyUSB0]
```

**Ưu điểm:**
- Simple, dễ sử dụng
- Terminal-based, không cần display
- Có thể chạy SSH/remote
- Dễ debug

**Cách dùng:**
```
>> connect              # Kết nối
>> status               # Xem trạng thái
>> enable               # Bật điều khiển
>> velocity 0.5         # Set vận tốc 0.5 m/s
>> yaw 0.2              # Set yaw rate 0.2 rad/s
>> send                 # Gửi lệnh
>> heartbeat            # Test heartbeat
>> stop                 # Emergency stop
>> disconnect           # Ngắt kết nối
>> exit                 # Thoát
```

### 2. **Interactive GUI** - For visual feedback
```bash
python tools/testing/test_uart_interactive.py [--mock] [--port /dev/ttyUSB0]
```

**Ưu điểm:**
- Real-time visual display
- Keyboard controls (arrow keys, W/S)
- Status monitoring
- Command history

**Controls:**
- `↑↓` - Tăng/giảm velocity
- `←→` - Tăng/giảm yaw rate
- `W/S` - Tăng/giảm leg height
- `E` - Enable/Disable control
- `SPACE` - Emergency stop
- `R` - Reset to zero
- `P` - Print values
- `Q` - Quit

## 🔧 Quick Start

### Test with Mock UART (no hardware needed)
```bash
# CLI version
python tools/testing/test_uart_cli.py --mock

# GUI version
python tools/testing/test_uart_interactive.py --mock
```

### Test with Real Hardware
**Linux/Mac:**
```bash
# Tìm serial port
ls /dev/ttyUSB*  # or /dev/ttyACM*

# Test with CLI
python tools/testing/test_uart_cli.py --port /dev/ttyUSB0

# Test with GUI
python tools/testing/test_uart_interactive.py --port /dev/ttyUSB0
```

**Windows:**
```bash
# COM port typically COM3, COM4, etc
python tools/testing/test_uart_cli.py --port COM3
```

### Custom Baud Rate
```bash
python tools/testing/test_uart_cli.py --port /dev/ttyUSB0 --baudrate 115200
```

## 📊 UART Protocol Reference

### Command Format
All commands are ASCII-based, ending with `\n`:
```
<COMMAND><VALUE>\n
```

### Supported Commands
| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Enable | `E1\n` | `E1` | Enable motor control |
| Disable | `E0\n` | `E0` | Disable motor control |
| Velocity | `Vxxx\n` | `V800` | Set velocity 0.8 m/s (value × 1000) |
| Yaw Rate | `Yxxx\n` | `Y500` | Set yaw rate 0.5 rad/s (value × 1000) |
| Leg Height | `Hxxx\n` | `H80` | Set height 0.08 m (value × 1000) |
| Roll | `Rxxx\n` | `R100` | Set roll 0.1 rad (value × 1000) |
| Line Error | `Lxxx\n` | `L-500` | Set line error -0.5 (value × 1000) |
| Jump | `J1\n` | `J1` | Trigger jump |
| PWM | `Cxxx\n` | `C500` | Set PWM duty (debug) |
| Heartbeat | `?\n` | `?` | Ping (expects `!` response) |

**Note:** All command letters must be **UPPERCASE**.

### Examples
```
# Enable control
E1

# Set velocity to 0.8 m/s
V800

# Set yaw rate to 0.5 rad/s (turn left)
Y500

# Set yaw rate to -0.3 rad/s (turn right)
Y-300

# Set leg height to 0.08 m
H80

# Set roll angle to 0.1 rad
R100

# Line following error (line is to the left)
L-500

# Trigger jump
J1

# Send heartbeat
?
```

## 🧪 Testing Workflow

### 1. **Connection Test**
```bash
$ python tools/testing/test_uart_cli.py
>> connect
>> status
```
Expected output:
```
✅ Connected!
CONNECTION STATUS:
  Connected:       ✅ YES
  Control Enabled: ❌ NO
  Uptime:          1.2s
```

### 2. **Enable Control**
```bash
>> enable
>> status
```
Expected:
```
✅ Control enabled!
CONNECTION STATUS:
  Connected:       ✅ YES
  Control Enabled: ✅ YES
```

### 3. **Motion Test**
```bash
>> velocity 0.3
>> yaw 0.1
>> send
```

### 4. **Heartbeat Test**
```bash
>> heartbeat
>> heartbeat
```
Monitor logs to see heartbeat responses.

### 5. **Emergency Stop**
```bash
>> stop
```
Should set velocity and yaw to 0.

## 📈 Monitoring

### View Logs
```bash
# Real-time logging
python tools/testing/test_uart_cli.py 2>&1 | tee uart_test.log

# Colorized output
python tools/testing/test_uart_cli.py | less -R
```

### STM32 Serial Monitor
Monitor the STM32 side:
```bash
# Linux/Mac - using minicom
minicom -D /dev/ttyUSB0 -b 115200

# Using picocom
picocom /dev/ttyUSB0 -b 115200 -l

# Using screen
screen /dev/ttyUSB0 115200
```

## 🐛 Troubleshooting

### "Connection failed"
- Kiểm tra kết nối USB
- Xác nhận port đúng: `ls /dev/ttyUSB*`
- Kiểm tra baud rate
- Có thể cần chmod: `sudo chmod 666 /dev/ttyUSB0`

### "Heartbeat timeout"
- STM32 firmware có thể không phản hồi
- Kiểm tra STM32 code
- Verify UART connections

### No response from STM32
- Check STM32 is powered on
- Verify TX/RX connections
- Test with minicom/picocom to confirm hardware works

## 💡 Tips

1. **Start with mock mode** để test UI/logic
2. **Monitor logs** để debug communication
3. **Test heartbeat first** trước khi gửi motion commands
4. **Emergency stop** (SPACE hay `stop`) nếu có vấn đề
5. **Reset values** trước khi disconnect

## 📝 Example Session

```bash
$ python tools/testing/test_uart_cli.py --port /dev/ttyUSB0

>> connect
🔌 Connecting to /dev/ttyUSB0...
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

>> heartbeat
❤️  Sending heartbeat...
✅ Heartbeat sent!

>> status
CONNECTION STATUS:
  Connected:       ✅ YES
  Control Enabled: ✅ YES
  Uptime:          15.3s

CURRENT VALUES:
  Velocity:        +0.500 m/s
  Yaw Rate:        +0.200 rad/s
  Leg Height:      0.050 m

STATISTICS:
  Commands sent:   4
  Errors:          0

>> stop
🛑 EMERGENCY STOP!

>> disconnect
🔌 Disconnecting...
✅ Disconnected!

>> exit
👋 Goodbye!
```

## 🔗 Related Files

- [UARTController](../../src/communication/uart_controller.py) - Main UART implementation
- [Config](../../src/core/config.py) - UART settings (port, baudrate, etc.)
- [MockUART](../../src/communication/__init__.py) - Mock for testing

