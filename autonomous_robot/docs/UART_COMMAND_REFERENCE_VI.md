# UART Command Reference - STM32 Robot Control (Tiếng Việt)

## Tổng Quan Giao Thức

- **Format**: `<COMMAND><VALUE>\n`
- **Encoding**: ASCII
- **Baudrate**: 115200
- **Unit scaling**: Giá trị được nhân với hệ số trước khi gửi

## Bảng Lệnh Chi Tiết

| Lệnh | Chức năng | Đơn vị gốc | Hệ số | Ví dụ | Kết quả |
|------|-----------|------------|-------|-------|---------|
| **E1** | Bật điều khiển | - | - | `E1\n` | Enable control mode |
| **E0** | Tắt điều khiển | - | - | `E0\n` | Disable (hold pose) |
| **Vxxx** | Vận tốc tiến/lùi | m/s | ×1000 | `V800\n` | 0.8 m/s forward |
| **V-xxx** | Vận tốc lùi | m/s | ×1000 | `V-500\n` | -0.5 m/s backward |
| **Yxxx** | Tốc độ quay (yaw rate) | rad/s | ×1000 | `Y500\n` | 0.5 rad/s turn right |
| **Y-xxx** | Quay trái | rad/s | ×1000 | `Y-300\n` | -0.3 rad/s turn left |
| **Hxxx** | Chiều cao chân | m | ×1000 | `H100\n` | 0.1 m = 10 cm |
| **Rxxx** | Góc roll (nghiêng) | rad | ×1000 | `R100\n` | 0.1 rad tilt |
| **R-xxx** | Roll âm | rad | ×1000 | `R-50\n` | -0.05 rad |
| **J1** | Nhảy | - | - | `J1\n` | Trigger jump |
| **Cxxx** | PWM duty (debug) | - | - | `C500\n` | Set TIM12 CCR2=500 |

## Phản Hồi từ STM32

- **Success**: `OK V=0.800\r\n` (tương ứng với lệnh)
- **Error**: `ERR\r\n` (sai format) hoặc `ERR CMD\r\n` (lệnh không tồn tại)

## Python API - Ví Dụ Sử Dụng

### Kết nối và Điều Khiển Cơ Bản

```python
from src.communication import UARTController

# Khởi tạo
uart = UARTController(port='/dev/ttyACM0')

# Kết nối
if uart.connect():
    print("Connected!")
    
    # Bật điều khiển
    uart.enable_control()
    
    # Gửi lệnh chuyển động
    uart.send_motion_command(
        velocity=0.5,      # 0.5 m/s
        yaw_rate=0.3,      # 0.3 rad/s
        leg_height=0.1,    # 0.1 m
        roll=0.05          # 0.05 rad
    )
    
    # Nhảy
    uart.send_jump()
    
    # Điều khiển Roll
    uart.send_roll(0.1)
    
    # Debug PWM
    uart.send_pwm(500)
    
    # Dừng khẩn cấp
    uart.send_emergency_stop()
    
    # Ngắt kết nối
    uart.disconnect()
```

### Command Line Tool

```bash
# Chế độ Mock (không cần phần cứng)
python tools/testing/test_uart_cli.py --mock

# Kết nối thực tế
python tools/testing/test_uart_cli.py --port /dev/ttyACM0

# Interactive Testing
python tools/testing/test_uart_interactive.py --port /dev/ttyACM0
```

## Giải Thích Chi Tiết các Lệnh

### Enable/Disable Control (E1/E0)
- **E1**: Bật chế độ điều khiển robot
- **E0**: Tắt chế độ, robot giữ tư thế hiện tại

### Motion Commands (V, Y, H, R)
Tất cả các lệnh chuyển động đều được chia tỷ lệ ×1000:
- **V500** = 0.5 m/s (nhân 0.5 với 1000)
- **Y300** = 0.3 rad/s
- **H150** = 0.15 m (15 cm)
- **R100** = 0.1 rad

### Jump Command (J1)
- Kích hoạt chế độ nhảy
- Không có tham số

### PWM Debug Command (C)
- Sử dụng cho debug phần cứng
- Thiết lập PWM duty trực tiếp

## Ghi Chú

1. Lệnh phải kết thúc bằng `\n` (newline)
2. Tất cả các giá trị số được mã hóa ở định dạng ASCII
3. Để gửi giá trị âm, sử dụng ký tự `-` (ví dụ: `V-500` cho -0.5 m/s)
4. Robot phải ở trạng thái enabled (E1) để nhận các lệnh chuyển động
5. Lệnh jump và PWM có thể gửi được ngay cả khi không enabled
