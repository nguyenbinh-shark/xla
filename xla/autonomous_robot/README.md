# Autonomous Robot System

Hệ thống robot tự hành với nhiều chế độ hoạt động: theo dõi vạch kẻ, bám theo vật thể, tuần tra phát hiện người lạ.

## 🎮 Các Chế Độ Hoạt Động

| Chế độ | Mô tả | Phím tắt |
|--------|-------|----------|
| **Line Following** | Đi theo vạch kẻ trên sàn | `1` |
| **Object Tracking** | Bám theo vật thể (người, xe...) | `2` |
| **Patrol** | Tuần tra, phát hiện người lạ | `3` |
| **Idle** | Dừng, chờ lệnh | `0` hoặc `i` |

## 📁 Cấu Trúc Dự Án

```
autonomous_robot/
├── run_robot.py              # Entry point chính - chạy robot
├── run_line_follower.py      # Chạy riêng line following
├── requirements.txt          # Dependencies
│
├── configs/                  # Cấu hình
│   ├── default_config.yaml   # Camera, detection params
│   └── modes_config.yaml     # Mode-specific params
│
├── src/                      # Source code chính
│   ├── core/                 # Core modules
│   │   └── config.py         # Configuration management
│   │
│   ├── perception/           # Perception modules
│   │   ├── camera.py         # RealSense camera
│   │   ├── simple_line_detector.py  # Line detection
│   │   ├── object_detector.py       # YOLO detection
│   │   └── depth_estimator.py       # Depth measurement
│   │
│   ├── modes/                # Operation modes ⭐
│   │   ├── base_mode.py              # Abstract base
│   │   ├── line_following_mode.py    # Line following
│   │   └── object_tracking_mode.py   # Object tracking
│   │
│   ├── control/              # Control modules
│   │   ├── robot_controller.py  # Main controller
│   │   └── motion_controller.py # Low-level motion
│   │
│   └── communication/        # Communication
│       └── uart_controller.py   # STM32 UART + Feedback
│
├── tools/                    # Công cụ hỗ trợ
│   ├── calibration/          # Calibration tools
│   └── testing/              # Testing tools
│
└── data/                     # Data files
    ├── models/               # AI models (YOLO)
    └── calibration/          # Calibration data
```

## 🚀 Cài Đặt

### 1. Tạo môi trường ảo
```bash
python -m venv .venv
source .venv/bin/activate  # Linux/Mac
# hoặc
.venv\Scripts\activate     # Windows
```

### 2. Cài đặt dependencies
```bash
pip install -r requirements.txt
```

### 3. Copy model YOLO (nếu cần)
```bash
cp yolov8n.pt data/models/
```

## 🎮 Sử Dụng

### Chạy Robot
```bash
# Chạy với UART thật
python run_robot.py

# Chạy với mock UART (test không cần hardware)
python run_robot.py --mock-uart

# Chạy không hiển thị GUI
python run_robot.py --no-viz

# Chạy với debug logging
python run_robot.py --debug

# Xem cấu hình hiện tại
python run_robot.py --print-config

# Sử dụng file config custom
python run_robot.py --config configs/my_config.yaml
```

### Điều khiển khi chạy
- `q` - Thoát
- `r` - Reset hệ thống

## 🔧 Công Cụ Calibration

### Calibrate Lane Detection
```bash
python -m tools.calibration.lane_calibration
```
**Điều khiển:**
- Trackbars để điều chỉnh ROI và parameters
- `s` - Lưu parameters
- `c` - Chụp ảnh
- `r` - Reset về mặc định
- `q` - Thoát

### Calibrate Depth Camera
```bash
python -m tools.calibration.depth_calibration
```
**Điều khiển:**
- Click chuột để đo khoảng cách tại điểm
- `g` - Hiển thị lưới đo
- `d` - Hiển thị depth colormap
- `p` - In tất cả measurements
- `c` - Xóa measurements
- `q` - Thoát

## 🧪 Testing

### Interactive Module Test
```bash
python -m tools.testing.test_modules_interactive
```
**Điều khiển:**
- `1` - Chỉ Lane Detection
- `2` - Chỉ Object Detection
- `3` - Chỉ Depth Estimation
- `4` - Tất cả modules
- `Space` - Pause/Resume
- `s` - Lưu frame
- `q` - Thoát

### Unit Tests
```bash
# Chạy tất cả tests
pytest tests/ -v

# Chạy test cụ thể
pytest tests/test_unit.py::TestPIDController -v
```

## 📋 Demo Scripts

### Object Detection với Depth
```bash
python scripts/demo_detection_depth.py
```
**Điều khiển:**
- `d` - Toggle depth map
- `s` - Lưu frame
- `q` - Thoát

## ⚙️ Cấu Hình

### Sử dụng YAML config
Copy `configs/default_config.yaml` và chỉnh sửa:
```bash
cp configs/default_config.yaml configs/my_config.yaml
```

### Các tham số quan trọng

#### Camera
```yaml
camera:
  width: 640
  height: 480
  fps: 30
```

#### ROI (Vùng phát hiện làn đường)
```yaml
roi:
  top_left_x: 0.33      # Góc trên trái X (0-1)
  top_right_x: 0.7      # Góc trên phải X
  bottom_left_x: 0.25   # Góc dưới trái X
  bottom_right_x: 0.8   # Góc dưới phải X
  top_y: 0.65           # Y trên (0-1)
  bottom_y: 1.0         # Y dưới
```

#### Safety (An toàn)
```yaml
obstacle:
  d_safe: 2.0           # Khoảng cách an toàn (m)
  d_emergency: 0.5      # Khoảng cách dừng khẩn cấp (m)
```

#### Motion Control
```yaml
motion_control:
  pid:
    kp: 0.005           # Proportional gain
    ki: 0.0001          # Integral gain
    kd: 0.002           # Derivative gain
  speed:
    max: 0.8            # Tốc độ tối đa (m/s)
    normal: 0.6         # Tốc độ bình thường
    slow: 0.3           # Tốc độ chậm
```

## 🔌 Giao Tiếp UART

### Protocol với STM32
```
E1          - Enable control
E0          - Disable control
V{xxx}      - Velocity (m/s × 1000)
Y{xxx}      - Yaw rate (rad/s × 1000)
H{xxx}      - Leg height (m × 1000)
```

### Cấu hình
```yaml
uart:
  port: "/dev/ttyUSB0"
  baudrate: 115200
```

## 🏗️ Kiến Trúc Hệ Thống

```
┌─────────────────────────────────────────────────────────────┐
│                      Main Loop (30Hz)                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │ RealSense│───▶│ Perception   │───▶│ Control          │  │
│  │ Camera   │    │ - Lane Det.  │    │ - State Machine  │  │
│  │          │    │ - Object Det.│    │ - Motion Control │  │
│  │          │    │ - Depth Est. │    │ - PID            │  │
│  └──────────┘    └──────────────┘    └────────┬─────────┘  │
│                                                │            │
│                                       ┌────────▼─────────┐  │
│                                       │ UART Controller  │  │
│                                       │ (STM32)          │  │
│                                       └──────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## 📝 Lưu Ý Quan Trọng

1. **Calibration trước khi chạy**: Luôn calibrate ROI và lane parameters trước
2. **Test từng module**: Sử dụng interactive tester để kiểm tra từng module
3. **Safety first**: Đảm bảo khoảng cách an toàn được cấu hình đúng
4. **Backup config**: Giữ backup của file cấu hình đã calibrate

## 🐛 Troubleshooting

### Camera không khởi động
```bash
# Kiểm tra RealSense được nhận
realsense-viewer

# Kiểm tra permissions
sudo chmod 666 /dev/video*
```

### UART không kết nối
```bash
# Kiểm tra port
ls /dev/ttyUSB*

# Thêm user vào group dialout
sudo usermod -a -G dialout $USER
```

### YOLO model không load
```bash
# Download model
pip install ultralytics
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

## 📄 License

MIT License
