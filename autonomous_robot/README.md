# 🤖 Autonomous Robot System

## Hệ Thống Robot Tự Hành Đa Chức Năng với Điều Khiển Gầm Xe Thông Minh

**Phiên bản:** 3.0 | **Cập nhật:** 30/01/2026

---

## 📋 Mục Lục

1. [Giới Thiệu](#-giới-thiệu)
2. [Tính Năng Chính](#-tính-năng-chính)
3. [Phần Cứng Yêu Cầu](#-phần-cứng-yêu-cầu)
4. [Cài Đặt](#-cài-đặt)
5. [Cấu Trúc Dự Án](#-cấu-trúc-dự-án)
6. [Các Chế Độ Hoạt Động](#-các-chế-độ-hoạt-động)
7. [Công Cụ Calibration](#-công-cụ-calibration)
8. [Công Cụ Testing](#-công-cụ-testing)
9. [Giao Thức UART](#-giao-thức-uart)
10. [Hướng Dẫn Sử Dụng](#-hướng-dẫn-sử-dụng)
11. [Cấu Hình](#-cấu-hình)
12. [Troubleshooting](#-troubleshooting)

---

## 🎯 Giới Thiệu

**Autonomous Robot System** là hệ thống điều khiển robot tự hành toàn diện với các khả năng:

| Chức Năng | Mô Tả |
|-----------|-------|
| 🛣️ **Line Following** | Đi theo vạch kẻ đen trên sàn với recovery mode |
| 🏔️ **Terrain Analysis** | Phát hiện trần thấp và vật cản, tự động điều chỉnh gầm xe |
| 🎯 **Object Tracking** | Bám theo vật thể (người, xe) bằng YOLOv8 |
| 🚨 **Patrol Mode** | Tuần tra tự động, phát hiện người xâm nhập |
| 📏 **Depth Sensing** | Đo khoảng cách 3D với Intel RealSense |

### Kiến Trúc Hệ Thống

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         AUTONOMOUS ROBOT SYSTEM                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐    ┌───────────────────────────────────────────┐         │
│  │  RealSense   │    │           PERCEPTION LAYER                │         │
│  │  D435i       │───▶│                                           │         │
│  │  Camera      │    │  ┌─────────────────┐  ┌────────────────┐  │         │
│  │              │    │  │ LineDetector    │  │TerrainAnalyzer │  │         │
│  │ ┌──────────┐ │    │  │ ObjectDetector  │  │DepthEstimator  │  │         │
│  │ │RGB+Depth │ │    │  └────────┬────────┘  └───────┬────────┘  │         │
│  │ └──────────┘ │    └───────────┼───────────────────┼───────────┘         │
│  └──────────────┘                │                   │                     │
│                                  ▼                   ▼                     │
│                      ┌───────────────────────────────────────────┐         │
│                      │         MODE CONTROLLER                    │         │
│                      │  • LineFollowingMode  • PatrolMode        │         │
│                      │  • ObjectTrackingMode • IdleMode          │         │
│                      └─────────────────────┬─────────────────────┘         │
│                                            │                               │
│                                            ▼                               │
│                      ┌───────────────────────────────────────────┐         │
│                      │         UART CONTROLLER                    │         │
│                      │  V(velocity) Y(yaw) H(height) B(buzzer)   │         │
│                      └─────────────────────┬─────────────────────┘         │
│                                            │                               │
│                                            ▼                               │
│                      ┌───────────────────────────────────────────┐         │
│                      │              STM32H7                       │         │
│                      │         Motor + Leg Controller            │         │
│                      └───────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## ✨ Tính Năng Chính

### 1. Line Following với Recovery Mode
- Phát hiện đường line đen bằng **Centerline Method**
- Tự động tìm lại line khi mất (oscillating search)
- ROI hình thang có thể calibrate

### 2. Terrain Analysis với Linear Baseline
- Phát hiện **trần thấp** (ceiling detection)
- Phát hiện **vật cản mặt đất** với linear baseline fit
- Tự động điều chỉnh **chiều cao gầm xe**:
  - 🟢 NORMAL: 10cm
  - 🔵 LOWER: 6cm (trần thấp)
  - 🟡 RAISE: 15cm (có vật cản)
  - 🔴 STOP: Dừng khẩn cấp

### 3. Object Detection với YOLOv8
- Nhận diện 80+ class vật thể
- Multi-point depth sampling cho độ chính xác cao
- Khoảng cách an toàn (D_SAFE) và khẩn cấp (D_EMERGENCY)

### 4. Patrol Mode với Intruder Detection
- Tuần tra tự động theo waypoints
- Phát hiện người xâm nhập
- Theo dõi (tracking) và cảnh báo

### 5. Buzzer Alert System
- B0: Tắt còi
- B1: Beep đơn (trần thấp)
- B2: Alarm liên tục (vật cản nguy hiểm)

---

## 🔧 Phần Cứng Yêu Cầu

| Thiết Bị | Model | Chức Năng | Kết Nối |
|----------|-------|-----------|---------|
| **Camera** | Intel RealSense D435i | RGB + Depth @ 640×480, 30fps | USB 3.0 |
| **MCU** | STM32H7 | Điều khiển motor, chân | UART 115200 |
| **PC/SBC** | Ubuntu 20.04+ / Windows | Xử lý CV/AI | Python 3.8+ |

### Sơ Đồ Kết Nối

```
┌──────────────┐     USB 3.0      ┌──────────────┐
│  RealSense   │─────────────────▶│              │
│  D435i       │                  │   PC/SBC     │
└──────────────┘                  │              │
                                  │  Python 3.8+ │
┌──────────────┐     UART         │  OpenCV      │
│  STM32H7     │◀────────────────▶│  YOLOv8      │
│  Controller  │  /dev/ttyACM0    │              │
└──────────────┘  115200 baud     └──────────────┘
```

---

## 🚀 Cài Đặt

### Bước 1: Clone Repository
```bash
git clone <repository_url>
cd autonomous_robot
```

### Bước 2: Tạo Virtual Environment
```bash
# Linux/Mac
python3 -m venv .venv
source .venv/bin/activate

# Windows
python -m venv .venv
.venv\Scripts\activate
```

### Bước 3: Cài Đặt Dependencies
```bash
pip install -r requirements.txt
```

**Dependencies chính:**
| Package | Version | Chức năng |
|---------|---------|-----------|
| numpy | ≥1.21.0 | Xử lý mảng |
| opencv-python | ≥4.5.0 | Computer Vision |
| pyrealsense2 | ≥2.50.0 | RealSense SDK |
| pyserial | ≥3.5 | UART communication |
| ultralytics | ≥8.0.0 | YOLOv8 |

### Bước 4: Chuẩn Bị Model YOLO
```bash
# Model sẽ tự động download khi chạy lần đầu
# Hoặc copy thủ công:
cp yolov8n.pt data/models/
```

### Bước 5: Cấu Hình UART Permission (Linux)
```bash
sudo usermod -a -G dialout $USER
# Logout và login lại để áp dụng
```

---

## 📁 Cấu Trúc Dự Án

```
autonomous_robot/
│
├── 📄 run_line_follower.py     # Entry point - Line Following
├── 📄 run_patrol.py            # ⭐ Entry point - Tuần tra giám sát
├── 📄 requirements.txt         # Dependencies
│
├── 📁 configs/                 # CẤU HÌNH
│   ├── default_config.yaml     # Cấu hình mặc định
│   └── modes_config.yaml       # Cấu hình các mode
│
├── 📁 src/                     # SOURCE CODE CHÍNH
│   │
│   ├── 📁 core/                # Core modules
│   │   ├── __init__.py
│   │   └── config.py           # Configuration constants
│   │
│   ├── 📁 perception/          # NHẬN THỨC 👁️
│   │   ├── __init__.py
│   │   ├── camera.py               # RealSense interface
│   │   ├── simple_line_detector.py # ⭐ Phát hiện đường line
│   │   ├── terrain_analyzer.py     # ⭐ Phân tích địa hình
│   │   ├── object_detector.py      # ⭐ YOLO detection
│   │   └── depth_estimator.py      # Đo khoảng cách
│   │
│   ├── 📁 modes/               # CHẾ ĐỘ HOẠT ĐỘNG 🎮
│   │   ├── __init__.py
│   │   ├── base_mode.py            # Abstract base class
│   │   ├── line_following_mode.py  # Đi theo line
│   │   ├── object_tracking_mode.py # Bám theo vật
│   │   └── patrol_mode.py          # ⭐ Tuần tra
│   │
│   ├── 📁 control/             # ĐIỀU KHIỂN 🎛️
│   │   ├── __init__.py
│   │   └── motion_controller.py    # Motion commands
│   │
│   └── 📁 communication/       # GIAO TIẾP 📡
│       ├── __init__.py
│       └── uart_controller.py      # ⭐ UART với STM32
│
├── 📁 tools/                   # CÔNG CỤ HỖ TRỢ 🔧
│   │
│   ├── 📁 calibration/         # Calibration tools
│   │   ├── lane_calibration.py          # ⭐ Calib ROI & threshold
│   │   ├── terrain_obstacle_calibration.py  # ⭐ Calib terrain
│   │   ├── depth_calibration.py         # Calib depth camera
│   │   └── camera_calibration.py        # Camera intrinsics
│   │
│   └── 📁 testing/             # Testing tools
│       ├── test_terrain_analyzer.py     # ⭐ Test terrain + UART
│       ├── test_line_detector.py        # Test line detector
│       ├── test_object_detector.py      # Test YOLO
│       ├── test_depth_estimator.py      # Test depth
│       ├── test_uart_simple.py          # Test UART
│       ├── test_camera.py               # Test camera
│       └── test_modules_interactive.py  # Interactive test
│
├── 📁 data/                    # DỮ LIỆU 💾
│   │
│   ├── 📁 calibration/         # Kết quả calibration
│   │   ├── lane_params.json         # ROI & lane params
│   │   ├── terrain_config.json      # ⭐ Terrain config
│   │   ├── depth_calibration.json   # Depth correction
│   │   └── scanline_config.json     # Scanline params
│   │
│   └── 📁 models/              # AI models
│       ├── yolov8n.pt              # YOLOv8 nano
│       └── best.pt                 # Custom trained
│
├── 📁 docs/                    # TÀI LIỆU 📚
│   ├── BAO_CAO_TONG_HOP_CHI_TIET.md   # Báo cáo chi tiết
│   ├── UART_COMMAND_REFERENCE_VI.md   # UART commands
│   └── ...
│
└── 📁 examples/                # VÍ DỤ 📝
    ├── object_tracking_example.py
    └── patrol_example.py
```

---

## 🎮 Các Chế Độ Hoạt Động

### Tổng Quan

| Mode | Phím | Mô Tả | Module |
|------|------|-------|--------|
| **Idle** | `0` / `i` | Dừng, chờ lệnh | - |
| **Line Following** | `1` | Đi theo vạch kẻ | `LineFollowingMode` |
| **Object Tracking** | `2` | Bám theo vật thể | `ObjectTrackingMode` |
| **Patrol** | `3` | Tuần tra tự động | `PatrolMode` |

---

### Mode 1: Line Following

**Mục tiêu:** Robot đi theo vạch kẻ đen trên nền sáng

**Thuật toán:**
```
1. Grayscale → Gaussian Blur
2. Adaptive + Otsu Thresholding
3. ROI Mask (hình thang)
4. Horizontal Slicing (10 slices)
5. Centerline fitting → position_error, heading_error
```

**Recovery Mode:**
```python
# Khi mất line > 3 frames
if frames_lost > 3:
    search_direction = sign(last_known_position)
    oscillate_search(amplitude, direction)
```

**Output:**
- `position_error`: [-1, +1] (trái/phải)
- `heading_error`: radians
- `confidence`: [0, 1]

---

### Mode 2: Object Tracking

**Mục tiêu:** Bám theo vật thể được chọn (người, xe, ...)

**Pipeline:**
```
1. YOLO Detection → List[DetectedObject]
2. Filter by class (person, car, ...)
3. Multi-point depth sampling
4. Calculate steering based on object position
5. Maintain safe distance
```

**Thông số:**
- `track_class`: Class cần track (mặc định: "person")
- `min_confidence`: 0.5
- `safe_distance`: 1.5m
- `tracking_gain`: 1.2

---

### Mode 3: Patrol

**Mục tiêu:** Tuần tra và phát hiện người xâm nhập

**State Machine:**
```
┌──────────────┐     timeout      ┌──────────────┐
│  PATROLLING  │────────────────▶│   ROTATING   │
│  (đi thẳng)  │◀────────────────│   (quay)     │
└──────┬───────┘                 └──────────────┘
       │
       │ detect person
       ▼
┌──────────────┐     timeout      ┌──────────────┐
│    ALERT     │────────────────▶│   TRACKING   │
│  🚨 CẢNH BÁO │                 │  (theo dõi)  │
└──────────────┘                 └──────────────┘
```

**Cấu hình:**
```python
PatrolConfig(
    patrol_velocity=0.3,       # m/s
    patrol_forward_time=5.0,   # seconds
    patrol_rotate_time=3.0,    # seconds
    alert_distance=3.0,        # meters
    track_intruder=True        # Bật theo dõi
)
```

---

### Terrain Analysis (Tích hợp với Line Following)

**Chức năng:** Phát hiện trần thấp và vật cản, điều chỉnh gầm

**Actions:**

| Action | Height | Buzzer | Điều kiện |
|--------|--------|--------|-----------|
| 🟢 NORMAL | 10cm | OFF | Đường thông thoáng |
| 🔵 LOWER | 6cm | BEEP | Trần thấp |
| 🟡 RAISE | 15cm | OFF | Vật cản ≤ 5cm |
| 🔴 STOP | - | ALARM | Vật cản > 5cm |

**Linear Baseline Algorithm:**
```python
# Fit baseline tuyến tính theo y
row_baseline[i] = median(valid_depths_in_row_i)
fit = np.polyfit(y_indices, baseline_values, degree=1)

# Phát hiện vật cản
obstacle_mask = depth < (baseline_line - threshold)
```

---

## 🔧 Công Cụ Calibration

### 1. Lane Calibration

**File:** `tools/calibration/lane_calibration.py`

```bash
python tools/calibration/lane_calibration.py
```

**Trackbars:**
| Parameter | Range | Mô tả |
|-----------|-------|-------|
| ROI Top Y | 0-100% | Vị trí Y đỉnh ROI |
| ROI Bot Y | 0-100% | Vị trí Y đáy ROI |
| Top Left/Right X | 0-100% | Góc trên |
| Bottom Left/Right X | 0-100% | Góc dưới |
| Threshold | 0-255 | Ngưỡng đen trắng |
| Morph Kernel | 1-15 | Kernel morphology |

**Controls:**
| Key | Action |
|-----|--------|
| S | Save parameters → `lane_params.json` |
| C | Capture image |
| R | Reset to defaults |
| Q | Quit |

---

### 2. Terrain Obstacle Calibration

**File:** `tools/calibration/terrain_obstacle_calibration.py`

```bash
python tools/calibration/terrain_obstacle_calibration.py
```

**Hiển thị 4 đồ thị:**
1. **Depth Heatmap** - Ảnh depth với ROI overlay
2. **Baseline Fit** - Linear fit của depth theo y
3. **Obstacle Mask** - Binary mask vật cản
4. **Depth Profile** - Profile depth theo cột được chọn

**Controls:**
| Key | Action |
|-----|--------|
| S | Save config → `terrain_config.json` |
| L | Load config |
| R | Reset to defaults |
| P | Pause/Resume |
| Mouse | Select column for profile |
| Q | Quit |

---

### 3. Depth Calibration

**File:** `tools/calibration/depth_calibration.py`

```bash
python tools/calibration/depth_calibration.py
```

**Chức năng:**
- Click để đo khoảng cách tại điểm
- So sánh với khoảng cách thực
- Tính correction factor

---

## 🧪 Công Cụ Testing

### 1. Test Terrain Analyzer (⭐ Recommended)

**File:** `tools/testing/test_terrain_analyzer.py`

```bash
# Với UART hardware
python tools/testing/test_terrain_analyzer.py

# Với mock UART
python tools/testing/test_terrain_analyzer.py --mock-uart

# Không UART
python tools/testing/test_terrain_analyzer.py --no-uart
```

**Features:**
- Test terrain detection
- Test UART buzzer (B0/B1/B2)
- Test height control (H60/H100/H150)
- Live visualization

**Controls:**
| Key | Action |
|-----|--------|
| Q | Quit |
| R | Reset analyzer |
| S | Save frame |
| B | Test beep |
| C | Save calibration |

---

### 2. Test Line Detector

```bash
python tools/testing/test_line_detector.py
```

---

### 3. Test Object Detector

```bash
python tools/testing/test_object_detector.py
```

**Controls:**
| Key | Action |
|-----|--------|
| + | Tăng confidence threshold |
| - | Giảm confidence threshold |

---

### 4. Test UART Simple

```bash
python tools/testing/test_uart_simple.py
```

---

### 5. Interactive Module Test

```bash
python tools/testing/test_modules_interactive.py
```

**Controls:**
| Key | Action |
|-----|--------|
| 1 | Chỉ Line Detection |
| 2 | Chỉ Object Detection |
| 3 | Chỉ Depth Estimation |
| 4 | Tất cả modules |
| Space | Pause/Resume |

---

## 📡 Giao Thức UART

### Thông Số Kết Nối

| Parameter | Value |
|-----------|-------|
| Port | `/dev/ttyACM0` (Linux) / `COMx` (Windows) |
| Baudrate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |

### Command Protocol

```
Format: <COMMAND><VALUE>\n
```

| Command | Example | Description |
|---------|---------|-------------|
| **E1** | `E1\n` | Enable motor control |
| **E0** | `E0\n` | Disable motor control |
| **V{x}** | `V100\n` | Velocity: 0.1 m/s (value × 0.001) |
| **Y{x}** | `Y300\n` | Yaw rate: 0.3 rad/s (value × 0.001) |
| **H{x}** | `H100\n` | Leg height: 10cm (value × 0.001 m) |
| **B0** | `B0\n` | Buzzer OFF |
| **B1** | `B1\n` | Buzzer single BEEP |
| **B2** | `B2\n` | Buzzer continuous ALARM |

### Terrain Action → UART Mapping

| Action | Height | Buzzer | Commands |
|--------|--------|--------|----------|
| NORMAL | 10cm | OFF | `E1`, `H100`, `B0` |
| LOWER | 6cm | BEEP | `E1`, `H60`, `B1` |
| RAISE | 15cm | OFF | `E1`, `H150`, `B0` |
| STOP | - | ALARM | `E1`, `V0`, `Y0`, `B2` |

---

## 📖 Hướng Dẫn Sử Dụng

### Quick Start

```bash
# 1. Activate environment
source .venv/bin/activate

# 2. Calibrate (chỉ cần lần đầu)
python tools/calibration/lane_calibration.py
python tools/calibration/terrain_obstacle_calibration.py

# 3. Test (tùy chọn)
python tools/testing/test_terrain_analyzer.py --mock-uart

# 4. Run Line Following
python run_line_follower.py

# 5. Run Patrol Surveillance
python run_patrol.py
```

### Chạy Line Following

```bash
python run_line_follower.py [OPTIONS]

Options:
  --mock-uart       Sử dụng mock UART (không cần hardware)
  --no-viz          Không hiển thị GUI
  --debug           Enable debug logging
```

### Chạy Patrol Surveillance

```bash
python run_patrol.py [OPTIONS]

Options:
  --mock-uart       Sử dụng mock UART (không cần hardware)
  --no-uart         Không sử dụng UART (chỉ camera)
  --no-viz          Không hiển thị GUI
  --debug           Enable debug logging
  --port PORT       UART port (default: /dev/ttyACM0)
```

### Keyboard Controls (khi chạy)

| Key | Action |
|-----|--------|
| Q | Quit |
| R | Reset |
| 0/I | Idle mode |
| 1 | Line Following mode |
| 2 | Object Tracking mode |
| 3 | Patrol mode |
| Space | Pause/Resume |

---

## ⚙️ Cấu Hình

### File: `data/calibration/terrain_config.json`

```json
{
  "robot_dimensions": {
    "robot_height": 0.3,
    "min_ground_clearance": 0.07,
    "max_ground_clearance": 0.18,
    "normal_ground_clearance": 0.10
  },
  "camera_setup": {
    "camera_height": 0.2,
    "camera_tilt_angle": 14,
    "camera_vfov": 58.0
  },
  "detection_zones": {
    "ceiling_zone_top": 0.0,
    "ceiling_zone_bottom": 0.3,
    "ground_zone_top": 0.6,
    "ground_zone_bottom": 1.0
  },
  "ground_obstacle_detection": {
    "obstacle_threshold": 0.06,
    "max_step_height": 0.05
  }
}
```

### File: `data/calibration/lane_params.json`

```json
{
  "roi": {
    "top_y": 0.65,
    "bottom_y": 1.0,
    "top_left_x": 0.33,
    "top_right_x": 0.7,
    "bottom_left_x": 0.25,
    "bottom_right_x": 0.8
  },
  "lane_detection": {
    "black_threshold": 100,
    "morph_kernel_size": 3
  }
}
```

### File: `src/core/config.py` (Constants)

```python
# Camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# YOLO
YOLO_MODEL_PATH = "data/models/yolov8n.pt"
YOLO_CONFIDENCE_THRESHOLD = 0.5
YOLO_NMS_THRESHOLD = 0.45

# Safety distances
D_SAFE = 1.5          # meters
D_EMERGENCY = 0.5     # meters

# Depth processing
DEPTH_MIN_VALID = 0.1   # meters
DEPTH_MAX_VALID = 10.0  # meters
```

---

## 🐛 Troubleshooting

### Camera không nhận

```bash
# Kiểm tra USB
lsusb | grep Intel

# Kiểm tra RealSense
realsense-viewer

# Reset udev
sudo systemctl restart udev

# Permissions
sudo chmod 666 /dev/video*
sudo usermod -a -G video $USER
```

### UART không kết nối

```bash
# Kiểm tra port
ls /dev/ttyACM* /dev/ttyUSB*

# Permissions
sudo usermod -a -G dialout $USER
# Logout và login lại

# Test với minicom
minicom -D /dev/ttyACM0 -b 115200
```

### YOLO model không load

```bash
# Cài lại ultralytics
pip install --upgrade ultralytics

# Download model
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

### Line detection không ổn định

1. Chạy `lane_calibration.py` để điều chỉnh ROI
2. Kiểm tra ánh sáng (tránh bóng, phản chiếu)
3. Tăng `morph_kernel_size` để giảm noise
4. Điều chỉnh `black_threshold`

### Terrain analyzer false positive

1. Chạy `terrain_obstacle_calibration.py`
2. Tăng `obstacle_threshold` (ví dụ: 0.06 → 0.08)
3. Điều chỉnh `ground_zone_top` (tăng để bỏ vùng xa)
4. Kiểm tra camera tilt angle

### Performance chậm

1. Giảm resolution: 640×480 → 320×240
2. Giảm FPS: 30 → 15
3. Sử dụng YOLOv8n (nano) thay vì model lớn hơn
4. Disable visualization: `--no-viz`

---

## 📄 Tài Liệu Liên Quan

- [BAO_CAO_TONG_HOP_CHI_TIET.md](docs/BAO_CAO_TONG_HOP_CHI_TIET.md) - Báo cáo chi tiết đầy đủ
- [UART_COMMAND_REFERENCE_VI.md](docs/UART_COMMAND_REFERENCE_VI.md) - Tham chiếu lệnh UART
- [UART_TESTING_GUIDE.md](tools/testing/UART_TESTING_GUIDE.md) - Hướng dẫn test UART

---

## 📜 License

MIT License

---

## 👥 Contributors

Autonomous Robot Project Team

---

**© 2026 Autonomous Robot Project**
