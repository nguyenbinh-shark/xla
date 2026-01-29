# BÁO CÁO TỔNG HỢP CHI TIẾT DỰ ÁN AUTONOMOUS ROBOT
## Hệ Thống Robot Tự Hành Theo Line Với Điều Khiển Gầm Xe

**Phiên bản:** 3.0 - Final  
**Ngày hoàn thành:** 30/01/2026  
**Tác giả:** Autonomous Robot Project Team

---

# MỤC LỤC

1. [GIỚI THIỆU TỔNG QUAN](#1-giới-thiệu-tổng-quan)
2. [KIẾN TRÚC HỆ THỐNG](#2-kiến-trúc-hệ-thống)
3. [MODULE CAMERA VÀ DEPTH SENSING](#3-module-camera-và-depth-sensing)
4. [MODULE PHÁT HIỆN ĐƯỜNG LINE](#4-module-phát-hiện-đường-line)
5. [MODULE PHÂN TÍCH ĐỊA HÌNH](#5-module-phân-tích-địa-hình)
6. [PHÁT HIỆN VẬT THỂ BẰNG AI VÀ ĐO KHOẢNG CÁCH](#6-phát-hiện-vật-thể-bằng-ai-và-đo-khoảng-cách)
7. [GIAO TIẾP UART VỚI STM32](#7-giao-tiếp-uart-với-stm32)
8. [CÔNG CỤ CALIBRATION](#8-công-cụ-calibration)
9. [CÔNG CỤ TESTING](#9-công-cụ-testing)
10. [HƯỚNG DẪN SỬ DỤNG](#10-hướng-dẫn-sử-dụng)
11. [PHỤ LỤC KỸ THUẬT](#11-phụ-lục-kỹ-thuật)

---

# 1. GIỚI THIỆU TỔNG QUAN

## 1.1 Mô Tả Dự Án

Dự án **Autonomous Robot** xây dựng hệ thống robot tự hành có khả năng:

| Chức năng | Mô tả | Module |
|-----------|-------|--------|
| 🛣️ **Theo dõi đường line** | Robot đi theo vạch kẻ đen trên sàn | `SimpleLineDetector` |
| 🏔️ **Phân tích địa hình** | Phát hiện trần thấp, vật cản trên mặt đất | `TerrainAnalyzer` |
| 🔧 **Điều chỉnh gầm xe** | Tự động nâng/hạ gầm khi gặp chướng ngại | `HeightController` |
| 🔊 **Cảnh báo còi** | Kêu còi khi phát hiện trần thấp hoặc vật cản | `BuzzerController` |

## 1.2 Phần Cứng Sử Dụng

| Thiết bị | Model | Chức năng | Thông số |
|----------|-------|-----------|----------|
| **Camera** | Intel RealSense D435i | Thu ảnh RGB + Depth | 640×480 @ 30fps |
| **MCU** | STM32H7 | Điều khiển motor, chân | UART 115200 baud |
| **PC** | Linux/Windows | Xử lý CV/AI | Python 3.8+ |

## 1.3 Sơ Đồ Hệ Thống

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         AUTONOMOUS ROBOT SYSTEM                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐    ┌───────────────────────────────────────────┐         │
│  │  RealSense   │    │           PERCEPTION LAYER                │         │
│  │  D435i       │───▶│                                           │         │
│  │  Camera      │    │  ┌─────────────────┐  ┌────────────────┐  │         │
│  │              │    │  │ SimpleLineDetector│  │TerrainAnalyzer │  │         │
│  │ ┌──────────┐ │    │  │                 │  │                │  │         │
│  │ │RGB+Depth │ │    │  │ • Position Error│  │ • Ceiling Detect│  │         │
│  │ └──────────┘ │    │  │ • Heading Error │  │ • Obstacle Detect│  │         │
│  └──────────────┘    │  │ • Recovery Mode │  │ • Height Recommend│ │         │
│                      │  └────────┬────────┘  └───────┬────────┘  │         │
│                      └───────────┼───────────────────┼───────────┘         │
│                                  │                   │                     │
│                                  ▼                   ▼                     │
│                      ┌───────────────────────────────────────────┐         │
│                      │           CONTROL LAYER                    │         │
│                      │                                           │         │
│                      │  velocity = f(position_error, heading)    │         │
│                      │  yaw_rate = STEERING_GAIN × pos_error     │         │
│                      │  height = NORMAL / RAISE / LOWER          │         │
│                      │                                           │         │
│                      └─────────────────────┬─────────────────────┘         │
│                                            │                               │
│                                            ▼                               │
│                      ┌───────────────────────────────────────────┐         │
│                      │         UART CONTROLLER                    │         │
│                      │                                           │         │
│                      │  E1      → Enable motor control           │         │
│                      │  V{xxx}  → Velocity (m/s × 1000)          │         │
│                      │  Y{xxx}  → Yaw rate (rad/s × 1000)        │         │
│                      │  H{xxx}  → Leg height (m × 1000)          │         │
│                      │  B0/B1/B2 → Buzzer off/beep/alarm         │         │
│                      │                                           │         │
│                      └─────────────────────┬─────────────────────┘         │
│                                            │                               │
│                                            ▼                               │
│                      ┌───────────────────────────────────────────┐         │
│                      │              STM32H7                       │         │
│                      │         Motor + Leg Controller            │         │
│                      └───────────────────────────────────────────┘         │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

# 2. KIẾN TRÚC HỆ THỐNG

## 2.1 Cấu Trúc Thư Mục

```
autonomous_robot/
│
├── 📄 run_line_follower.py     # Entry point chính
├── 📄 requirements.txt         # Dependencies
│
├── 📁 src/                     # SOURCE CODE CHÍNH
│   ├── 📁 perception/          # Nhận thức
│   │   ├── camera.py           # RealSense camera interface
│   │   ├── simple_line_detector.py  # ⭐ Phát hiện đường line
│   │   ├── terrain_analyzer.py      # ⭐ Phân tích địa hình
│   │   ├── object_detector.py       # YOLO object detection
│   │   └── depth_estimator.py       # Đo khoảng cách
│   │
│   ├── 📁 control/             # Điều khiển
│   │   └── motion_controller.py     # Tạo lệnh di chuyển
│   │
│   ├── 📁 communication/       # Giao tiếp
│   │   └── uart_controller.py       # ⭐ UART với STM32
│   │
│   └── 📁 core/                # Core utilities
│       └── config.py           # Cấu hình hệ thống
│
├── 📁 tools/                   # CÔNG CỤ HỖ TRỢ
│   ├── 📁 calibration/         # Calibration tools
│   │   ├── lane_calibration.py          # ⭐ Calib ROI, threshold
│   │   ├── terrain_obstacle_calibration.py  # ⭐ Calib terrain
│   │   └── depth_calibration.py         # Calib depth
│   │
│   └── 📁 testing/             # Testing tools
│       ├── test_terrain_analyzer.py     # ⭐ Test terrain + UART
│       ├── test_line_detector.py        # Test line detector
│       └── test_uart_simple.py          # Test UART đơn giản
│
├── 📁 data/                    # DỮ LIỆU
│   ├── 📁 calibration/         # Kết quả calibration
│   │   ├── lane_params.json         # Thông số lane
│   │   └── terrain_config.json      # ⭐ Thông số terrain
│   │
│   └── 📁 models/              # AI models
│       └── yolov8n.pt
│
└── 📁 docs/                    # TÀI LIỆU
    ├── BAO_CAO_TONG_HOP_CHI_TIET.md   # ← File này
    ├── TERRAIN_OBSTACLE_CALIBRATION_GUIDE.md
    └── UART_COMMAND_REFERENCE_VI.md
```

## 2.2 Pipeline Xử Lý Chính

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         MAIN CONTROL LOOP @ 30Hz                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  while running:                                                         │
│      │                                                                  │
│      ▼                                                                  │
│  ┌──────────────┐                                                      │
│  │ Get Frames   │◀──── camera.get_frames()                             │
│  │ RGB + Depth  │                                                      │
│  └──────┬───────┘                                                      │
│         │                                                               │
│         ├─────────────────────────────────────────────┐                │
│         ▼                                             ▼                │
│  ┌──────────────────┐                    ┌────────────────────┐        │
│  │ LINE DETECTION   │                    │ TERRAIN ANALYSIS   │        │
│  │                  │                    │ (mỗi 3 frames)     │        │
│  │ • Grayscale      │                    │                    │        │
│  │ • Threshold      │                    │ • Ceiling zone     │        │
│  │ • ROI mask       │                    │ • Ground zone      │        │
│  │ • Centerline     │                    │ • Baseline fit     │        │
│  │ • Error calc     │                    │ • Obstacle detect  │        │
│  └────────┬─────────┘                    └──────────┬─────────┘        │
│           │                                         │                  │
│           │ position_error                          │ action           │
│           │ heading_error                           │ (NORMAL/RAISE/   │
│           │                                         │  LOWER/STOP)     │
│           ▼                                         ▼                  │
│  ┌─────────────────────────────────────────────────────────────┐      │
│  │                 CONTROL CALCULATION                          │      │
│  │                                                              │      │
│  │  velocity = BASE_SPEED × (1 - |pos_error|)                  │      │
│  │  yaw_rate = STEERING_GAIN × pos_error + HEADING_GAIN × heading     │
│  │                                                              │      │
│  │  if terrain.action == RAISE:                                │      │
│  │      height = 0.15m (15cm)                                  │      │
│  │  elif terrain.action == LOWER:                              │      │
│  │      height = 0.06m (6cm)                                   │      │
│  │  else:                                                      │      │
│  │      height = 0.10m (10cm)                                  │      │
│  │                                                              │      │
│  └──────────────────────────┬──────────────────────────────────┘      │
│                             │                                          │
│                             ▼                                          │
│  ┌─────────────────────────────────────────────────────────────┐      │
│  │                   UART SEND                                  │      │
│  │                                                              │      │
│  │  "E1"      → Enable control                                 │      │
│  │  "V100"    → 0.1 m/s forward                                │      │
│  │  "Y300"    → 0.3 rad/s turn                                 │      │
│  │  "H100"    → 10cm leg height                                │      │
│  │  "B1"      → Beep (trần thấp)                               │      │
│  │  "B2"      → Alarm (vật cản)                                │      │
│  │                                                              │      │
│  └──────────────────────────────────────────────────────────────┘      │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

# 3. MODULE CAMERA VÀ DEPTH SENSING

## 3.1 Intel RealSense D435i

**File:** `src/perception/camera.py`

### Thông Số Camera

| Thông số | Giá trị |
|----------|---------|
| Resolution | 640 × 480 |
| Frame rate | 30 FPS |
| Horizontal FOV | 87° |
| Vertical FOV | 58° |
| Depth range | 0.1m - 10m |
| Baseline (stereo) | ~50mm |

### Nguyên Lý Stereo Depth

```
         Vật thể P
            ●
           /|\
          / | \
         /  |  \
        /   |   \
  IR Left   |   IR Right
    ◯ ─────┼───── ◯
      ←──B──→
    
    depth = (f × B) / disparity
    
    f = focal length
    B = baseline (50mm)
    disparity = sự chênh lệch pixel
```

### Góc Nhìn Theo Vùng Frame

```
                    Camera (nghiêng 15° xuống)
                         ║
  ──────────────────────╨──────────────────────
                     MẶT ĐẤT

Frame 640×480:
┌────────────────────────────────────────────┐ 0%
│         CEILING ZONE (0-30%)               │ ← Nhìn lên 14°
│         Phát hiện trần thấp                │
├────────────────────────────────────────────┤ 30%
│                                            │
│         (Vùng giữa - bỏ qua)              │
│                                            │
├────────────────────────────────────────────┤ 60%
│         GROUND ZONE (60-100%)              │ ← Nhìn xuống 30-44°
│         Phát hiện vật cản mặt đất          │
└────────────────────────────────────────────┘ 100%
```

---

# 4. MODULE PHÁT HIỆN ĐƯỜNG LINE

## 4.1 Tổng Quan

**File:** `src/perception/simple_line_detector.py`

**Mục tiêu:** Phát hiện đường line đen trên nền sáng, tính toán:
- `position_error`: Robot lệch trái/phải bao nhiêu ([-1, +1])
- `heading_error`: Robot hướng chệch bao nhiêu độ (radians)

## 4.2 Pipeline Chi Tiết

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       LINE DETECTION PIPELINE                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  INPUT: Color Frame (640×480, BGR)                                      │
│         │                                                               │
│         ▼                                                               │
│  ┌──────────────────────────────────────────────────────┐              │
│  │ STEP 1: PREPROCESSING                                 │              │
│  │                                                       │              │
│  │ gray = cv2.cvtColor(frame, COLOR_BGR2GRAY)           │              │
│  │ gray = cv2.GaussianBlur(gray, (5,5), 0)              │              │
│  │                                                       │              │
│  │ # Adaptive + Otsu thresholding (kết hợp 2 phương pháp)│              │
│  │ binary_adaptive = cv2.adaptiveThreshold(...)          │              │
│  │ binary_otsu = cv2.threshold(... OTSU)                │              │
│  │ binary = bitwise_and(adaptive, otsu)                 │              │
│  │                                                       │              │
│  │ # Morphological operations                            │              │
│  │ binary = cv2.morphologyEx(binary, MORPH_CLOSE)       │              │
│  │ binary = cv2.morphologyEx(binary, MORPH_OPEN)        │              │
│  └──────────────────────────────────────────────────────┘              │
│         │                                                               │
│         ▼                                                               │
│  ┌──────────────────────────────────────────────────────┐              │
│  │ STEP 2: ROI MASK (hình thang)                         │              │
│  │                                                       │              │
│  │              ╱──────────╲        ← Top (33%-70%)     │              │
│  │             ╱            ╲                            │              │
│  │            ╱   ROI ZONE   ╲                           │              │
│  │           ╱                ╲                          │              │
│  │          ╱──────────────────╲   ← Bottom (25%-80%)   │              │
│  │                                                       │              │
│  │ binary = bitwise_and(binary, roi_mask)               │              │
│  └──────────────────────────────────────────────────────┘              │
│         │                                                               │
│         ▼                                                               │
│  ┌──────────────────────────────────────────────────────┐              │
│  │ STEP 3: HORIZONTAL SLICING (10 slices)                │              │
│  │                                                       │              │
│  │   Slice 0: ─────────█████─────────  centroid_x = 320 │              │
│  │   Slice 1: ────────███████────────  centroid_x = 315 │              │
│  │   Slice 2: ───────█████████───────  centroid_x = 310 │              │
│  │   ...                                                 │              │
│  │   Slice 9: ─────████████████──────  centroid_x = 290 │              │
│  │                                                       │              │
│  │   centerline_points = [(x0,y0), (x1,y1), ...]        │              │
│  └──────────────────────────────────────────────────────┘              │
│         │                                                               │
│         ▼                                                               │
│  ┌──────────────────────────────────────────────────────┐              │
│  │ STEP 4: ERROR CALCULATION                             │              │
│  │                                                       │              │
│  │ # Position Error                                      │              │
│  │ image_center_x = width/2 + CAMERA_OFFSET_X           │              │
│  │ pos_error = (line_x - image_center_x) / (width/2)    │              │
│  │                                                       │              │
│  │ # Heading Error                                       │              │
│  │ coeffs = np.polyfit(y_vals, x_vals, 1)               │              │
│  │ slope = coeffs[0]  # dx/dy                           │              │
│  │ heading_error = arctan(slope)                        │              │
│  │                                                       │              │
│  │ # Confidence                                          │              │
│  │ confidence = len(points) / num_slices                │              │
│  └──────────────────────────────────────────────────────┘              │
│         │                                                               │
│         ▼                                                               │
│  OUTPUT: LineDetectionResult                                            │
│          - line_detected: True/False                                    │
│          - position_error: -1 to +1                                     │
│          - heading_error: radians                                       │
│          - confidence: 0 to 1                                           │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 4.3 Chế Độ Recovery (Tìm Lại Line)

Khi mất line, robot tự động vào chế độ tìm kiếm:

```python
# Xác định hướng tìm dựa trên vị trí cuối cùng
if last_known_position > 0.1:
    search_direction = +1  # Line ở bên phải, quay phải
elif last_known_position < -0.1:
    search_direction = -1  # Line ở bên trái, quay trái

# Dao động tìm kiếm với biên độ tăng dần
phase = (frames_lost - 3) % 20
if phase < 10:
    search_error = amplitude * search_direction
else:
    search_error = -amplitude * search_direction
```

## 4.4 Thông Số Calibration (lane_params.json)

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
    "morph_kernel_size": 3,
    "canny_low": 89,
    "canny_high": 108,
    "hough_threshold": 31,
    "hough_min_line_length": 153,
    "hough_max_line_gap": 20
  }
}
```

---

# 5. MODULE PHÂN TÍCH ĐỊA HÌNH

## 5.1 Tổng Quan

**File:** `src/perception/terrain_analyzer.py`

**Mục tiêu:**
- Phát hiện trần thấp (gầm cầu, cửa, ...)
- Phát hiện vật cản trên mặt đất
- Đề xuất hành động: **NORMAL**, **LOWER**, **RAISE**, **STOP**

## 5.2 Cấu Hình (TerrainConfig)

```python
@dataclass 
class TerrainConfig:
    # Robot dimensions
    robot_height: float = 0.3              # 30cm
    min_ground_clearance: float = 0.07     # 7cm - hạ gầm
    max_ground_clearance: float = 0.18     # 18cm - nâng gầm
    normal_ground_clearance: float = 0.10  # 10cm - bình thường
    raised_ground_clearance: float = 0.15  # 15cm - khi có vật cản
    
    # Camera mounting
    camera_height: float = 0.20            # 20cm so với mặt đất
    camera_tilt_angle: float = 14.0        # Nghiêng xuống 14°
    camera_vfov: float = 58.0              # Vertical FOV
    
    # Detection zones
    ceiling_zone_top: float = 0.0          # 0% frame
    ceiling_zone_bottom: float = 0.30      # 30% frame
    ground_zone_top: float = 0.60          # 60% frame
    ground_zone_bottom: float = 1.0        # 100% frame
    
    # Thresholds
    ceiling_min_clearance: float = 0.5     # Tối thiểu 50cm không gian trên đầu
    ceiling_warning_distance: float = 1.5  # Cảnh báo khi trần < 1.5m
    obstacle_threshold: float = 0.03       # 3cm chênh lệch = vật cản
    max_step_height: float = 0.05          # Bước qua được vật 5cm
```

## 5.3 Thuật Toán Phát Hiện Trần Thấp

```
┌──────────────────────────────────────────────────────────────────────┐
│                    CEILING DETECTION                                  │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  1. Lấy vùng CEILING ZONE (top 30% của frame)                       │
│                                                                      │
│  2. Lọc depth hợp lệ:                                               │
│     valid = (depth > 0.1m) AND (depth < 2.5m)                       │
│                                                                      │
│  3. Tính khoảng cách trần:                                          │
│     ceiling_distance = percentile(valid_depths, 10)                 │
│     (Lấy percentile 10 = điểm gần nhất)                             │
│                                                                      │
│  4. Smooth với history buffer (5 frames)                            │
│                                                                      │
│  5. Quyết định:                                                     │
│     - ceiling_distance < 0.5m → Không đủ không gian                 │
│     - ceiling_distance < 1.5m → Cảnh báo                            │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

## 5.4 Thuật Toán Phát Hiện Vật Cản (Linear Baseline)

Đây là phần quan trọng nhất - sử dụng **baseline tuyến tính** để phát hiện vật cản với camera nghiêng.

```
┌──────────────────────────────────────────────────────────────────────┐
│                GROUND OBSTACLE DETECTION                              │
│                (Linear Baseline Method)                               │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  1. Lấy vùng GROUND ZONE (60-100% của frame)                        │
│                                                                      │
│  2. Tính baseline depth cho TỪNG HÀNG (median):                     │
│                                                                      │
│     row_baseline[i] = median(valid_depths_in_row_i)                 │
│                                                                      │
│  3. Fit baseline TUYẾN TÍNH theo y:                                 │
│                                                                      │
│     fit = np.polyfit(y_indices, baseline_values, degree=1)          │
│     baseline_line = np.polyval(fit, all_y_indices)                  │
│                                                                      │
│     Tại sao tuyến tính?                                             │
│     → Camera nghiêng → hàng dưới gần hơn → depth nhỏ hơn            │
│     → Quan hệ depth-y xấp xỉ tuyến tính                             │
│                                                                      │
│     Depth                                                            │
│       │                                                              │
│       │    ○ ○ ○ ○                                                   │
│       │        ○ ○ ○ ○    ← baseline_line (fit)                     │
│       │            ○ ○ ○ ○                                           │
│       │                ○ ○ ○ ○                                       │
│       └────────────────────────▶ y (row index)                      │
│                                                                      │
│  4. Tìm vật cản: depth < baseline - threshold                       │
│                                                                      │
│     obstacle_mask[i,j] = depth[i,j] < (baseline_line[i] - 0.03m)   │
│                                                                      │
│  5. Tính chiều cao vật cản:                                         │
│                                                                      │
│     depth_diff = baseline_at_obstacle - obstacle_depth              │
│     avg_ground_angle = camera_tilt + FOV × 0.25                     │
│     height = depth_diff × sin(avg_ground_angle)                     │
│                                                                      │
│  6. Smooth với history buffer (5 frames)                            │
│                                                                      │
│  7. Quyết định:                                                     │
│     - height <= 5cm → có thể bước qua                               │
│     - height > 5cm → STOP                                           │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

## 5.5 Logic Xác Định Hành Động

```python
def _determine_action(self, ceiling, ground):
    # Priority logic:
    
    # Case 0: CẢ trần thấp VÀ vật cản → STOP
    if ceiling['detected'] and not ceiling['clearance_ok'] and ground['obstacle']:
        return ClearanceAction.STOP
        # → Không thể hạ gầm (vật cản) và không thể nâng gầm (trần)
    
    # Case 1: Trần quá thấp (không có vật cản) → LOWER
    elif ceiling['detected'] and not ceiling['clearance_ok']:
        return ClearanceAction.LOWER
        # → Hạ gầm xuống 6cm
    
    # Case 2: Vật cản quá cao → STOP
    elif ground['obstacle'] and not ground['can_step_over']:
        return ClearanceAction.STOP
        # → Vật cao hơn 5cm, không thể bước qua
    
    # Case 3: Vật cản có thể bước qua → RAISE
    elif ground['obstacle'] and ground['can_step_over']:
        return ClearanceAction.RAISE
        # → Nâng gầm lên 15cm
    
    # Case 4: Trần thấp nhưng qua được → LOWER
    elif ceiling['detected']:
        return ClearanceAction.LOWER
        # → Hạ gầm xuống
    
    # Case 5: Bình thường → NORMAL
    else:
        return ClearanceAction.NORMAL
        # → Giữ gầm 10cm
```

## 5.6 Thông Số Calibration (terrain_config.json)

```json
{
  "robot_dimensions": {
    "robot_height": 0.3,
    "min_ground_clearance": 0.07,
    "max_ground_clearance": 0.18,
    "normal_ground_clearance": 0.08
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
  "ceiling_detection": {
    "ceiling_min_clearance": 0.5,
    "ceiling_warning_distance": 1.5
  },
  "ground_obstacle_detection": {
    "ground_baseline_distance": 1.0,
    "obstacle_threshold": 0.06,
    "max_step_height": 0.05
  },
  "processing": {
    "depth_min_valid": 0.1,
    "depth_max_valid": 1,
    "smoothing_window": 5
  }
}
```

---

# 6. PHÁT HIỆN VẬT THỂ BẰNG AI VÀ ĐO KHOẢNG CÁCH

## 6.1 Tổng Quan

Hệ thống sử dụng **YOLOv8** để phát hiện vật thể trong thời gian thực, kết hợp với **Intel RealSense Depth** để đo khoảng cách chính xác đến từng vật thể.

**Files:**
- `src/perception/object_detector.py` - Phát hiện vật thể với YOLO
- `src/perception/depth_estimator.py` - Đo khoảng cách với depth camera
- `src/modes/patrol_mode.py` - Ứng dụng tuần tra phát hiện người xâm nhập

## 6.2 Module ObjectDetector

### Kiến Trúc

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      OBJECT DETECTOR PIPELINE                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  INPUT: Color Frame (640×480, BGR) + Depth Frame                        │
│         │                                                               │
│         ▼                                                               │
│  ┌──────────────────────────────────────────────────────┐              │
│  │ STEP 1: YOLO INFERENCE                                │              │
│  │                                                       │              │
│  │ results = model(frame,                               │              │
│  │                 conf=0.5,      # Confidence threshold │              │
│  │                 iou=0.45,      # NMS threshold       │              │
│  │                 verbose=False)                        │              │
│  │                                                       │              │
│  │ Output: List[Box] với class_id, confidence, bbox     │              │
│  └──────────────────────────────────────────────────────┘              │
│         │                                                               │
│         ▼                                                               │
│  ┌──────────────────────────────────────────────────────┐              │
│  │ STEP 2: MULTI-POINT DEPTH SAMPLING                    │              │
│  │                                                       │              │
│  │   Lấy 5 điểm trong bbox để đo depth:                 │              │
│  │                                                       │              │
│  │   ┌─────────────────────────┐                        │              │
│  │   │         ⬤ Upper        │                        │              │
│  │   │                         │                        │              │
│  │   │    ⬤    ⬤    ⬤       │ ← Left, Center, Right  │              │
│  │   │      (center)          │                        │              │
│  │   │                         │                        │              │
│  │   │         ⬤ Lower        │ ← Chân người           │              │
│  │   └─────────────────────────┘                        │              │
│  │                                                       │              │
│  │   depth = percentile(all_valid_depths, 25)           │              │
│  │   (Lấy 25th percentile để bỏ outliers)               │              │
│  └──────────────────────────────────────────────────────┘              │
│         │                                                               │
│         ▼                                                               │
│  ┌──────────────────────────────────────────────────────┐              │
│  │ STEP 3: OBSTACLE CLASSIFICATION                       │              │
│  │                                                       │              │
│  │ is_obstacle = (confidence >= 0.5) AND                │              │
│  │               (0 < depth < D_SAFE)                   │              │
│  │                                                       │              │
│  │ emergency_stop = (depth < D_EMERGENCY)               │              │
│  │                                                       │              │
│  │ D_SAFE = 1.5m      (khoảng cách an toàn)             │              │
│  │ D_EMERGENCY = 0.5m (dừng khẩn cấp)                   │              │
│  └──────────────────────────────────────────────────────┘              │
│         │                                                               │
│         ▼                                                               │
│  OUTPUT: ObjectDetectionResult                                          │
│          - objects: List[DetectedObject]                                │
│          - obstacles: List[DetectedObject] (trong D_SAFE)               │
│          - closest_obstacle: DetectedObject                             │
│          - emergency_stop: bool                                         │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Data Classes

```python
@dataclass
class DetectedObject:
    """Thông tin vật thể phát hiện được."""
    class_id: int                         # ID class (0=person, 1=bicycle, ...)
    class_name: str                       # Tên class ("person", "car", ...)
    confidence: float                     # Độ tin cậy (0-1)
    bbox: Tuple[int, int, int, int]       # Bounding box (x1, y1, x2, y2)
    center: Tuple[int, int]               # Tâm (cx, cy)
    depth: float                          # Khoảng cách (meters)
    is_obstacle: bool                     # Có phải chướng ngại vật

@dataclass
class ObjectDetectionResult:
    """Kết quả phát hiện hoàn chỉnh."""
    objects: List[DetectedObject]         # Tất cả vật thể
    obstacles: List[DetectedObject]       # Chướng ngại vật trong D_SAFE
    closest_obstacle: DetectedObject      # Vật gần nhất
    emergency_stop: bool                  # Cờ dừng khẩn cấp
```

### Cách Sử Dụng

```python
from src.perception import ObjectDetector

# Khởi tạo detector
detector = ObjectDetector(model_path="data/models/yolov8n.pt")

# Phát hiện vật thể
result = detector.detect(color_frame, depth_frame)

# Kiểm tra kết quả
for obj in result.objects:
    print(f"{obj.class_name}: {obj.depth:.2f}m, conf={obj.confidence:.2f}")
    
if result.emergency_stop:
    print("⚠️ DỪNG KHẨN CẤP!")
    
if result.closest_obstacle:
    print(f"Vật gần nhất: {result.closest_obstacle.depth:.2f}m")

# Visualization
vis_frame = detector.visualize(color_frame, result)
```

## 6.3 Module DepthEstimator

### Chức Năng

Module đo khoảng cách từ depth frame của RealSense với nhiều phương pháp:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       DEPTH ESTIMATION METHODS                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  1. CENTER POINT DEPTH                                                  │
│     └─ Đo tại tâm bounding box với median filter                       │
│                                                                         │
│  2. MULTI-POINT SAMPLING                                                │
│     └─ Lấy 5+ điểm trong bbox, trả về percentile                       │
│                                                                         │
│  3. BOX STATISTICS                                                      │
│     └─ avg_depth: Trung bình trong vùng                                │
│     └─ min_depth: Điểm gần nhất                                        │
│     └─ max_depth: Điểm xa nhất                                         │
│                                                                         │
│  4. CALIBRATION CORRECTION                                              │
│     └─ corrected = raw × correction_factor + offset                    │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Data Classes

```python
@dataclass
class BoxDepthResult:
    """Kết quả đo depth cho bounding box."""
    bbox: Tuple[int, int, int, int]   # (x1, y1, x2, y2)
    center: Tuple[int, int]           # Tâm (cx, cy)
    center_depth: float               # Depth tại tâm (meters)
    avg_depth: float                  # Depth trung bình
    min_depth: float                  # Depth nhỏ nhất (gần nhất)
    max_depth: float                  # Depth lớn nhất (xa nhất)
    valid: bool                       # Có hợp lệ không
```

### Cách Sử Dụng

```python
from src.perception import DepthEstimator

# Khởi tạo
estimator = DepthEstimator(
    median_filter_size=5,    # Kích thước filter
    min_valid_depth=0.1,     # Depth tối thiểu (m)
    max_valid_depth=10.0     # Depth tối đa (m)
)

# Đo tại bounding box
bbox = (100, 150, 200, 300)  # x1, y1, x2, y2
result = estimator.get_depth_at_center(depth_frame, bbox)

print(f"Center depth: {result.center_depth:.2f}m")
print(f"Min depth: {result.min_depth:.2f}m")
print(f"Avg depth: {result.avg_depth:.2f}m")

# Đo tại điểm cụ thể
depth = estimator.get_depth_at_point(depth_frame, x=320, y=240)

# Đo nhiều boxes
bboxes = [(100,100,200,200), (300,100,400,200)]
results = estimator.measure_boxes(depth_frame, bboxes)
```

## 6.4 Ứng Dụng Tuần Tra (Patrol Mode)

### Tổng Quan

Chế độ **Patrol** sử dụng AI để tuần tra tự động và phát hiện người xâm nhập:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         PATROL MODE STATE MACHINE                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│                          ┌─────────────┐                               │
│                     ┌───▶│ PATROLLING  │◀───┐                          │
│                     │    │ (đi thẳng)  │    │                          │
│                     │    └──────┬──────┘    │                          │
│                     │           │           │                          │
│                     │    timeout│           │ hết alert                │
│                     │           ▼           │                          │
│                     │    ┌─────────────┐    │                          │
│                     │    │  ROTATING   │    │                          │
│                     │    │ (quay 90°)  │    │                          │
│                     │    └──────┬──────┘    │                          │
│                     │           │           │                          │
│                     └───────────┘           │                          │
│                                             │                          │
│  Phát hiện người ──────────────────────────────────────┐               │
│                                             │          │               │
│                                             │          ▼               │
│                     ┌─────────────┐         │   ┌─────────────┐        │
│                     │  RETURNING  │◀────────┤   │   ALERT     │        │
│                     │ (quay lại)  │         │   │ 🚨 CẢNH BÁO │        │
│                     └─────────────┘         │   └──────┬──────┘        │
│                           ▲                 │          │               │
│                           │                 │          │ track_intruder│
│                     mất người               │          ▼               │
│                           │                 │   ┌─────────────┐        │
│                           └─────────────────┴───│  TRACKING   │        │
│                                                 │ (theo dõi)  │        │
│                                                 └─────────────┘        │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Cấu Hình PatrolConfig

```python
@dataclass
class PatrolConfig:
    # Patrol movement
    patrol_velocity: float = 0.3      # Tốc độ tuần tra (m/s)
    rotate_yaw_rate: float = 0.5      # Tốc độ quay (rad/s)
    
    # Waypoint timing
    patrol_forward_time: float = 5.0  # Đi thẳng 5 giây
    patrol_rotate_time: float = 3.0   # Quay 3 giây
    
    # Detection settings
    detect_class: str = "person"      # Lớp phát hiện
    min_confidence: float = 0.5       # Độ tin cậy tối thiểu
    min_box_area: int = 3000          # Diện tích bbox tối thiểu (pixels²)
    
    # Alert settings
    alert_distance: float = 3.0       # Khoảng cách cảnh báo (m)
    alert_duration: float = 3.0       # Thời gian alert (s)
    
    # Tracking settings
    track_intruder: bool = True       # Bật theo dõi
    tracking_distance: float = 2.0    # Giữ khoảng cách (m)
    tracking_gain: float = 1.5        # Gain điều khiển
    max_track_time: float = 10.0      # Thời gian track tối đa (s)
    
    # Sound alert
    enable_sound_alert: bool = True   # Bật còi cảnh báo
```

### Intruder Detection

```python
@dataclass
class Intruder:
    """Thông tin người xâm nhập."""
    bbox: Tuple[int, int, int, int]   # Bounding box
    center_x: float                    # Tâm X normalized (-1 to 1)
    center_y: float                    # Tâm Y normalized (-1 to 1)
    distance: float                    # Khoảng cách (meters)
    confidence: float                  # Độ tin cậy
    timestamp: float                   # Thời điểm phát hiện
```

### Logic Phát Hiện Người Xâm Nhập

```python
def _detect_intruder(self, color_frame, depth_frame) -> Optional[Intruder]:
    # 1. Chạy YOLO detection
    result = self.object_detector.detect(color_frame, depth_frame)
    
    for det in result.objects:
        # 2. Lọc theo class (chỉ "person")
        if det.class_name != "person":
            continue
        
        # 3. Lọc theo confidence
        if det.confidence < 0.5:
            continue
        
        # 4. Lọc theo kích thước bbox
        box_area = (x2 - x1) * (y2 - y1)
        if box_area < 3000:
            continue
        
        # 5. Lọc theo khoảng cách
        if det.depth > 3.0:  # Quá xa, bỏ qua
            continue
        
        # 6. Tính score và chọn người gần + rõ nhất
        score = confidence × (1.0 / (distance + 0.1))
        
    return best_intruder
```

### Cách Sử Dụng Patrol Mode

```python
from src.modes import PatrolMode, PatrolConfig

# Cấu hình tùy chỉnh
config = PatrolConfig(
    patrol_velocity=0.25,
    alert_distance=2.5,
    track_intruder=True
)

# Khởi tạo
patrol = PatrolMode(config)

# Set callback khi phát hiện
def on_intruder(intruder):
    print(f"🚨 Phát hiện người tại {intruder.distance:.1f}m!")
    # Gửi UART cảnh báo
    uart.send("B2")  # Alarm

patrol.set_alert_callback(on_intruder)

# Enable và chạy
patrol.enable()

while True:
    color, depth = camera.get_frames()
    output = patrol.process(color, depth)
    
    # Gửi lệnh điều khiển
    uart.send_velocity(output.velocity)
    uart.send_yaw_rate(output.yaw_rate)
    
    # Hiển thị
    cv2.imshow("Patrol", output.viz_frame)
```

### Visualization

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       PATROL MODE VISUALIZATION                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────┐       │
│  │                                                             │       │
│  │   ┌──────────────────┐                                      │       │
│  │   │ 🚨 ALERT: 2.3m  │  ← Bbox màu đỏ khi phát hiện         │       │
│  │   │                  │                                      │       │
│  │   │    [PERSON]      │                                      │       │
│  │   │    conf: 0.87    │                                      │       │
│  │   │                  │                                      │       │
│  │   └──────────────────┘                                      │       │
│  │                                                             │       │
│  │  ┌───────────────────────────────────────────────────────┐ │       │
│  │  │ State: TRACKING │ Intruder: 2.3m │ Patrol Cycle: 5   │ │       │
│  │  └───────────────────────────────────────────────────────┘ │       │
│  └─────────────────────────────────────────────────────────────┘       │
│                                                                         │
│  Màu bbox:                                                              │
│   🔴 Đỏ     = Alert (trong alert_distance)                             │
│   🟢 Xanh   = Bình thường (xa, không phải threat)                      │
│   🟠 Cam    = Warning (đang tiến gần)                                  │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 6.5 Thông Số Cấu Hình AI Detection

Trong `src/core/config.py`:

```python
# YOLO Configuration
YOLO_MODEL_PATH = "data/models/yolov8n.pt"
YOLO_CONFIDENCE_THRESHOLD = 0.5      # Ngưỡng tin cậy
YOLO_NMS_THRESHOLD = 0.45            # Non-max suppression

# Detect specific classes (None = all classes)
DETECT_CLASSES = {
    0: "person",
    1: "bicycle", 
    2: "car",
    # ... thêm class khác nếu cần
}

# Distance thresholds
D_SAFE = 1.5          # Khoảng cách an toàn (m)
D_EMERGENCY = 0.5     # Khoảng cách khẩn cấp (m)

# Depth processing
DEPTH_MEDIAN_FILTER_SIZE = 5
DEPTH_MIN_VALID = 0.1   # meters
DEPTH_MAX_VALID = 10.0  # meters

# Depth calibration
DEPTH_CALIBRATION_ENABLED = True
DEPTH_CORRECTION_FACTOR = 1.0
DEPTH_OFFSET = 0.0
```

## 6.6 Test Object Detector

**File:** `tools/testing/test_object_detector.py`

```bash
python tools/testing/test_object_detector.py
```

### Controls

| Key | Chức năng |
|-----|-----------|
| Q | Quit |
| + | Tăng confidence threshold |
| - | Giảm confidence threshold |
| S | Save frame |

---

# 7. GIAO TIẾP UART VỚI STM32

## 7.1 Tổng Quan

**File:** `src/communication/uart_controller.py`

### Thông Số Kết Nối

| Thông số | Giá trị |
|----------|---------|
| Port | /dev/ttyACM0 (Linux) hoặc COMx (Windows) |
| Baudrate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |

## 7.2 Protocol Command

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       UART COMMAND PROTOCOL                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Format: <COMMAND><VALUE>\n                                             │
│  All commands are UPPERCASE                                             │
│                                                                         │
│  ┌─────────┬───────────────────┬────────────────────────────────────┐  │
│  │ Command │ Format            │ Description                        │  │
│  ├─────────┼───────────────────┼────────────────────────────────────┤  │
│  │ E1      │ E1\n              │ Enable motor control               │  │
│  │ E0      │ E0\n              │ Disable motor control              │  │
│  ├─────────┼───────────────────┼────────────────────────────────────┤  │
│  │ Vxxx    │ V100\n            │ Velocity 0.1 m/s (value × 0.001)   │  │
│  │         │ V-150\n           │ Velocity -0.15 m/s (lùi)           │  │
│  ├─────────┼───────────────────┼────────────────────────────────────┤  │
│  │ Yxxx    │ Y500\n            │ Yaw rate 0.5 rad/s (value × 0.001) │  │
│  │         │ Y-300\n           │ Yaw rate -0.3 rad/s (quay trái)    │  │
│  ├─────────┼───────────────────┼────────────────────────────────────┤  │
│  │ Hxxx    │ H100\n            │ Leg height 10cm (value × 0.001 m)  │  │
│  │         │ H60\n             │ Leg height 6cm                     │  │
│  │         │ H150\n            │ Leg height 15cm                    │  │
│  ├─────────┼───────────────────┼────────────────────────────────────┤  │
│  │ B0      │ B0\n              │ Buzzer OFF                         │  │
│  │ B1      │ B1\n              │ Buzzer single BEEP                 │  │
│  │ B2      │ B2\n              │ Buzzer continuous ALARM            │  │
│  ├─────────┼───────────────────┼────────────────────────────────────┤  │
│  │ Rxxx    │ R100\n            │ Roll angle 0.1 rad                 │  │
│  │ J1      │ J1\n              │ Trigger jump                       │  │
│  │ ?       │ ?\n               │ Heartbeat ping (expects '!')       │  │
│  └─────────┴───────────────────┴────────────────────────────────────┘  │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 7.3 Mapping Hành Động → UART

```
┌───────────────────────────────────────────────────────────────────────┐
│                    TERRAIN ACTION → UART MAPPING                       │
├───────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌─────────────┬────────────┬─────────────┬─────────────────────────┐│
│  │ Action      │ Height     │ Buzzer      │ UART Commands           ││
│  ├─────────────┼────────────┼─────────────┼─────────────────────────┤│
│  │ NORMAL      │ 10cm       │ OFF         │ E1, H100, B0           ││
│  │ LOWER       │ 6cm        │ BEEP (1x)   │ E1, H60, B1            ││
│  │ RAISE       │ 15cm       │ OFF         │ E1, H150, B0           ││
│  │ STOP        │ giữ nguyên │ ALARM       │ E1, V0, Y0, B2         ││
│  └─────────────┴────────────┴─────────────┴─────────────────────────┘│
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

## 7.4 HeightController Class

```python
class HeightController:
    """Điều khiển chiều cao chân qua UART."""
    
    def __init__(self, uart, normal_height=0.10, raised_height=0.15, lowered_height=0.06):
        self.uart = uart
        self.normal_height = normal_height    # 10cm
        self.raised_height = raised_height    # 15cm
        self.lowered_height = lowered_height  # 6cm
        self.is_enabled = False
    
    def enable(self):
        """Enable motor control (E1)."""
        self.uart._send_command_direct("E1")
        self.is_enabled = True
    
    def set_height(self, height_m):
        """Gửi chiều cao qua UART: Hxxx"""
        if not self.is_enabled:
            self.enable()
        cmd = f"H{int(height_m * 1000)}"
        self.uart._send_command_direct(cmd)
    
    def set_normal(self):  self.set_height(self.normal_height)   # H100
    def set_raised(self):  self.set_height(self.raised_height)   # H150
    def set_lowered(self): self.set_height(self.lowered_height)  # H60
```

## 7.5 BuzzerController Class

```python
class BuzzerController:
    """Điều khiển còi qua UART."""
    
    def __init__(self, uart):
        self.uart = uart
        self.ceiling_beep_cooldown = 2.0  # Không kêu lại trong 2s
    
    def single_beep(self):
        """Kêu 1 tiếng ngắn (B1) - trần thấp."""
        self.uart._send_command_direct("B1")
    
    def continuous_alarm(self):
        """Kêu liên tục (B2) - STOP vật cản."""
        self.uart._send_command_direct("B2")
    
    def stop_alarm(self):
        """Tắt còi (B0)."""
        self.uart._send_command_direct("B0")
```

---

# 8. CÔNG CỤ CALIBRATION

## 8.1 Lane Calibration Tool

**File:** `tools/calibration/lane_calibration.py`

### Mục Đích
Điều chỉnh các thông số phát hiện đường line:
- ROI (vùng quan tâm) hình thang
- Threshold cho binary image
- Morphological kernel size
- Canny edge parameters
- Hough transform parameters

### Cách Sử Dụng

```bash
python tools/calibration/lane_calibration.py
```

### Controls

| Key | Chức năng |
|-----|-----------|
| S | Save parameters |
| C | Capture image |
| R | Reset to defaults |
| Q | Quit |

### Trackbars

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       LANE CALIBRATION TOOL                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ROI Shape:                                                             │
│  ┌──────────────┬────────────┬──────────────┐                          │
│  │ ROI Top Y %  │ 0 ──────── 100 │ Đỉnh ROI │                          │
│  │ ROI Bot Y %  │ 0 ──────── 100 │ Đáy ROI  │                          │
│  │ Top Left X % │ 0 ──────── 100 │ Góc trên trái │                     │
│  │ Top Right X %│ 0 ──────── 100 │ Góc trên phải │                     │
│  │ Bot Left X % │ 0 ──────── 100 │ Góc dưới trái │                     │
│  │ Bot Right X %│ 0 ──────── 100 │ Góc dưới phải │                     │
│  └──────────────┴────────────┴──────────────┘                          │
│                                                                         │
│  Processing:                                                            │
│  ┌──────────────┬────────────┬──────────────┐                          │
│  │ Threshold    │ 0 ──────── 255 │ Ngưỡng đen trắng │                  │
│  │ Morph Kernel │ 1 ──────── 15  │ Kernel size │                       │
│  │ Canny Low    │ 0 ──────── 255 │ Canny threshold │                   │
│  │ Canny High   │ 0 ──────── 255 │ Canny threshold │                   │
│  └──────────────┴────────────┴──────────────┘                          │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Output

Lưu vào `data/calibration/lane_params.json`

---

## 8.2 Terrain Obstacle Calibration Tool

**File:** `tools/calibration/terrain_obstacle_calibration.py`

### Mục Đích
Visualize và điều chỉnh thuật toán phát hiện vật cản:
- Xem depth heatmap
- Xem baseline fit (linear)
- Xem obstacle detection mask
- Xem depth profile theo cột

### Cách Sử Dụng

```bash
python tools/calibration/terrain_obstacle_calibration.py
```

### Controls

| Key | Chức năng |
|-----|-----------|
| Q | Quit |
| S | Save config |
| L | Load config |
| R | Reset to defaults |
| P | Pause/Resume |
| SPACE | Capture single frame |
| Mouse Click | Select column for depth profile |

### Đồ Thị Hiển Thị (4 graphs)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                  TERRAIN OBSTACLE CALIBRATION TOOL                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────┐  ┌─────────────────────┐                      │
│  │  1. DEPTH HEATMAP   │  │  2. BASELINE FIT    │                      │
│  │                     │  │                     │                      │
│  │  Depth frame với    │  │  Depth profile và   │                      │
│  │  ROI overlay        │  │  linear fit line    │                      │
│  │                     │  │                     │                      │
│  │  ████████████       │  │      ○ ○ ○ ○        │                      │
│  │  ████████████       │  │    ○ ─────○ ○      │                      │
│  │                     │  │      baseline       │                      │
│  └─────────────────────┘  └─────────────────────┘                      │
│                                                                         │
│  ┌─────────────────────┐  ┌─────────────────────┐                      │
│  │  3. OBSTACLE MASK   │  │  4. DEPTH PROFILE   │                      │
│  │                     │  │  (selected column)  │                      │
│  │  Binary mask các    │  │                     │                      │
│  │  điểm obstacle      │  │  Depth vs Y cho     │                      │
│  │                     │  │  1 cột cụ thể       │                      │
│  │  ░░░░████░░░░       │  │                     │                      │
│  │  ░░░░████░░░░       │  │  ─────┐             │                      │
│  │                     │  │       └────         │                      │
│  └─────────────────────┘  └─────────────────────┘                      │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Output

Lưu vào `data/calibration/terrain_config.json`

---

# 9. CÔNG CỤ TESTING

## 9.1 Test Terrain Analyzer

**File:** `tools/testing/test_terrain_analyzer.py`

### Mục Đích
Test toàn bộ chức năng terrain analyzer với UART buzzer và height control.

### Cách Sử Dụng

```bash
# Với hardware UART
python tools/testing/test_terrain_analyzer.py

# Với mock UART (không hardware)
python tools/testing/test_terrain_analyzer.py --mock-uart

# Không UART
python tools/testing/test_terrain_analyzer.py --no-uart
```

### Controls

| Key | Chức năng |
|-----|-----------|
| Q | Quit |
| R | Reset analyzer |
| S | Save frame |
| B | Test beep |
| C | Save calibration |
| L | Load calibration |
| D | Reset to defaults |

### Hiển Thị

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       TERRAIN ANALYZER TEST                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────────────┐  ┌─────────────────────────────┐      │
│  │     COLOR + OVERLAY         │  │      DEPTH COLORMAP         │      │
│  │                             │  │                             │      │
│  │  ┌─────────────────────┐   │  │  ┌─────────────────────┐   │      │
│  │  │ CEILING: 2.50m      │   │  │  │                     │   │      │
│  │  └─────────────────────┘   │  │  │                     │   │      │
│  │                             │  │  │                     │   │      │
│  │  ┌─────────────────────┐   │  │  │                     │   │      │
│  │  │ GROUND: 0cm @ 1.2m  │   │  │  │                     │   │      │
│  │  └─────────────────────┘   │  │  └─────────────────────┘   │      │
│  │                             │  │                             │      │
│  │  ┌─────────────────────────────────────────────────────┐   │      │
│  │  │ ACTION: NORMAL        │ Height: 10cm │ ✓ OK         │   │      │
│  │  └─────────────────────────────────────────────────────┘   │      │
│  └─────────────────────────────┴─────────────────────────────┘      │
│                                                                         │
│  Terminal Output:                                                       │
│  📏 UART Height: 10.0cm (H100)                                         │
│  📏 UART Height: 15.0cm (H150)  ← Khi phát hiện vật cản                │
│  🔔 BEEP! (Trần thấp)                                                  │
│  🚨 ALARM! (Vật cản - STOP)                                            │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Behavior

| Trạng thái | Buzzer | Height UART |
|------------|--------|-------------|
| NORMAL | OFF (B0) | H100 (10cm) |
| LOWER (trần thấp) | BEEP 1 lần (B1) | H60 (6cm) |
| RAISE (vật cản) | OFF (B0) | H150 (15cm) |
| STOP | ALARM liên tục (B2) | Giữ nguyên |

---

## 9.2 Test Line Detector

**File:** `tools/testing/test_line_detector.py`

### Mục Đích
Test phát hiện đường line với visualization.

### Cách Sử Dụng

```bash
python tools/testing/test_line_detector.py
```

---

## 9.3 Test UART Simple

**File:** `tools/testing/test_uart_simple.py`

### Mục Đích
Test giao tiếp UART cơ bản với STM32.

### Cách Sử Dụng

```bash
python tools/testing/test_uart_simple.py
```

---

# 10. HƯỚNG DẪN SỬ DỤNG

## 10.1 Cài Đặt

```bash
# Clone repository
git clone <repo_url>
cd autonomous_robot

# Tạo virtual environment
python -m venv .venv
source .venv/bin/activate  # Linux
# hoặc .venv\Scripts\activate  # Windows

# Cài dependencies
pip install -r requirements.txt
```

## 10.2 Calibration

### Bước 1: Calibrate Lane Detection

```bash
python tools/calibration/lane_calibration.py
```

1. Điều chỉnh ROI (vùng quan tâm) bằng trackbar
2. Điều chỉnh threshold cho đường line
3. Nhấn **S** để lưu
4. Nhấn **Q** để thoát

### Bước 2: Calibrate Terrain Analyzer

```bash
python tools/calibration/terrain_obstacle_calibration.py
```

1. Đặt camera ở vị trí lắp đặt trên robot
2. Điều chỉnh ground zone top/bottom
3. Điều chỉnh obstacle threshold
4. Kiểm tra baseline fit trong đồ thị
5. Nhấn **S** để lưu

## 10.3 Testing

### Test Terrain + UART

```bash
# Với hardware
python tools/testing/test_terrain_analyzer.py

# Không hardware
python tools/testing/test_terrain_analyzer.py --mock-uart
```

### Test Line Detector

```bash
python tools/testing/test_line_detector.py
```

## 10.4 Chạy Chương Trình Chính

```bash
# Với hardware
python run_line_follower.py

# Với mock UART
python run_line_follower.py --mock-uart

# Không visualization
python run_line_follower.py --no-viz

# Debug mode
python run_line_follower.py --debug
```

## 10.5 Thông Số Điều Khiển

Trong `run_line_follower.py`:

```python
# Speed control (m/s)
BASE_SPEED = 0.10         # Tốc độ cơ bản
MAX_SPEED = 0.15          # Tốc độ tối đa
MIN_SPEED = 0.05          # Tốc độ tối thiểu

# Steering control
STEERING_GAIN = 1.2       # Độ nhạy lái theo position error
HEADING_GAIN = 0.3        # Độ nhạy lái theo heading error
MAX_YAW_RATE = 0.7        # Tốc độ quay tối đa (rad/s)

# Line search
SEARCH_YAW_RATE = 0.5     # Tốc độ quay khi tìm line
MAX_FRAMES_LOST = 30      # Số frame mất line trước khi dừng
```

---

# 11. PHỤ LỤC KỸ THUẬT

## 11.1 Công Thức Tính Chiều Cao Vật Cản

```
                📷 Camera (cao 20cm, nghiêng θ)
                     │
                     │  d₂ (depth đến vật cản)
                     │ ╱
                     │╱
             ════════╳════════  ← Vật cản cao h
                    ╱│
                   ╱ │
                  ╱  │ d₁ (depth đến mặt đất)
                 ╱   │
════════════════╱════╧═══════════ Mặt đất

Công thức:
    depth_diff = d₁ - d₂
    avg_angle = camera_tilt + FOV × 0.25
    h = depth_diff × sin(avg_angle)

Ví dụ:
    d₁ = 1.5m, d₂ = 1.3m
    depth_diff = 0.2m
    avg_angle = 14° + 58° × 0.25 = 28.5°
    h = 0.2 × sin(28.5°) = 0.095m ≈ 9.5cm
```

## 11.2 Bảng Tham Chiếu Nhanh

### UART Commands

| Command | Ví dụ | Ý nghĩa |
|---------|-------|---------|
| E1 | E1\n | Enable control |
| E0 | E0\n | Disable control |
| V{x} | V100\n | Velocity 0.1 m/s |
| Y{x} | Y300\n | Yaw rate 0.3 rad/s |
| H{x} | H100\n | Height 10cm |
| B0 | B0\n | Buzzer off |
| B1 | B1\n | Single beep |
| B2 | B2\n | Continuous alarm |

### Terrain Actions

| Action | Height | Buzzer | Điều kiện |
|--------|--------|--------|-----------|
| NORMAL | 10cm | OFF | Không có obstacle |
| LOWER | 6cm | BEEP | Trần thấp |
| RAISE | 15cm | OFF | Vật cản ≤ 5cm |
| STOP | - | ALARM | Vật cản > 5cm hoặc trần + vật cản |

### Line Detection

| Thông số | Giá trị | Mô tả |
|----------|---------|-------|
| position_error | [-1, +1] | -1=trái, +1=phải |
| heading_error | radians | Góc lệch |
| confidence | [0, 1] | Độ tin cậy |
| frames_lost | 0-30 | Số frame mất line |

## 11.3 Troubleshooting

### Camera không nhận

```bash
# Check USB
lsusb | grep Intel

# Reset RealSense
sudo systemctl restart udev

# Check permissions
sudo usermod -a -G video $USER
```

### UART không kết nối

```bash
# Check port
ls /dev/ttyACM*
ls /dev/ttyUSB*

# Check permissions
sudo usermod -a -G dialout $USER

# Test với minicom
minicom -D /dev/ttyACM0 -b 115200
```

### Line detection không ổn định

1. Chạy `lane_calibration.py` để điều chỉnh ROI
2. Tăng/giảm threshold
3. Kiểm tra ánh sáng
4. Tăng morph kernel size

### Terrain analyzer false positive

1. Chạy `terrain_obstacle_calibration.py`
2. Điều chỉnh `obstacle_threshold` (tăng lên nếu false positive)
3. Điều chỉnh `ground_zone_top` (tăng lên để bỏ vùng xa)
4. Kiểm tra `depth_max_valid`

---

# KẾT LUẬN

Dự án Autonomous Robot đã hoàn thành với các chức năng chính:

✅ **Line Following** - Theo dõi đường line với recovery mode  
✅ **Terrain Analysis** - Phát hiện trần thấp và vật cản với baseline tuyến tính  
✅ **Height Control** - Tự động điều chỉnh gầm xe (6cm / 10cm / 15cm)  
✅ **Buzzer Alert** - Cảnh báo còi khi gặp chướng ngại  
✅ **UART Communication** - Giao tiếp với STM32  
✅ **Calibration Tools** - Công cụ hiệu chỉnh với GUI  
✅ **Testing Tools** - Công cụ test độc lập từng module  

---

**© 2026 Autonomous Robot Project Team**
