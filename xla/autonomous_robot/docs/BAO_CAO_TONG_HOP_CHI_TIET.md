# BÁO CÁO TỔNG HỢP CHI TIẾT DỰ ÁN AUTONOMOUS ROBOT
## Hướng Dẫn Toàn Diện Cho Người Mới Bắt Đầu Xử Lý Ảnh

**Phiên bản:** 2.0  
**Ngày cập nhật:** Tháng 1, 2026  
**Tác giả:** Autonomous Robot Project Team

---

# MỤC LỤC

1. [GIỚI THIỆU TỔNG QUAN](#1-giới-thiệu-tổng-quan)
2. [KIẾN THỨC NỀN TẢNG XỬ LÝ ẢNH](#2-kiến-thức-nền-tảng-xử-lý-ảnh)
3. [CAMERA VÀ DEPTH SENSING](#3-camera-và-depth-sensing)
4. [CẤU TRÚC DỰ ÁN CHI TIẾT](#4-cấu-trúc-dự-án-chi-tiết)
5. [PIPELINE XỬ LÝ CHÍNH](#5-pipeline-xử-lý-chính)
6. [MODULE PHÁT HIỆN ĐƯỜNG LINE](#6-module-phát-hiện-đường-line)
7. [MODULE PHÂN TÍCH ĐỊA HÌNH](#7-module-phân-tích-địa-hình)
8. [MODULE PHÁT HIỆN VẬT THỂ](#8-module-phát-hiện-vật-thể)
9. [GIAO TIẾP UART VỚI STM32](#9-giao-tiếp-uart-với-stm32)
10. [CÁC CHẾ ĐỘ HOẠT ĐỘNG](#10-các-chế-độ-hoạt-động)
11. [CÔNG CỤ CALIBRATION VÀ TESTING](#11-công-cụ-calibration-và-testing)
12. [PHÂN TÍCH FILE THỪA](#12-phân-tích-file-thừa)
13. [HƯỚNG DẪN SỬ DỤNG](#13-hướng-dẫn-sử-dụng)

---

# 1. GIỚI THIỆU TỔNG QUAN

## 1.1 Dự Án Là Gì?

Dự án **Autonomous Robot** xây dựng một hệ thống robot tự hành có khả năng:

| Chức năng | Mô tả | Module chính |
|-----------|-------|--------------|
| 🛣️ **Theo dõi đường line** | Robot đi theo vạch kẻ trên sàn | `SimpleLineDetector` |
| 🏔️ **Phân tích địa hình** | Phát hiện trần thấp, vật cản | `TerrainAnalyzer` |
| 🎯 **Bám theo vật thể** | Theo dõi người/xe bằng AI | `ObjectDetector` |
| 🚨 **Tuần tra phát hiện** | Phát hiện người lạ xâm nhập | `PatrolMode` |

## 1.2 Sơ Đồ Hệ Thống Tổng Quan

```
┌────────────────────────────────────────────────────────────────────────────┐
│                         AUTONOMOUS ROBOT SYSTEM                             │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌──────────────┐    ┌──────────────────┐    ┌────────────────────┐       │
│  │  RealSense   │    │   PERCEPTION     │    │    CONTROL         │       │
│  │  D435i       │───▶│                  │───▶│                    │       │
│  │  Camera      │    │ • Line Detector  │    │ • Motion Controller│       │
│  │              │    │ • Terrain Analyzer    │ • PID Control      │       │
│  │ ┌──────────┐ │    │ • Object Detector│    │                    │       │
│  │ │RGB + Depth│ │    │ • Depth Estimator│    └────────┬───────────┘       │
│  │ └──────────┘ │    └──────────────────┘             │                   │
│  └──────────────┘                                      │                   │
│                                                        ▼                   │
│                                              ┌─────────────────┐           │
│                                              │  UART Controller│           │
│                                              │  (Serial Comm)  │           │
│                                              └────────┬────────┘           │
│                                                       │                    │
└───────────────────────────────────────────────────────┼────────────────────┘
                                                        ▼
                                               ┌─────────────────┐
                                               │     STM32       │
                                               │  Motor Control  │
                                               │   + Sensors     │
                                               └─────────────────┘
```

## 1.3 Phần Cứng Sử Dụng

| Thiết bị | Model | Chức năng | Thông số |
|----------|-------|-----------|----------|
| **Camera** | Intel RealSense D435i | Thu ảnh RGB + Depth | 640×480 @ 30fps |
| **MCU** | STM32H7 | Điều khiển motor | UART 115200 baud |
| **PC** | Linux/Windows | Xử lý CV/AI | Python 3.8+ |

---

# 2. KIẾN THỨC NỀN TẢNG XỬ LÝ ẢNH

## 2.1 Ảnh Số Là Gì?

### Cấu Trúc Ảnh Số

Ảnh số là **ma trận 2D** của các **pixel** (picture element):

```
Ảnh Grayscale (1 kênh):          Ảnh RGB (3 kênh):
┌───┬───┬───┬───┬───┐           ┌───┬───┬───┬───┬───┐
│125│130│128│127│126│ ← Hàng 0  │R,G,B│R,G,B│...│...│...│
├───┼───┼───┼───┼───┤           ├───┼───┼───┼───┼───┤
│120│122│125│128│130│ ← Hàng 1  │...│...│...│...│...│
├───┼───┼───┼───┼───┤           ├───┼───┼───┼───┼───┤
│ . │ . │ . │ . │ . │           │...│...│...│...│...│
└───┴───┴───┴───┴───┘           └───┴───┴───┴───┴───┘
  ↑
  Mỗi pixel = giá trị 0-255

Với ảnh 640×480:
- Grayscale: 640 × 480 = 307,200 pixels
- RGB: 640 × 480 × 3 = 921,600 giá trị
```

### Tọa Độ Trong Ảnh

```
        x (cột) →
      0   1   2   ...  639
    ┌───┬───┬───┬─────┬───┐
  0 │   │   │   │     │   │
    ├───┼───┼───┼─────┼───┤
  1 │   │   │   │     │   │
y   ├───┼───┼───┼─────┼───┤
(hàng) │   │   │ P │     │   │  ← Pixel P tại (x=2, y=2)
↓   ├───┼───┼───┼─────┼───┤
... │   │   │   │     │   │
    ├───┼───┼───┼─────┼───┤
479 │   │   │   │     │   │
    └───┴───┴───┴─────┴───┘
```

**Lưu ý quan trọng:**
- **Gốc tọa độ (0,0)** ở góc trên bên trái
- **x** tăng sang phải (cột)
- **y** tăng xuống dưới (hàng)
- Trong NumPy: `image[y, x]` (hàng trước, cột sau)

## 2.2 Không Gian Màu

### RGB (Red, Green, Blue)

```
Màu = (R, G, B)

Ví dụ:
- Đỏ thuần: (255, 0, 0)
- Xanh lá: (0, 255, 0)
- Xanh dương: (0, 0, 255)
- Trắng: (255, 255, 255)
- Đen: (0, 0, 0)
- Xám: (128, 128, 128)

⚠️ OpenCV sử dụng BGR (ngược với RGB)!
```

### Grayscale (Ảnh Xám)

Công thức chuyển đổi (theo cảm nhận mắt người):

$$Gray = 0.299 \times R + 0.587 \times G + 0.114 \times B$$

```python
# Chuyển BGR sang Grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
```

### HLS (Hue, Lightness, Saturation)

Tách biệt màu sắc và độ sáng - hữu ích khi ánh sáng thay đổi:

```
┌─────────────────────────────────────────┐
│ H (Hue) - Màu sắc: 0-179 trong OpenCV   │
│   0°=Đỏ, 60°=Vàng, 120°=Xanh lá         │
├─────────────────────────────────────────┤
│ L (Lightness) - Độ sáng: 0-255          │
│   0=Đen, 128=Bình thường, 255=Trắng     │
├─────────────────────────────────────────┤
│ S (Saturation) - Độ bão hòa: 0-255      │
│   0=Xám, 255=Màu nguyên chất            │
└─────────────────────────────────────────┘
```

## 2.3 Phép Toán Hình Thái Học (Morphological Operations)

Các phép toán xử lý ảnh nhị phân (đen-trắng):

### Erosion (Xói mòn)

```
Trước:          Sau Erosion:     Tác dụng:
████████        ██████           - Thu nhỏ vùng trắng
██  ████   →    ██  ██           - Loại bỏ nhiễu nhỏ
████████        ██████           - Tách các vùng dính nhau
```

### Dilation (Giãn nở)

```
Trước:          Sau Dilation:    Tác dụng:
████████        ██████████       - Mở rộng vùng trắng
██  ████   →    ██████████       - Lấp đầy lỗ nhỏ
████████        ██████████       - Nối các vùng gần nhau
```

### Opening = Erosion → Dilation

```
Tác dụng: Loại bỏ nhiễu nhỏ mà vẫn giữ hình dạng gốc
```

### Closing = Dilation → Erosion

```
Tác dụng: Lấp đầy lỗ hổng mà vẫn giữ hình dạng gốc
```

**Code trong dự án:**
```python
# Tạo kernel hình chữ nhật
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

# Áp dụng closing để nối các đoạn line bị đứt
binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=3)

# Áp dụng opening để loại nhiễu
binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
```

## 2.4 Phân Ngưỡng (Thresholding)

### Binary Threshold Đơn Giản

```python
# Pixel < 80 → Trắng (line), Pixel >= 80 → Đen (nền)
_, binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
```

Công thức:
$$
\text{output}(x,y) = 
\begin{cases} 
255 & \text{if } \text{input}(x,y) < \text{threshold} \\
0 & \text{otherwise}
\end{cases}
$$

### Adaptive Threshold

Tự động điều chỉnh ngưỡng theo vùng lân cận:

```python
binary = cv2.adaptiveThreshold(
    gray,
    255,                              # Giá trị max
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,   # Dùng Gaussian blur cho vùng lân cận
    cv2.THRESH_BINARY_INV,            # Đảo ngược (line đen → trắng)
    blockSize=51,                     # Kích thước vùng lân cận (phải lẻ)
    C=10                              # Hằng số trừ đi từ mean
)
```

**Tại sao dùng Adaptive?**
- Ánh sáng không đều trong ảnh
- Một góc sáng, góc khác tối
- Threshold cố định sẽ thất bại

## 2.5 Canny Edge Detection

Thuật toán 5 bước để tìm cạnh:

```
Bước 1: Gaussian Blur        Bước 2: Gradient (Sobel)
   ┌───────────────┐            ┌───────────────┐
   │ Làm mờ để     │            │ Tính đạo hàm  │
   │ giảm nhiễu    │     →      │ theo x và y   │
   └───────────────┘            └───────────────┘
                                       │
   ┌───────────────┐            ┌──────▼────────┐
   │ Chỉ giữ cạnh  │            │ Chỉ giữ pixel │
   │ mạnh và       │     ←      │ có gradient   │
   │ liên kết      │            │ cực đại       │
   └───────────────┘            └───────────────┘
 Bước 5: Hysteresis          Bước 3: Non-max Suppression
                                       │
                               ┌───────▼───────┐
                               │ Phân loại     │
                               │ strong/weak   │
                               │ edges         │
                               └───────────────┘
                            Bước 4: Double Threshold
```

```python
edges = cv2.Canny(gray, 50, 150)  # low_threshold=50, high_threshold=150
```

---

# 3. CAMERA VÀ DEPTH SENSING

## 3.1 Intel RealSense D435i Hoạt Động Như Thế Nào?

### Cấu Tạo Camera

```
┌──────────────────────────────────────────────────────────────┐
│                    INTEL REALSENSE D435i                      │
│                                                              │
│   ┌────────┐    ┌────────┐    ┌────────┐    ┌────────┐      │
│   │IR Left │    │Projector│   │IR Right│    │  RGB   │      │
│   │Camera  │    │   IR    │   │Camera  │    │Camera  │      │
│   └────────┘    └────────┘    └────────┘    └────────┘      │
│       ↑              │             ↑             ↑           │
│       │              ▼             │             │           │
│       │    Chiếu pattern IR       │             │           │
│       │    lên không gian         │             │           │
│       │              │             │             │           │
│       └──────────────┴─────────────┘             │           │
│                      │                           │           │
│              ┌───────▼───────┐          ┌───────▼───────┐   │
│              │ Stereo Match  │          │   RGB Image   │   │
│              │ → Depth Map   │          │   640×480     │   │
│              └───────────────┘          └───────────────┘   │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### Nguyên Lý Stereo Vision (Tính Depth)

Giống như mắt người nhìn bằng 2 mắt để cảm nhận độ sâu:

```
                    Vật thể P
                        ●
                       /│\
                      / │ \
                     /  │  \
                    /   │d  \        d = khoảng cách từ camera đến vật
                   /    │    \
                  /     │     \
    IR Left     /      │      \     IR Right
    Camera ●──────────────────────● Camera
           └──────── B ─────────┘
                  Baseline (~50mm)
```

**Công thức tính depth:**

$$depth = \frac{f \times B}{disparity}$$

Trong đó:
- $f$ = focal length (tiêu cự) của camera
- $B$ = baseline (khoảng cách giữa 2 camera IR) ≈ 50mm
- $disparity$ = độ lệch pixel của cùng một điểm giữa 2 ảnh IR

**Ví dụ:**
- Vật ở xa: disparity nhỏ → depth lớn
- Vật ở gần: disparity lớn → depth nhỏ

### Depth Frame Trong Code

```python
# Mỗi pixel trong depth_frame chứa khoảng cách (đơn vị: mét)
depth_frame = np.array([
    [2.1, 2.0, 1.9, ...],  # Hàng 0: các khoảng cách
    [1.8, 1.7, 1.6, ...],  # Hàng 1
    ...
])

# Lấy khoảng cách tại pixel (x=320, y=240)
distance = depth_frame[240, 320]  # Ví dụ: 1.5 mét
```

## 3.2 Góc Nhìn Camera (Field of View)

```
                        Camera
                           │
               ┌───────────┼───────────┐
               │           │           │
               │    FOV    │           │
               │   ┌───────▼───────┐   │
         ←────►   │               │   │◄────►
          87°     │   Frame       │     58°
       Horizontal │   640×480     │   Vertical
                  │               │
                  └───────────────┘

RealSense D435 FOV:
- Horizontal: 87°
- Vertical: 58°
- Diagonal: 95°
```

### Pixel Nào Nhìn Đâu?

```
                    Camera (nghiêng 15° xuống)
                           ╲
          Top pixel         ╲ 15° - 29° = -14° (nhìn LÊN)
                             ╲
          Center pixel ───────╲─── 15° (nhìn xuống nhẹ)
                               ╲
          Bottom pixel ─────────╲── 15° + 29° = 44° (nhìn XUỐNG mặt đất)
                                 ╲
═══════════════════════════════════════════════════════
                    MẶT ĐẤT
```

**Với camera nghiêng 15° và FOV dọc 58°:**
- Top của frame nhìn lên 14° so với phương ngang → thấy trần
- Bottom của frame nhìn xuống 44° → thấy mặt đất gần

---

# 4. CẤU TRÚC DỰ ÁN CHI TIẾT

## 4.1 Sơ Đồ Thư Mục

```
autonomous_robot/
│
├── 📄 run_line_follower.py     # Entry point chính - chạy robot theo line
├── 📄 requirements.txt         # Dependencies
├── 📄 README.md                # Hướng dẫn nhanh
│
├── 📁 configs/                 # Cấu hình hệ thống
│   ├── default_config.yaml     # Camera, detection params
│   └── modes_config.yaml       # Mode-specific params
│
├── 📁 src/                     # SOURCE CODE CHÍNH
│   │
│   ├── 📁 core/                # Core utilities
│   │   └── config.py           # Quản lý cấu hình, load YAML
│   │
│   ├── 📁 perception/          # 👁️ PERCEPTION (Nhận thức)
│   │   ├── camera.py           # RealSense camera interface
│   │   ├── simple_line_detector.py  # ⭐ Phát hiện đường line
│   │   ├── terrain_analyzer.py      # ⭐ Phân tích địa hình
│   │   ├── object_detector.py       # YOLO object detection
│   │   └── depth_estimator.py       # Đo khoảng cách từ depth
│   │
│   ├── 📁 control/             # 🎮 CONTROL (Điều khiển)
│   │   └── motion_controller.py     # Tạo lệnh di chuyển
│   │
│   ├── 📁 communication/       # 📡 COMMUNICATION
│   │   └── uart_controller.py      # Giao tiếp UART với STM32
│   │
│   ├── 📁 modes/               # 🔄 OPERATION MODES
│   │   ├── base_mode.py             # Abstract base class
│   │   ├── line_following_mode.py   # Chế độ theo line
│   │   ├── object_tracking_mode.py  # Chế độ bám vật
│   │   └── patrol_mode.py           # Chế độ tuần tra
│   │
│   └── 📁 utils/               # Tiện ích
│       └── data_logger.py          # Ghi log dữ liệu
│
├── 📁 tools/                   # CÔNG CỤ HỖ TRỢ
│   │
│   ├── 📁 calibration/         # Hiệu chỉnh
│   │   ├── camera_calibration.py   # Hiệu chỉnh camera
│   │   ├── lane_calibration.py     # Hiệu chỉnh tham số lane
│   │   └── depth_calibration.py    # Hiệu chỉnh depth
│   │
│   └── 📁 testing/             # Test riêng lẻ các module
│       ├── test_camera.py          # Test camera
│       ├── test_line_detector.py   # Test line detector
│       ├── test_terrain_analyzer.py # Test terrain
│       ├── test_object_detector.py  # Test YOLO
│       ├── test_depth_estimator.py  # Test depth
│       ├── test_uart_simple.py      # Test UART
│       └── test_modules_interactive.py # Test tất cả
│
├── 📁 data/                    # DỮ LIỆU
│   ├── 📁 models/              # AI models
│   │   ├── yolov8n.pt          # YOLOv8 nano model
│   │   └── best.pt             # Custom trained model
│   │
│   └── 📁 calibration/         # Dữ liệu hiệu chỉnh
│       └── lane_params.json    # Thông số đã calibrate
│
├── 📁 docs/                    # TÀI LIỆU
│   ├── BAO_CAO_DU_AN.md
│   ├── LINE_DETECTOR_TECHNICAL_REPORT.md
│   ├── TERRAIN_ANALYZER_TECHNICAL_REPORT.md
│   └── UART_COMMAND_REFERENCE_VI.md
│
└── 📁 examples/                # VÍ DỤ SỬ DỤNG
    ├── object_tracking_example.py
    └── patrol_example.py
```

## 4.2 Vai Trò Từng Module

| Module | File | Chức năng chính |
|--------|------|-----------------|
| **Camera** | `camera.py` | Lấy frame RGB + Depth từ RealSense |
| **Line Detector** | `simple_line_detector.py` | Phát hiện đường line, tính error |
| **Terrain Analyzer** | `terrain_analyzer.py` | Phát hiện trần thấp, vật cản |
| **Object Detector** | `object_detector.py` | Phát hiện người/xe bằng YOLO |
| **Depth Estimator** | `depth_estimator.py` | Đo khoảng cách từ bounding box |
| **Motion Controller** | `motion_controller.py` | Tạo lệnh velocity, yaw |
| **UART Controller** | `uart_controller.py` | Gửi lệnh qua serial đến STM32 |

---

# 5. PIPELINE XỬ LÝ CHÍNH

## 5.1 Luồng Xử Lý Tổng Quan

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         MAIN CONTROL LOOP                                │
│                         (run_line_follower.py)                          │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                                                                  │   │
│  │  while running:                              30 Hz (~33ms/loop) │   │
│  │      │                                                          │   │
│  │      ▼                                                          │   │
│  │  ┌──────────────┐                                               │   │
│  │  │ Get Frames   │◀──── RealSenseCamera.get_frames()             │   │
│  │  │ RGB + Depth  │                                               │   │
│  │  └──────┬───────┘                                               │   │
│  │         │                                                       │   │
│  │         ├─────────────────┬─────────────────┐                   │   │
│  │         ▼                 ▼                 ▼                   │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │   │
│  │  │ Line         │  │ Terrain      │  │ (Optional)   │          │   │
│  │  │ Detection    │  │ Analysis     │  │ Object Det   │          │   │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘          │   │
│  │         │                 │                 │                   │   │
│  │         │ position_error  │ ceiling_dist   │ obstacles         │   │
│  │         │ heading_error   │ obstacle_height│                   │   │
│  │         ▼                 ▼                 ▼                   │   │
│  │  ┌──────────────────────────────────────────────────┐          │   │
│  │  │              CONTROL CALCULATION                  │          │   │
│  │  │                                                   │          │   │
│  │  │  velocity = f(position_error, confidence)        │          │   │
│  │  │  yaw_rate = STEERING_GAIN × pos_error            │          │   │
│  │  │           + HEADING_GAIN × heading_error         │          │   │
│  │  │                                                   │          │   │
│  │  └─────────────────────┬────────────────────────────┘          │   │
│  │                        │                                        │   │
│  │                        ▼                                        │   │
│  │  ┌──────────────────────────────────────────────────┐          │   │
│  │  │              UART SEND COMMANDS                   │          │   │
│  │  │                                                   │          │   │
│  │  │  V{velocity×1000}  →  "V150\n" (0.15 m/s)        │          │   │
│  │  │  Y{yaw×1000}       →  "Y500\n" (0.5 rad/s)       │          │   │
│  │  │  H{height×1000}    →  "H80\n"  (8cm leg height)  │          │   │
│  │  │                                                   │          │   │
│  │  └─────────────────────┬────────────────────────────┘          │   │
│  │                        │                                        │   │
│  │                        ▼                                        │   │
│  │                   TO STM32                                      │   │
│  │                                                                  │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 5.2 Code Minh Họa

```python
# Trích từ run_line_follower.py

def _control_step(self):
    """Một bước xử lý trong vòng lặp chính."""
    
    # 1. Lấy frame từ camera
    color_frame, depth_frame = self.camera.get_frames()
    
    # 2. Phát hiện line
    result = self.line_detector.detect(color_frame)
    
    # 3. Phân tích địa hình (mỗi N frame)
    if self._frame_count % TERRAIN_ANALYZE_INTERVAL == 0:
        terrain_result = self.terrain_analyzer.analyze(depth_frame)
        self._process_terrain(terrain_result)
    
    # 4. Tính toán điều khiển
    self._process_detection(result)
    
    # 5. Hiển thị visualization
    if self.enable_viz:
        self._update_display(color_frame, result, depth_frame, terrain_result)
```

---

# 6. MODULE PHÁT HIỆN ĐƯỜNG LINE

## 6.1 Tổng Quan

**File:** `src/perception/simple_line_detector.py`

**Mục tiêu:** Phát hiện đường line đen trên nền sáng và tính toán:
- `position_error`: Robot lệch trái/phải bao nhiêu
- `heading_error`: Robot hướng chệch bao nhiêu độ

## 6.2 Pipeline Chi Tiết

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
│  │ a) Grayscale conversion                               │              │
│  │    gray = cv2.cvtColor(frame, COLOR_BGR2GRAY)         │              │
│  │                                                       │              │
│  │ b) Gaussian blur (giảm nhiễu)                         │              │
│  │    gray = cv2.GaussianBlur(gray, (5,5), 0)            │              │
│  │                                                       │              │
│  │ c) Adaptive + Otsu thresholding                       │              │
│  │    binary = intersection(adaptive, otsu)              │              │
│  │                                                       │              │
│  │ d) Apply ROI mask (hình thang)                        │              │
│  │    binary = bitwise_and(binary, roi_mask)             │              │
│  │                                                       │              │
│  │ e) Morphological operations                           │              │
│  │    binary = close(binary) → open(binary)              │              │
│  └──────────────────────────────────────────────────────┘              │
│         │                                                               │
│         ▼ Binary image (line = trắng, nền = đen)                        │
│                                                                         │
│  ┌──────────────────────────────────────────────────────┐              │
│  │ STEP 2: HORIZONTAL SLICING                            │              │
│  │                                                       │              │
│  │ Chia ROI thành 10 slice ngang:                        │              │
│  │                                                       │              │
│  │   Slice 0: ─────────█████───────── → centroid_x = 320 │              │
│  │   Slice 1: ────────██████──────── → centroid_x = 315  │              │
│  │   Slice 2: ───────███████─────── → centroid_x = 310   │              │
│  │   ...                                                 │              │
│  │   Slice 9: ─────████████──────── → centroid_x = 290   │              │
│  │                                                       │              │
│  │ Mỗi slice: Tính centroid (trọng tâm) của pixels trắng │              │
│  │            centerline_points = [(x0,y0), (x1,y1), ...]│              │
│  └──────────────────────────────────────────────────────┘              │
│         │                                                               │
│         ▼ List of centerline points                                     │
│                                                                         │
│  ┌──────────────────────────────────────────────────────┐              │
│  │ STEP 3: ERROR CALCULATION                             │              │
│  │                                                       │              │
│  │ a) Position Error (độ lệch ngang):                    │              │
│  │    bottom_point = point gần robot nhất                │              │
│  │    center_x = frame_width / 2 + camera_offset         │              │
│  │    pos_error = (bottom_point.x - center_x) / (width/2)│              │
│  │    Range: [-1, +1]                                    │              │
│  │                                                       │              │
│  │ b) Heading Error (độ lệch hướng):                     │              │
│  │    Fit line qua các centerline points                 │              │
│  │    slope = dx/dy (thay đổi x theo y)                  │              │
│  │    heading_error = arctan(slope)                      │              │
│  │    Range: ~[-90°, +90°]                               │              │
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

## 6.3 Giải Thích Chi Tiết Từng Bước

### Bước 1: ROI (Region of Interest)

**Tại sao cần ROI?**
- Chỉ xử lý vùng quan trọng (đường phía trước)
- Loại bỏ nhiễu từ các vùng không liên quan
- Tăng tốc độ xử lý

```
Frame gốc 640×480:
┌────────────────────────────────────────────────────────────┐
│                                                            │
│                  ← Bỏ qua (trời, tường, ...)              │
│                                                            │
├────────────────────────────────────────────────────────────┤ y = 55%
│              ╱▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓╲                 │
│             ╱ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ ╲                │
│            ╱  ▓▓▓▓▓▓▓▓ ROI ZONE ▓▓▓▓▓▓▓▓  ╲               │
│           ╱   ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓   ╲              │
│          ╱    ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓    ╲             │
├─────────╱─────────────────────────────────────╲────────────┤ y = 95%
└────────────────────────────────────────────────────────────┘

Thông số ROI (hình thang - giống góc nhìn camera):
- Top: x từ 30% đến 70%    (hẹp - vùng xa)
- Bottom: x từ 10% đến 90% (rộng - vùng gần)
- Y từ 55% đến 95%         (nửa dưới frame)
```

### Bước 2: Horizontal Slicing

**Tại sao chia slice ngang?**
- Tìm vị trí line tại nhiều khoảng cách khác nhau
- Xây dựng hình dạng của đường line
- Robust với nhiễu cục bộ

```
Binary image sau threshold:

Slice 0 ─────────█████────────────  ← Line ở xa
Slice 1 ────────███████───────────
Slice 2 ───────█████████──────────
Slice 3 ──────███████████─────────
Slice 4 ─────█████████████────────
Slice 5 ────███████████████───────
Slice 6 ───█████████████████──────
Slice 7 ──███████████████████─────
Slice 8 ─█████████████████████────
Slice 9 ██████████████████████████  ← Line ở gần
        ↑                      ↑
     Left                   Right

Với mỗi slice:
1. Tìm tất cả pixel trắng (thuộc line)
2. Tính trung bình x của các pixel đó → centroid
3. Lưu vào centerline_points
```

### Bước 3: Tính Position Error

```
                    Frame 640 pixels
    ←───────────────────────────────────────────→
    │                    │                      │
    │       ●            │                      │
    │      Line          │                      │
    │     Center         │                      │
    │                    │                      │
    └────────────────────┼──────────────────────┘
                         │
                    Robot Center
                    (x = 320)

Position Error = (Line_x - Robot_x) / (width/2)

Ví dụ:
- Line ở x = 270, Robot ở x = 320
- Error = (270 - 320) / 320 = -0.156
- Âm = Line ở bên TRÁI → Robot cần rẽ TRÁI
```

### Bước 4: Tính Heading Error

```
        Line thẳng:              Line nghiêng phải:
        
        │                              ╱
        │                             ╱
        │                            ╱
        │                           ╱
        │                          ╱
        ●────────● Robot          ╱────────● Robot
                                 ╱
    heading = 0°             heading = +20°
                             (line nghiêng phải = quay phải)

Công thức:
1. Fit đường thẳng qua centerline_points: x = m×y + b
2. slope (m) = thay đổi x khi y thay đổi
3. heading_error = arctan(slope)
```

## 6.4 Chế Độ Khôi Phục (Recovery Mode)

Khi mất line, robot sẽ tự tìm lại:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       LINE RECOVERY MODE                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Trạng thái: LINE FOUND                                                 │
│      │                                                                  │
│      │ (mất line)                                                       │
│      ▼                                                                  │
│  ┌──────────────────────────┐                                          │
│  │ frames_lost = 1          │                                          │
│  │ Xác định hướng tìm kiếm: │                                          │
│  │ - Nếu line cuối ở PHẢI   │                                          │
│  │   → search_direction = +1│                                          │
│  │ - Nếu line cuối ở TRÁI   │                                          │
│  │   → search_direction = -1│                                          │
│  └──────────────────────────┘                                          │
│      │                                                                  │
│      │ (vẫn không thấy)                                                 │
│      ▼                                                                  │
│  ┌──────────────────────────┐                                          │
│  │ frames_lost > 3          │                                          │
│  │ Bắt đầu quay tìm:        │                                          │
│  │                          │                                          │
│  │   Frame 4-13:  Quay phải │                                          │
│  │   Frame 14-23: Quay trái │                                          │
│  │   Frame 24-33: Quay phải │                                          │
│  │   ... (dao động)         │                                          │
│  │                          │                                          │
│  │ Biên độ tăng dần:        │                                          │
│  │   0.3 → 0.4 → 0.5 → ...  │                                          │
│  └──────────────────────────┘                                          │
│      │                                                                  │
│      │ (tìm thấy line)                                                  │
│      ▼                                                                  │
│  ┌──────────────────────────┐                                          │
│  │ Reset về trạng thái      │                                          │
│  │ bình thường              │                                          │
│  └──────────────────────────┘                                          │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

# 7. MODULE PHÂN TÍCH ĐỊA HÌNH

## 7.1 Tổng Quan

**File:** `src/perception/terrain_analyzer.py`

**Mục tiêu:**
- Phát hiện trần thấp (gầm cầu, cửa thấp, ...)
- Phát hiện vật cản trên mặt đất
- Đề xuất hành động: NORMAL, LOWER, RAISE, STOP

## 7.2 Nguyên Lý Cơ Bản

### Depth Camera Đo Gì?

```
                    📷 Camera (trên robot)
                     │
                     │ ← Depth = khoảng cách THẲNG từ camera đến điểm
                     │
                     ╲
                      ╲ θ (góc nhìn)
                       ╲
                        ╲
                         ●────── Điểm P trên mặt đất
                              
Mỗi pixel trong depth frame = khoảng cách theo TIA NHÌN (cạnh huyền)
KHÔNG PHẢI khoảng cách ngang hay dọc riêng lẻ!
```

### Chia Frame Thành Các Vùng

```
                    Depth Frame (640×480)
┌────────────────────────────────────────────────────────────┐
│░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│ ← 0%
│░░░░░░░░░░░░░░░░ CEILING ZONE ░░░░░░░░░░░░░░░░░░░░░░░░░░░░│
│░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│ ← 30%
├────────────────────────────────────────────────────────────┤
│                                                            │
│                    (Vùng giữa - bỏ qua)                   │
│                                                            │
├────────────────────────────────────────────────────────────┤ ← 55%
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GROUND ZONE ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│ ← 90%
└────────────────────────────────────────────────────────────┘

Với camera nghiêng 15°:
- CEILING ZONE (0-30%): Nhìn lên ~14° → thấy trần
- GROUND ZONE (55-90%): Nhìn xuống ~30-44° → thấy mặt đất
```

## 7.3 Phát Hiện Trần Thấp

### Nguyên Lý

```
TRẦN BÌNH THƯỜNG (cao 3m):         TRẦN THẤP (cao 0.5m):

    (không có gì hoặc rất xa)           ████████████ Trần
              ↑                              ↑
              │ ~3m hoặc invalid             │ 0.5m
              │                              │
             📷                             📷

Trong depth frame:
- Ceiling zone có depth LỚN/invalid    - Ceiling zone có depth NHỎ (0.5m)
  → Không phát hiện trần                → Phát hiện trần thấp!
```

### Code Logic

```python
def _analyze_ceiling(self, depth_frame, h, w):
    # Lấy vùng ceiling (top 30%)
    ceiling_zone = depth_frame[0:int(h*0.3), :]
    
    # Lọc depth hợp lệ (0.1m - 5m)
    valid_depths = ceiling_zone[(ceiling_zone > 0.1) & (ceiling_zone < 5.0)]
    
    # Lấy điểm GẦN NHẤT (percentile 10)
    ceiling_distance = np.percentile(valid_depths, 10)
    
    # So sánh với ngưỡng
    if ceiling_distance < ceiling_warning_distance:  # < 1.5m
        return {'detected': True, 'distance': ceiling_distance}
```

## 7.4 Phát Hiện Vật Cản Trên Đất

### Nguyên Lý

```
MẶT ĐẤT PHẲNG:                     CÓ VẬT CẢN:

       📷                                📷
        ╲                                 ╲
         ╲                                 ╲
          ╲ d₁ = 1.5m                       ╲ d₂ = 1.2m
           ╲                                 ╲
════════════╲════════════           ═══┌────╲────┐════════
            ●                          │  ●    │ VẬT
         mặt đất                       └───────┘

Tất cả pixel trong ground zone     Một số pixel có depth NHỎ HƠN baseline
có depth TƯƠNG ĐƯƠNG (~1.5m)       (1.2m < 1.5m - 0.08m = 1.42m)
→ Không có vật cản                 → CÓ VẬT CẢN!
```

### Tính Chiều Cao Vật Cản

```
                📷 Camera (cao 20cm)
               ╱│╲
              ╱ │ ╲
             ╱  │  ╲
            ╱   │   ╲ θ = 30°
           ╱    │    ╲
          ╱     │     ╲
         ╱      │      ╲
        ●───────┼───────●
      VẬT       │    MẶT ĐẤT
      d₂=1.2m   │    d₁=1.5m

Công thức:
h_vật = (d₁ - d₂) × sin(θ)
      = (1.5 - 1.2) × sin(30°)
      = 0.3 × 0.5
      = 0.15m = 15cm
```

### Code Logic

```python
def _analyze_ground(self, depth_frame, h, w):
    # Lấy vùng ground (55-90%)
    ground_zone = depth_frame[int(h*0.55):int(h*0.9), :]
    
    # Baseline = khoảng cách đến mặt đất (median)
    baseline = np.median(valid_depths)  # ~1.5m
    
    # Tìm điểm GẦN HƠN baseline quá nhiều → vật cản
    obstacle_mask = valid_depths < (baseline - obstacle_threshold)
    
    if np.any(obstacle_mask):
        obstacle_distance = np.min(valid_depths[obstacle_mask])
        depth_diff = baseline - obstacle_distance
        
        # Ước tính chiều cao
        angle = camera_tilt + fov/4  # ~30°
        height = depth_diff * np.sin(np.radians(angle))
        
        return {'obstacle': True, 'height': height}
```

## 7.5 Quyết Định Hành Động

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        DECISION LOGIC                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │ IF trần < ceiling_min_clearance (0.5m)                           │  │
│  │     → ACTION: LOWER (Hạ gầm xuống minimum)                       │  │
│  │     → 🔔 Beep cảnh báo                                           │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                                │                                        │
│                                ▼ (không)                                │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │ IF vật_cản AND vật_cản.height > max_step_height (5cm)            │  │
│  │     → ACTION: STOP (Không thể vượt qua)                          │  │
│  │     → 🚨 Alarm liên tục                                          │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                                │                                        │
│                                ▼ (không)                                │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │ IF vật_cản AND vật_cản.height <= max_step_height                 │  │
│  │     → ACTION: RAISE (Nâng gầm để bước qua)                       │  │
│  │     → recommended_height = vật_cản.height + 2cm buffer           │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                                │                                        │
│                                ▼ (không)                                │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │ ELSE                                                              │  │
│  │     → ACTION: NORMAL (Giữ nguyên độ cao gầm bình thường)         │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 7.6 Bảng Tóm Tắt Thông Số

| Thông số | Giá trị | Ý nghĩa |
|----------|---------|---------|
| `ceiling_warning_distance` | 1.5m | Bắt đầu cảnh báo khi trần < 1.5m |
| `ceiling_min_clearance` | 0.5m | Phải hạ gầm khi trần < 0.5m |
| `obstacle_threshold` | 0.08m | Chênh lệch > 8cm = có vật cản |
| `max_step_height` | 0.05m | Bước qua được vật cao tối đa 5cm |
| `min_ground_clearance` | 0.07m | Gầm thấp nhất = 7cm |
| `max_ground_clearance` | 0.18m | Gầm cao nhất = 18cm |

---

# 8. MODULE PHÁT HIỆN VẬT THỂ

## 8.1 Tổng Quan

**File:** `src/perception/object_detector.py`

**Sử dụng:** YOLOv8 (You Only Look Once) - mạng neural network cho object detection

## 8.2 YOLOv8 Hoạt Động Như Thế Nào?

### Kiến Trúc Cơ Bản

```
INPUT                BACKBONE              NECK                HEAD              OUTPUT
640×640              Feature               Multi-scale         Detection         Boxes
RGB                  Extraction            Fusion              Layer             + Classes
  │                      │                     │                    │                │
  ▼                      ▼                     ▼                    ▼                ▼
┌─────┐              ┌─────────┐           ┌─────────┐        ┌─────────┐      ┌─────────┐
│     │              │ CSPNet  │           │  PANet  │        │ Detect  │      │ person  │
│     │     →        │ (DarkNet)│    →     │(Pyramid)│   →    │ Heads   │  →   │ 0.92    │
│     │              │         │           │         │        │         │      │ [x,y,w,h]│
└─────┘              └─────────┘           └─────────┘        └─────────┘      └─────────┘
```

### Output Của YOLO

```python
# YOLO trả về:
results = model(frame)

for box in results[0].boxes:
    class_id = int(box.cls[0])      # ID lớp (0=person, 1=bicycle, ...)
    class_name = model.names[class_id]  # Tên lớp
    confidence = float(box.conf[0])  # Độ tin cậy (0-1)
    x1, y1, x2, y2 = box.xyxy[0]    # Bounding box góc trên-trái, dưới-phải
```

### Non-Maximum Suppression (NMS)

Loại bỏ các box trùng lặp:

```
Trước NMS:                      Sau NMS:
┌─────────────┐                 ┌─────────────┐
│   ┌─────────┼───┐             │             │
│   │ person  │   │             │   person    │
│   │ 0.92    │   │      →      │   0.92      │
│   │ ┌───────┼───┼────┐        │             │
│   │ │person │   │    │        │             │
│   └─┼───────┘   │    │        └─────────────┘
│     │ 0.85     │    │
└─────┼───────────┘    │
      │ person 0.78    │        Chỉ giữ box có confidence cao nhất
      └────────────────┘        khi IoU > threshold (0.45)
```

## 8.3 Kết Hợp Với Depth

```python
# Sau khi phát hiện vật thể với YOLO
for obj in detected_objects:
    # Lấy tâm bounding box
    cx = (obj.x1 + obj.x2) // 2
    cy = (obj.y1 + obj.y2) // 2
    
    # Đọc depth tại tâm
    depth = depth_frame[cy, cx]
    
    # Kiểm tra có phải obstacle không
    is_obstacle = (depth > 0) and (depth < D_SAFE)  # D_SAFE = 2m
```

---

# 9. GIAO TIẾP UART VỚI STM32

## 9.1 Giao Thức Lệnh

| Lệnh | Chức năng | Đơn vị | Hệ số | Ví dụ | Kết quả |
|------|-----------|--------|-------|-------|---------|
| **E1** | Bật điều khiển | - | - | `E1\n` | Enable motor |
| **E0** | Tắt điều khiển | - | - | `E0\n` | Disable motor |
| **Vxxx** | Vận tốc tiến | m/s | ×1000 | `V800\n` | 0.8 m/s forward |
| **V-xxx** | Vận tốc lùi | m/s | ×1000 | `V-500\n` | 0.5 m/s backward |
| **Yxxx** | Tốc độ quay | rad/s | ×1000 | `Y500\n` | 0.5 rad/s right |
| **Y-xxx** | Quay trái | rad/s | ×1000 | `Y-300\n` | 0.3 rad/s left |
| **Hxxx** | Chiều cao gầm | m | ×1000 | `H100\n` | 10cm leg height |
| **Rxxx** | Góc roll | rad | ×1000 | `R100\n` | 0.1 rad tilt |
| **J1** | Nhảy | - | - | `J1\n` | Trigger jump |
| **?** | Heartbeat | - | - | `?\n` | Check connection |

## 9.2 Quy Trình Điều Khiển

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         UART CONTROL FLOW                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Python (PC)                              STM32                         │
│      │                                       │                          │
│      │ ──── "E1\n" ─────────────────────────▶│ Enable control           │
│      │                                       │                          │
│      │ ──── "V150\n" ───────────────────────▶│ velocity = 0.15 m/s      │
│      │                                       │                          │
│      │ ──── "Y300\n" ───────────────────────▶│ yaw_rate = 0.3 rad/s     │
│      │                                       │                          │
│      │ ◀─── "!\n" ──────────────────────────│ Heartbeat response       │
│      │                                       │                          │
│      │ ──── "E0\n" ─────────────────────────▶│ Disable (stop)           │
│      │                                       │                          │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 9.3 Watchdog / Heartbeat

Đảm bảo an toàn khi mất kết nối:

```python
# Python gửi heartbeat mỗi 500ms
HEARTBEAT_INTERVAL = 0.5

# Nếu không nhận được response trong 1.5s → coi như mất kết nối
HEARTBEAT_TIMEOUT = 1.5

# Sau 3 lần miss → emergency stop
MAX_MISSED_HEARTBEATS = 3
```

---

# 10. CÁC CHẾ ĐỘ HOẠT ĐỘNG

## 10.1 Tổng Quan Các Mode

| Mode | File | Mô tả | Phím tắt |
|------|------|-------|----------|
| **Line Following** | `line_following_mode.py` | Bám theo vạch kẻ | `1` |
| **Object Tracking** | `object_tracking_mode.py` | Bám theo người/xe | `2` |
| **Patrol** | `patrol_mode.py` | Tuần tra, phát hiện xâm nhập | `3` |
| **Idle** | - | Dừng, chờ lệnh | `0` |

## 10.2 State Machine Chung

```
                    ┌───────────┐
                    │   IDLE    │◀───────────────────┐
                    └─────┬─────┘                    │
                          │ start()                  │
                          ▼                          │
                    ┌───────────┐                    │
             ┌─────▶│  RUNNING  │────────────────────┤
             │      └─────┬─────┘                    │
             │            │                          │
             │            ▼                          │
             │      ┌───────────┐     error()        │
             │      │  ERROR    │────────────────────┘
             │      └─────┬─────┘
             │            │
             └────────────┘
               recover()
```

---

# 11. CÔNG CỤ CALIBRATION VÀ TESTING

## 11.1 Danh Sách Công Cụ

| Tool | File | Mục đích |
|------|------|----------|
| **test_camera.py** | `tools/testing/` | Test camera RGB + Depth |
| **test_line_detector.py** | `tools/testing/` | Test phát hiện line |
| **test_terrain_analyzer.py** | `tools/testing/` | Test phân tích địa hình |
| **test_object_detector.py** | `tools/testing/` | Test YOLO detection |
| **test_depth_estimator.py** | `tools/testing/` | Test đo khoảng cách |
| **test_uart_simple.py** | `tools/testing/` | Test UART commands |
| **lane_calibration.py** | `tools/calibration/` | Hiệu chỉnh tham số lane |

## 11.2 Cách Sử Dụng Test Tools

```bash
# Test camera
python tools/testing/test_camera.py

# Test line detector với trackbar điều chỉnh offset
python tools/testing/test_line_detector.py

# Test terrain analyzer với UART buzzer
python tools/testing/test_terrain_analyzer.py --mock-uart

# Test UART commands
python tools/testing/test_uart_simple.py --mock
```

---

# 12. PHÂN TÍCH FILE THỪA

## 12.1 Đánh Giá Các File

Sau khi đọc toàn bộ project, tôi đánh giá:

### ✅ Files Cần Thiết

| File/Folder | Lý do |
|-------------|-------|
| `src/perception/` | Core perception modules |
| `src/communication/` | UART communication |
| `src/control/` | Motion control |
| `src/modes/` | Operation modes |
| `src/core/config.py` | Configuration management |
| `run_line_follower.py` | Main entry point |
| `tools/testing/` | Testing tools |
| `tools/calibration/` | Calibration tools |
| `data/models/` | AI models |

### ⚠️ Files Có Thể Gộp/Xem Xét

| File | Nhận xét |
|------|----------|
| `docs/BAO_CAO_DU_AN.md` | Trùng lặp với file mới này |
| `docs/BAO_CAO_CHI_TIET_DU_AN.txt` | Nên xóa (định dạng txt khó đọc) |
| `docs/HUONG_DAN_DU_AN_CHI_TIET.md` | Có thể gộp vào README |
| `test_terrain_debug.py` | Chức năng trùng với `test_terrain_analyzer.py` |
| `test_modules_interactive.py` | Ít được sử dụng, có thể xóa |

### 🗑️ Files Có Thể Xóa

| File | Lý do |
|------|-------|
| `docs/BAO_CAO_CHI_TIET_DU_AN.txt` | Định dạng cũ, có .md tốt hơn |
| `tools/testing/test_terrain_debug.py` | Trùng chức năng với `test_terrain_analyzer.py` |

## 12.2 Đề Xuất Cấu Trúc Docs

```
docs/
├── BAO_CAO_TONG_HOP_CHI_TIET.md  # File này (tổng hợp tất cả)
├── UART_COMMAND_REFERENCE_VI.md  # Giữ lại (reference nhanh)
└── (xóa các file khác)
```

---

# 13. HƯỚNG DẪN SỬ DỤNG

## 13.1 Cài Đặt

```bash
# Clone project
git clone <repository>
cd autonomous_robot

# Tạo virtual environment
python -m venv .venv
source .venv/bin/activate  # Linux/Mac
# hoặc: .venv\Scripts\activate  # Windows

# Cài dependencies
pip install -r requirements.txt
```

## 13.2 Chạy Robot

```bash
# Chạy với UART thật
python run_line_follower.py

# Chạy với mock UART (test không cần hardware)
python run_line_follower.py --mock-uart

# Chạy không hiển thị GUI
python run_line_follower.py --no-viz

# Xem debug log
python run_line_follower.py --debug
```

## 13.3 Điều Chỉnh Tham Số

### Camera Offset

Nếu camera không ở chính giữa robot:

```python
# Trong src/core/config.py
CAMERA_OFFSET_X = 50  # Camera lệch phải 50 pixels
```

### ROI (Vùng quan tâm)

Điều chỉnh trong `tools/calibration/lane_calibration.py`:
- Chạy tool và dùng trackbar điều chỉnh
- Nhấn 'S' để lưu thông số

### Thông số điều khiển

Trong `run_line_follower.py`:

```python
BASE_SPEED = 0.10         # Tốc độ cơ bản (m/s)
MAX_SPEED = 0.15          # Tốc độ tối đa
STEERING_GAIN = 1.2       # Độ nhạy steering
HEADING_GAIN = 0.3        # Ảnh hưởng của góc lệch
```

## 13.4 Troubleshooting

| Vấn đề | Nguyên nhân | Giải pháp |
|--------|-------------|-----------|
| Camera không bật | Permission denied | `sudo chmod 666 /dev/video*` |
| UART không kết nối | Port sai | Kiểm tra `ls /dev/ttyACM*` |
| Robot không phản ứng | Chưa enable | Gọi `E1` trước khi gửi lệnh |
| Line không detect | Ánh sáng không đủ | Điều chỉnh threshold |
| Robot chạy lệch | Camera offset sai | Calibrate lại offset |

---

# PHỤ LỤC

## A. Glossary (Thuật ngữ)

| Thuật ngữ | Tiếng Việt | Giải thích |
|-----------|------------|------------|
| **Depth Frame** | Khung độ sâu | Ảnh chứa khoảng cách mỗi pixel |
| **ROI** | Vùng quan tâm | Region of Interest |
| **Threshold** | Ngưỡng | Giá trị phân chia 2 lớp |
| **Centroid** | Trọng tâm | Điểm trung bình của một vùng |
| **FOV** | Góc nhìn | Field of View |
| **Baseline** | Đường cơ sở | Khoảng cách giữa 2 camera |
| **Disparity** | Độ lệch | Sự khác biệt vị trí pixel giữa 2 ảnh |

## B. Công Thức Quan Trọng

### B.1 Tính Depth từ Stereo

$$depth = \frac{f \times B}{disparity}$$

### B.2 Tính Chiều Cao Vật Cản

$$h = (d_{ground} - d_{obstacle}) \times \sin(\theta)$$

### B.3 Position Error

$$error = \frac{x_{line} - x_{center}}{width / 2}$$

### B.4 Heading Error

$$heading = \arctan\left(\frac{dx}{dy}\right)$$

---

**END OF DOCUMENT**

*Tài liệu này được tạo để hướng dẫn toàn diện cho người mới tiếp xúc với xử lý ảnh và dự án robot tự hành.*
