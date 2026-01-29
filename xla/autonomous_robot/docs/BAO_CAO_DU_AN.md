# BÁO CÁO CHI TIẾT DỰ ÁN AUTONOMOUS ROBOT
## Hệ Thống Robot Tự Hành Sử Dụng Computer Vision

---

## MỤC LỤC

1. [Tổng Quan Dự Án](#1-tổng-quan-dự-án)
2. [Kiến Thức Nền Tảng](#2-kiến-thức-nền-tảng)
3. [Kiến Trúc Hệ Thống](#3-kiến-trúc-hệ-thống)
4. [Chi Tiết Các Module](#4-chi-tiết-các-module)
5. [Thông Số Quan Trọng](#5-thông-số-quan-trọng-và-cách-điều-chỉnh)
6. [Thuật Toán](#6-thuật-toán-và-phương-pháp)
7. [Hướng Dẫn Calibration](#7-hướng-dẫn-calibration)
8. [Giao Tiếp UART](#8-giao-tiếp-uart-với-stm32)
9. [Troubleshooting](#9-xử-lý-lỗi-và-troubleshooting)
10. [Kết Luận](#10-kết-luận)

---

## 1. TỔNG QUAN DỰ ÁN

### 1.1. Mô Tả Dự Án

Dự án xây dựng hệ thống robot tự hành với các khả năng:
- 🛣️ **Lane Following**: Nhận diện và bám theo làn đường
- 🚧 **Obstacle Avoidance**: Phát hiện và né tránh vật cản
- 📏 **Distance Measurement**: Đo khoảng cách bằng camera depth
- 🧠 **Decision Making**: Ra quyết định thông minh với FSM
- 🎮 **Motion Control**: Điều khiển chuyển động mượt mà với PID

### 1.2. Phần Cứng

| Thành phần | Model | Chức năng |
|------------|-------|-----------|
| Camera RGB-D | Intel RealSense D435i | Thu thập hình ảnh RGB và depth map |
| Vi điều khiển | STM32 | Điều khiển động cơ, đọc encoder |
| Máy tính xử lý | PC/Jetson | Chạy thuật toán CV, AI |

### 1.3. Thư Viện Phần Mềm

```
Python 3.8+
├── OpenCV 4.x          # Xử lý ảnh
├── NumPy               # Tính toán ma trận
├── PyRealsense2        # Giao tiếp camera RealSense
├── Ultralytics YOLOv8  # Phát hiện vật thể
└── PySerial            # Giao tiếp UART
```

---

## 2. KIẾN THỨC NỀN TẢNG

### 2.1. Computer Vision

#### 2.1.1. Không Gian Màu (Color Spaces)

**RGB (Red, Green, Blue):**
- Mỗi pixel = 3 giá trị (R, G, B), mỗi giá trị 0-255
- Tổng: 256³ = 16.7 triệu màu
- ⚠️ Nhược điểm: Nhạy với thay đổi ánh sáng

**HLS (Hue, Lightness, Saturation):**
- **H (Hue)**: Màu sắc, 0-179 trong OpenCV
- **L (Lightness)**: Độ sáng, 0-255
- **S (Saturation)**: Độ bão hòa, 0-255
- ✅ Ưu điểm: Tách biệt màu sắc và độ sáng

**Grayscale (Ảnh xám):**
```
Gray = 0.299×R + 0.587×G + 0.114×B
```

#### 2.1.2. Morphological Operations

| Phép toán | Công thức | Ứng dụng |
|-----------|-----------|----------|
| **Erosion** | `dst = min(kernel region)` | Loại bỏ nhiễu nhỏ |
| **Dilation** | `dst = max(kernel region)` | Lấp đầy lỗ hổng |
| **Opening** | Erosion → Dilation | Loại nhiễu, giữ shape |
| **Closing** | Dilation → Erosion | Lấp lỗ, nối đường đứt |

```
Original     Erosion      Dilation     Opening      Closing
████████     ██████       ██████████   ████████     ████████
██  ████  →  ██  ██   →   ██  ██████   ██  ████     ██████████
████████     ██████       ██████████   ████████     ████████
```

#### 2.1.3. Canny Edge Detection

**Quy trình 5 bước:**

1. **Giảm nhiễu Gaussian**
   $$G(x,y) = \frac{1}{2\pi\sigma^2} e^{-\frac{x^2 + y^2}{2\sigma^2}}$$

2. **Tính gradient với Sobel**
   ```
   Sobel X:        Sobel Y:
   ┌────────────┐  ┌────────────┐
   │ -1  0  +1 │  │ -1  -2  -1 │
   │ -2  0  +2 │  │  0   0   0 │
   │ -1  0  +1 │  │ +1  +2  +1 │
   └────────────┘  └────────────┘
   ```
   $$G = \sqrt{G_x^2 + G_y^2}$$
   $$\theta = \arctan(G_y/G_x)$$

3. **Non-maximum Suppression** - Giữ cực đại địa phương

4. **Double Threshold** - Phân loại strong/weak edges

5. **Hysteresis** - Kết nối weak edges với strong edges

#### 2.1.4. Hough Transform

**Biểu diễn đường thẳng:**
$$\rho = x \cos\theta + y \sin\theta$$

**Parameters HoughLinesP:**

| Parameter | Ý nghĩa | Giá trị đề xuất |
|-----------|---------|-----------------|
| `rho` | Độ phân giải ρ (pixels) | 1 |
| `theta` | Độ phân giải θ (radians) | π/180 |
| `threshold` | Số vote tối thiểu | 20-50 |
| `minLineLength` | Độ dài tối thiểu | 20-100 |
| `maxLineGap` | Khoảng cách max để nối | 10-50 |

### 2.2. YOLOv8 Object Detection

#### Architecture Overview

```
Input     →   Backbone    →    Neck      →     Head     →   Output
Image         (CSPNet)        (PANet)         (Detect)      Predictions

640×640   →  Feature      →  Multi-scale →  Classification →  Boxes
RGB           Extraction     Fusion          + Regression     + Classes
```

#### Non-Maximum Suppression (NMS)

**IoU (Intersection over Union):**
$$IoU = \frac{Area_{intersection}}{Area_{union}}$$

```
┌─────────┐
│    A    │
│    ┌────┼────┐       IoU = 0: Không giao nhau
└────┼────┘    │       IoU = 1: Hoàn toàn trùng
     │    B    │       IoU > 0.5: Coi là "trùng"
     └─────────┘
```

### 2.3. Depth Sensing

#### Intel RealSense D435i

**Công nghệ:** Active IR Stereo Vision

```
IR Projector    Left IR Camera    Right IR Camera    RGB Camera
     │                │                  │                │
     ▼                ▼                  ▼                ▼
Chiếu pattern   Thu pattern IR     Thu pattern IR    Thu ảnh màu
IR lên scene         │                  │                │
                     └────────┬─────────┘                │
                              ▼                          │
                       Stereo Matching                   │
                              │                          │
                              ▼                          │
                        Depth Map  ←───── Alignment ─────┘
```

**Công thức tính depth:**
$$depth = \frac{focal\_length \times baseline}{disparity}$$

### 2.4. PID Controller

#### Công thức

$$u(t) = K_p \cdot e(t) + K_i \cdot \int e(\tau)d\tau + K_d \cdot \frac{de(t)}{dt}$$

**Trong đó:**
- $e(t)$: Error = Setpoint - Measured Value
- $K_p$: Proportional Gain
- $K_i$: Integral Gain  
- $K_d$: Derivative Gain

#### Vai trò các thành phần

| Thành phần | Tác dụng | Nhược điểm |
|------------|----------|------------|
| **P** | Phản ứng nhanh với error | Không triệt tiêu steady-state error |
| **I** | Triệt tiêu steady-state error | Gây overshoot nếu quá lớn |
| **D** | Giảm overshoot, tăng ổn định | Nhạy với noise |

### 2.5. Finite State Machine (FSM)

```
                    ┌─────────────────┐
                    │   CENTER_LANE   │◄─────────── All Clear
                    │   (Mặc định)    │
                    └────────┬────────┘
                             │
           ┌─────────────────┼─────────────────┐
           │                 │                 │
           ▼                 ▼                 ▼
   ┌───────────────┐ ┌───────────────┐ ┌───────────────┐
   │  LEFT_LANE    │ │   STOPPED     │ │  RIGHT_LANE   │
   │  (Né trái)    │ │ (Dừng khẩn    │ │  (Né phải)    │
   │               │ │  cấp)         │ │               │
   └───────────────┘ └───────────────┘ └───────────────┘
```

**Điều kiện chuyển:**
- Obstacle Center + Right Free → LEFT_LANE
- Obstacle Center + Left Free → RIGHT_LANE
- Obstacle < D_EMERGENCY → STOPPED
- All Clear → CENTER_LANE

---

## 3. KIẾN TRÚC HỆ THỐNG

### 3.1. Sơ Đồ Tổng Quan

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           MAIN CONTROL LOOP (30Hz)                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                         PERCEPTION LAYER                              │  │
│  │                                                                       │  │
│  │  ┌────────────┐    ┌────────────────┐    ┌────────────────────┐     │  │
│  │  │ RealSense  │───▶│ Lane Detector  │───▶│ Cross-Track Error  │     │  │
│  │  │ Camera     │    │ (Hough + Poly) │    │ Curvature          │     │  │
│  │  │            │    └────────────────┘    └────────────────────┘     │  │
│  │  │ RGB + Depth│                                                      │  │
│  │  │            │    ┌────────────────┐    ┌────────────────────┐     │  │
│  │  │            │───▶│ Object         │───▶│ Object List        │     │  │
│  │  │            │    │ Detector       │    │ (class, bbox,      │     │  │
│  │  │            │    │ (YOLOv8)       │    │  depth, lane)      │     │  │
│  │  └────────────┘    └────────────────┘    └────────────────────┘     │  │
│  │                                                                       │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                      │                                      │
│                                      ▼                                      │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                          CONTROL LAYER                                │  │
│  │                                                                       │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │                     STATE MACHINE (FSM)                         │  │  │
│  │  │  CENTER_LANE ←→ LEFT_LANE ←→ RIGHT_LANE ←→ STOPPED             │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │                              │                                        │  │
│  │                              ▼                                        │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │          MOTION CONTROLLER (PID + Rate Limiting)                │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │                                                                       │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                      │                                      │
│                                      ▼                                      │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                      COMMUNICATION LAYER                              │  │
│  │                                                                       │  │
│  │  ┌────────────────────────────────────────────────────────────────┐  │  │
│  │  │     UART CONTROLLER → STM32 (E1/E0, Vxxx, Yxxx, Hxxx)          │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  │                                                                       │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2. Data Flow Timeline

```
Frame N
┌────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────┐
│RGB │→│ Lane     │→│ State    │→│ Motion   │→│ UART     │→│ CMD │
│Depth│ │ Object   │ │ Machine  │ │ Control  │ │ Send     │ │ TX  │
└────┘  └──────────┘  └──────────┘  └──────────┘  └──────────┘  └────┘
│         │             │             │             │             │
0ms       10ms          15ms          18ms          20ms          33ms

◄─────────────────── 33ms (30 FPS) ─────────────────────────────────▶
```

---

## 4. CHI TIẾT CÁC MODULE

### 4.1. Camera Module

**File:** `src/perception/camera.py`

| Method | Mô tả | Return |
|--------|-------|--------|
| `start()` | Khởi động camera | bool |
| `stop()` | Dừng camera | None |
| `get_frames()` | Lấy RGB + Depth | (ndarray, ndarray) |
| `get_depth_at_point()` | Đo depth tại 1 điểm | float (meters) |

### 4.2. Lane Detector

**File:** `src/perception/lane_detector.py`

**Pipeline:**

```
Input Frame (640×480 BGR)
         │
         ▼
┌──────────────────┐
│ 1. Apply ROI     │  ← Mask hình thang
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ 2. Grayscale     │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ 3. Threshold     │  ← Binary cho làn đen
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ 4. Morphology    │  ← Close + Open
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ 5. Canny Edge    │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ 6. HoughLinesP   │  ← Phát hiện đoạn thẳng
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ 7. Clustering    │  ← Left/Center/Right
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ 8. Poly Fitting  │  ← x = Ay² + By + C
└────────┬─────────┘
         │
         ▼
Output: LaneDetectionResult
```

### 4.3. Object Detector

**File:** `src/perception/object_detector.py`

**DetectedObject Structure:**

```python
@dataclass
class DetectedObject:
    class_id: int          # COCO class index
    class_name: str        # "person", "car", etc.
    confidence: float      # 0-1
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[int, int]          # (cx, cy)
    depth: float           # meters
    lane: ObstacleLane     # LEFT/CENTER/RIGHT
    is_obstacle: bool      # depth < D_SAFE
```

### 4.4. State Machine

**File:** `src/control/state_machine.py`

**Transition Table:**

| Current State | Condition | Next State |
|--------------|-----------|------------|
| CENTER_LANE | obstacle_center && !obstacle_right | RIGHT_LANE |
| CENTER_LANE | obstacle_center && !obstacle_left | LEFT_LANE |
| CENTER_LANE | emergency_stop | STOPPED |
| LEFT_LANE | all_clear (sustained) | CENTER_LANE |
| RIGHT_LANE | all_clear (sustained) | CENTER_LANE |
| STOPPED | all_clear (sustained) | CENTER_LANE |

### 4.5. Motion Controller

**File:** `src/control/motion_controller.py`

**Velocity Calculation:**

```python
velocity = SPEED_NORMAL

# Giảm tốc khi cua
if curvature > CURVATURE_HIGH_THRESHOLD:
    velocity = min(velocity, SPEED_SLOW)

# Giảm tốc khi có obstacle
if has_obstacle:
    distance_ratio = obstacle_distance / D_SAFE
    obstacle_speed = SPEED_MIN + (SPEED_SLOW - SPEED_MIN) * distance_ratio
    velocity = min(velocity, obstacle_speed)

velocity = clamp(velocity, SPEED_MIN, SPEED_MAX)
```

---

## 5. THÔNG SỐ QUAN TRỌNG VÀ CÁCH ĐIỀU CHỈNH

### 5.1. Camera Parameters

| Parameter | Default | Range | Ảnh hưởng |
|-----------|---------|-------|-----------|
| `CAMERA_WIDTH` | 640 | 320-1920 | Resolution. Cao = chi tiết hơn, chậm hơn |
| `CAMERA_HEIGHT` | 480 | 240-1080 | Resolution |
| `CAMERA_FPS` | 30 | 15-60 | Frame rate |

### 5.2. ROI Parameters

```
0%─────────────────────────────────────────────────────────────────100%
│                                                                    │
│                     ╔═══════════════════════════╗                 │
40%   ROI_TOP_LEFT_X ─▶ ║                           ║ ◀─ ROI_TOP_RIGHT_X
│                     ║    ROI TRAPEZOID          ║
│                     ║    (Lane Detection Area)  ║
│                     ║                           ║
79% ROI_BOTTOM_LEFT_X─▶ ╚═══════════════════════════╝ ◀─ROI_BOTTOM_RIGHT_X
│                                                                    │
100%└────────────────────────────────────────────────────────────────┘
```

| Parameter | Default | Cách điều chỉnh |
|-----------|---------|-----------------|
| `ROI_TOP_Y` | 0.40 | Nhỏ hơn = nhìn xa hơn |
| `ROI_BOTTOM_Y` | 0.79 | Tránh = 1.0 để bỏ nắp xe |
| `ROI_TOP_LEFT_X` | 0.15 | Lớn hơn = thu hẹp vùng |
| `ROI_TOP_RIGHT_X` | 0.97 | Nhỏ hơn = thu hẹp vùng |

### 5.3. Lane Detection Parameters

#### Threshold

| Parameter | Default | Range | Cách điều chỉnh |
|-----------|---------|-------|-----------------|
| `BLACK_THRESHOLD` | 100 | 50-150 | Làn mờ → giảm; Nhiễu → tăng |

#### Morphology

| Parameter | Default | Range | Ảnh hưởng |
|-----------|---------|-------|-----------|
| `MORPH_KERNEL_SIZE` | 1 | 1-7 | Lớn = mạnh hơn |
| `MORPH_CLOSE_ITERATIONS` | 2 | 1-5 | Nhiều = lấp lỗ tốt |
| `MORPH_OPEN_ITERATIONS` | 1 | 1-3 | Nhiều = loại nhiễu |

#### Canny Edge

| Parameter | Default | Cách điều chỉnh |
|-----------|---------|-----------------|
| `CANNY_LOW_THRESHOLD` | 50 | Giảm = nhiều edge (nhiễu) |
| `CANNY_HIGH_THRESHOLD` | 94 | Thường = 2-3× low |

#### Hough Transform

| Parameter | Default | Cách điều chỉnh |
|-----------|---------|-----------------|
| `HOUGH_THRESHOLD` | 23 | Giảm = nhiều lines (có nhiễu) |
| `HOUGH_MIN_LINE_LENGTH` | 57 | Giảm = phát hiện đoạn ngắn |
| `HOUGH_MAX_LINE_GAP` | 29 | Tăng = nối nhiều đoạn đứt |

### 5.4. Obstacle Detection

| Parameter | Default | Range | Ý nghĩa |
|-----------|---------|-------|---------|
| `D_SAFE` | 2.0m | 1.5-5.0 | Object < D_SAFE = obstacle |
| `D_EMERGENCY` | 0.5m | 0.3-1.0 | Object < D_EMERGENCY = STOP |

### 5.5. PID Parameters

| Parameter | Default | Range | Cách điều chỉnh |
|-----------|---------|-------|-----------------|
| `PID_KP` | 0.005 | 0.001-0.02 | Tăng = phản ứng nhanh, có thể dao động |
| `PID_KI` | 0.0001 | 0-0.001 | Tăng = giảm steady-state error |
| `PID_KD` | 0.002 | 0-0.01 | Tăng = giảm overshoot |

**Quy trình tuning PID:**
1. Đặt Ki = Kd = 0
2. Tăng Kp cho đến khi bắt đầu dao động
3. Giảm Kp xuống ~60% giá trị đó
4. Tăng Kd để giảm overshoot
5. Thêm Ki nhỏ nếu cần

### 5.6. Speed Parameters

| Parameter | Default | Ý nghĩa |
|-----------|---------|---------|
| `SPEED_MAX` | 0.8 m/s | Tốc độ tối đa |
| `SPEED_NORMAL` | 0.6 m/s | Tốc độ bình thường |
| `SPEED_SLOW` | 0.3 m/s | Tốc độ khi có obstacle/cua |
| `SPEED_MIN` | 0.2 m/s | Tốc độ tối thiểu |

---

## 6. THUẬT TOÁN VÀ PHƯƠNG PHÁP

### 6.1. Cross-Track Error (CTE)

```
                    Image Frame (640×480)
┌─────────────────────────────────────────────────────────────────┐
│                                                                   │
│                        Target Point                               │
│                            ●                                      │
│                            │                                      │
│          CTE = target_x - center_x                               │
│                            │                                      │
│              ◄─────────────┼───────────▶                         │
│                            │                                      │
│                     Image Center (320, y)                        │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘

CTE > 0: Target ở bên phải → Robot rẽ phải
CTE < 0: Target ở bên trái → Robot rẽ trái
CTE = 0: Đang đúng hướng
```

### 6.2. Target Point Selection

**Ưu tiên:**
1. Nếu có CENTER_LANE: `target_x = center_lane tại look_ahead_y`
2. Nếu có LEFT + RIGHT: `target_x = (left_x + right_x) / 2`
3. Nếu chỉ có LEFT: `target_x = left_x + offset`
4. Nếu chỉ có RIGHT: `target_x = right_x - offset`
5. Không có gì: `target_x = 320` (image center)

### 6.3. Obstacle Lane Assignment

```
◄──────── LEFT ─────────▶◄────── CENTER ──────▶◄────── RIGHT ───────▶
0                       213                   426                  640
```

```python
if center_x < CAMERA_WIDTH / 3:
    lane = LEFT
elif center_x > CAMERA_WIDTH * 2 / 3:
    lane = RIGHT
else:
    lane = CENTER
```

---

## 7. HƯỚNG DẪN CALIBRATION

### 7.1. Lane Calibration

**Chạy tool:**
```bash
python -m tools.calibration.lane_calibration
```

**Các bước:**

1. **ROI**: Điều chỉnh vùng quan sát
2. **Threshold**: Điều chỉnh để làn đường hiện rõ
3. **Morphology**: Lấp lỗ, loại nhiễu
4. **Canny**: Edge rõ ràng
5. **Hough**: Số lines phù hợp
6. **Save**: Nhấn 'S' để lưu

### 7.2. Depth Calibration

**Chạy tool:**
```bash
python -m tools.calibration.depth_calibration
```

**Kiểm tra:**
1. Đặt vật ở khoảng cách đã biết
2. Click vào vật trong camera view
3. So sánh với thực tế
4. Sai số < 5% cho < 3m

### 7.3. PID Tuning

**Triệu chứng và điều chỉnh:**

| Triệu chứng | Nguyên nhân | Điều chỉnh |
|-------------|-------------|------------|
| Dao động qua lại | Kp quá cao | Giảm Kp |
| Phản ứng chậm | Kp quá thấp | Tăng Kp |
| Overshoot nhiều | Kd quá thấp | Tăng Kd |
| Steady-state error | Ki quá thấp | Tăng Ki |
| Giật khi có noise | Kd quá cao | Giảm Kd |

---

## 8. GIAO TIẾP UART VỚI STM32

### 8.1. Protocol

**Format:** `<COMMAND><VALUE>\n`

| Lệnh | Ý nghĩa | Ví dụ |
|------|---------|-------|
| `E1` | Enable control | `"E1\n"` |
| `E0` | Disable control | `"E0\n"` |
| `Vxxx` | Velocity (m/s × 1000) | `"V500\n"` = 0.5 m/s |
| `Yxxx` | Yaw rate (rad/s × 1000) | `"Y200\n"` = 0.2 rad/s |
| `Hxxx` | Height (m × 1000) | `"H150\n"` = 0.15m |

### 8.2. Quy ước dấu

- **Velocity (+)**: Tiến; **(-)**: Lùi
- **Yaw (+)**: Quay phải; **(-)**: Quay trái

---

## 9. XỬ LÝ LỖI VÀ TROUBLESHOOTING

### 9.1. Lỗi Camera

| Lỗi | Giải pháp |
|-----|-----------|
| Failed to start | Kiểm tra USB, driver |
| No frame | Reconnect, dùng USB 3.0 |
| FPS thấp | Giảm resolution, USB 3.0 |

### 9.2. Lỗi Lane Detection

| Lỗi | Giải pháp |
|-----|-----------|
| Không detect | Calibrate ROI, threshold |
| Detect sai | Tăng morph kernel, Hough params |
| Flickering | Tăng min_line_length |

### 9.3. Lỗi UART

| Lỗi | Giải pháp |
|-----|-----------|
| Permission denied | `sudo usermod -a -G dialout $USER` |
| Port not found | `ls /dev/ttyUSB*` |
| Không phản hồi | Check baudrate, reset STM32 |

---

## 10. KẾT LUẬN

### 10.1. Tóm tắt

Hệ thống robot tự hành bao gồm:
- **Perception**: RealSense + Lane Detection + Object Detection
- **Decision**: FSM với hysteresis
- **Control**: PID + Rate limiting
- **Communication**: UART với STM32

### 10.2. Điểm mạnh
- ✅ Modular design
- ✅ Real-time (30 FPS)
- ✅ Safety-first
- ✅ Configurable (YAML)

### 10.3. Hạn chế
- ⚠️ Phụ thuộc lane markings
- ⚠️ Depth range 0.1-10m
- ⚠️ Chưa có path planning

### 10.4. Hướng phát triển
1. Deep Learning Lane Detection
2. Sensor Fusion (LiDAR, IMU)
3. Path Planning (A*, RRT)
4. SLAM
5. V2X Communication

---

## CHECKLIST TRƯỚC KHI CHẠY

- [ ] Camera kết nối (`realsense-viewer`)
- [ ] UART đúng port (`ls /dev/ttyUSB*`)
- [ ] Model YOLO trong `data/models/`
- [ ] Config đã calibrate
- [ ] Test từng module
- [ ] Không gian an toàn
- [ ] Emergency stop sẵn sàng

---

*Document Version: 1.0 | Last Updated: January 2026*
