# HƯỚNG DẪN DỰ ÁN ROBOT TỰ ĐỘNG - CHI TIẾT

**Dự án:** Autonomous Robot - Hệ thống robot tự hành đa chức năng  
**Phiên bản:** 1.0  
**Ngày cập nhật:** Tháng 1, 2026

---

## MỤC LỤC

1. [Tổng Quan Dự Án](#1-tổng-quan-dự-án)
2. [Kiến Trúc Hệ Thống](#2-kiến-trúc-hệ-thống)
3. [Cấu Trúc Thư Mục](#3-cấu-trúc-thư-mục)
4. [Module Perception (Nhận Thức)](#4-module-perception-nhận-thức)
5. [Module Communication (Giao Tiếp)](#5-module-communication-giao-tiếp)
6. [Module Control (Điều Khiển)](#6-module-control-điều-khiển)
7. [Module Modes (Chế Độ Hoạt Động)](#7-module-modes-chế-độ-hoạt-động)
8. [Module Core (Cấu Hình)](#8-module-core-cấu-hình)
9. [Các File Chạy Chính](#9-các-file-chạy-chính)
10. [Tools - Công Cụ Hỗ Trợ](#10-tools---công-cụ-hỗ-trợ)
11. [Luồng Dữ Liệu (Data Flow)](#11-luồng-dữ-liệu-data-flow)
12. [Lưu Ý Quan Trọng](#12-lưu-ý-quan-trọng)
13. [Câu Hỏi Thường Gặp](#13-câu-hỏi-thường-gặp)

---

## 1. Tổng Quan Dự Án

### 1.1 Mục Tiêu

Xây dựng hệ thống robot tự hành có khả năng:

| Chức năng | Mô tả | File chạy |
|-----------|-------|-----------|
| **Dò line** | Theo đường line đen trên nền sáng | `run_line_follower.py` |
| **Tuần tra** | Di chuyển tuần tra và phát hiện người xâm nhập | `examples/patrol_example.py` |
| **Theo dõi vật thể** | Khóa mục tiêu và giữ khoảng cách (gimbal-style) | `examples/object_tracking_example.py` |
| **Phân tích địa hình** | Phát hiện chướng ngại vật, nâng/hạ gầm | Tích hợp trong `run_line_follower.py` |

### 1.2 Phần Cứng Sử Dụng

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROBOT HARDWARE                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐         ┌─────────────────┐               │
│  │ Intel RealSense │         │ STM32           │               │
│  │ D435i Camera    │         │ Microcontroller │               │
│  │                 │         │                 │               │
│  │ - RGB 640×480   │  UART   │ - Motor Control │               │
│  │ - Depth 640×480 │ ──────→ │ - Leg Height    │               │
│  │ - 30 FPS        │         │ - Jump Control  │               │
│  └─────────────────┘         └─────────────────┘               │
│         │                            │                         │
│         ▼                            ▼                         │
│  ┌─────────────────┐         ┌─────────────────┐               │
│  │ Computer        │         │ Motors          │               │
│  │ (Python)        │         │ - Left leg      │               │
│  │ - OpenCV        │         │ - Right leg     │               │
│  │ - YOLO          │         │ - Servo         │               │
│  │ - RealSense SDK │         │                 │               │
│  └─────────────────┘         └─────────────────┘               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 Thông Số Camera RealSense D435i

| Thông số | Giá trị |
|----------|---------|
| Độ phân giải RGB | 640 × 480 pixels |
| Độ phân giải Depth | 640 × 480 pixels |
| FPS | 30 |
| FOV ngang | 87° |
| FOV dọc | 58° |
| Phạm vi depth | 0.1m - 10m |
| Góc gắn camera | 15° cúi xuống (so với phương ngang) |
| Độ cao camera | 20cm so với mặt đất |

---

## 2. Kiến Trúc Hệ Thống

### 2.1 Sơ Đồ Kiến Trúc Tổng Thể

```
┌─────────────────────────────────────────────────────────────────────┐
│                         AUTONOMOUS ROBOT                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────────────────┐   │
│  │   CAMERA    │   │ PERCEPTION  │   │         MODES           │   │
│  │  (Hardware) │ → │  (Modules)  │ → │    (Business Logic)     │   │
│  │             │   │             │   │                         │   │
│  │ RealSense   │   │ - Line Det. │   │ - LineFollowingMode     │   │
│  │ D435i       │   │ - Obj Det.  │   │ - PatrolMode            │   │
│  │             │   │ - Terrain   │   │ - ObjectTrackingMode    │   │
│  │             │   │ - Depth Est.│   │                         │   │
│  └─────────────┘   └─────────────┘   └───────────┬─────────────┘   │
│                                                   │                 │
│                                                   ▼                 │
│                    ┌─────────────┐   ┌─────────────────────────┐   │
│                    │   CONTROL   │ ← │        OUTPUT           │   │
│                    │             │   │                         │   │
│                    │ Motion      │   │ - velocity (m/s)        │   │
│                    │ Controller  │   │ - yaw_rate (rad/s)      │   │
│                    │             │   │ - clearance (m)         │   │
│                    └──────┬──────┘   └─────────────────────────┘   │
│                           │                                        │
│                           ▼                                        │
│                    ┌─────────────┐                                 │
│                    │    UART     │                                 │
│                    │ Controller  │                                 │
│                    │             │                                 │
│                    │ → STM32     │                                 │
│                    └─────────────┘                                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 Mô Hình Layer (Lớp)

```
┌─────────────────────────────────────────────────────┐
│ LAYER 4: Application (File chạy)                   │
│ run_line_follower.py, patrol_example.py, ...       │
├─────────────────────────────────────────────────────┤
│ LAYER 3: Modes (Logic điều khiển)                  │
│ LineFollowingMode, PatrolMode, ObjectTrackingMode  │
├─────────────────────────────────────────────────────┤
│ LAYER 2: Perception + Control (Xử lý)              │
│ SimpleLineDetector, ObjectDetector, TerrainAnalyzer│
│ MotionController, DepthEstimator                   │
├─────────────────────────────────────────────────────┤
│ LAYER 1: Hardware Interface (Giao tiếp phần cứng)  │
│ RealSenseCamera, UARTController                    │
├─────────────────────────────────────────────────────┤
│ LAYER 0: Configuration (Cấu hình)                  │
│ config.py, YAML files                              │
└─────────────────────────────────────────────────────┘
```

---

## 3. Cấu Trúc Thư Mục

```
autonomous_robot/
│
├── src/                          # SOURCE CODE CHÍNH
│   ├── core/                     # Cấu hình hệ thống
│   │   └── config.py            
│   │
│   ├── perception/               # Nhận thức (camera, detector)
│   │   ├── camera.py            
│   │   ├── simple_line_detector.py
│   │   ├── object_detector.py   
│   │   ├── depth_estimator.py   
│   │   └── terrain_analyzer.py  
│   │
│   ├── communication/            # Giao tiếp UART
│   │   └── uart_controller.py   
│   │
│   ├── control/                  # Điều khiển
│   │   └── motion_controller.py 
│   │
│   ├── modes/                    # Các chế độ hoạt động
│   │   ├── base_mode.py         
│   │   ├── line_following_mode.py
│   │   ├── patrol_mode.py       
│   │   └── object_tracking_mode.py
│   │
│   └── utils/                    # Tiện ích
│       └── data_logger.py       
│
├── configs/                      # CẤU HÌNH YAML
│   ├── default_config.yaml      
│   └── modes_config.yaml        
│
├── data/                         # DỮ LIỆU
│   ├── models/                   # Model YOLO
│   │   ├── yolov8n.pt           
│   │   └── best.pt              
│   └── calibration/              # Dữ liệu calibration
│       └── lane_params.json     
│
├── tools/                        # CÔNG CỤ HỖ TRỢ
│   ├── testing/                  # Test các module
│   │   ├── test_camera.py       
│   │   ├── test_line_detector.py
│   │   ├── test_object_detector.py
│   │   ├── test_terrain_analyzer.py
│   │   ├── test_depth_estimator.py
│   │   ├── test_modules_interactive.py
│   │   └── test_uart_simple.py
│   │
│   └── calibration/              # Công cụ calibration
│       ├── camera_calibration.py
│       ├── depth_calibration.py 
│       └── lane_calibration.py  
│
├── examples/                     # VÍ DỤ SỬ DỤNG
│   ├── patrol_example.py        
│   └── object_tracking_example.py
│
├── docs/                         # TÀI LIỆU
│   ├── HUONG_DAN_DU_AN_CHI_TIET.md
│   ├── LINE_DETECTOR_TECHNICAL_REPORT.md
│   ├── TERRAIN_ANALYZER_TECHNICAL_REPORT.md
│   └── UART_COMMAND_REFERENCE_VI.md
│
├── run_line_follower.py          # FILE CHẠY CHÍNH: Dò line
├── requirements.txt              # Thư viện Python
└── README.md                     # Giới thiệu dự án
```

---

## 4. Module Perception (Nhận Thức)

### 4.1 camera.py - Intel RealSense Camera Interface

**Vai trò:** Giao tiếp với camera RealSense D435i, lấy frame RGB và Depth.

**Chức năng chính:**

| Method | Mô tả |
|--------|-------|
| `start()` | Khởi động camera, cấu hình stream RGB + Depth |
| `stop()` | Dừng camera an toàn |
| `get_frames()` | Lấy cặp frame (color, depth) đã align |

**Cách hoạt động:**

```python
# Khởi tạo
camera = RealSenseCamera(width=640, height=480, fps=30)

# Bắt đầu
camera.start()

# Lấy frame
color_frame, depth_frame = camera.get_frames()
# color_frame: numpy array (480, 640, 3) - BGR
# depth_frame: numpy array (480, 640) - meters (float)

# Dừng
camera.stop()
```

**Điểm quan trọng:**
- **Align depth to color**: Depth frame được căn chỉnh với color frame để cùng tọa độ
- **Warm-up**: Skip 10 frame đầu để camera ổn định
- **Depth scale**: Tự động lấy từ camera (thường = 0.001)

---

### 4.2 simple_line_detector.py - Phát Hiện Đường Line

**Vai trò:** Phát hiện đường line đen trên nền sáng, tính toán error để điều khiển.

**Chức năng chính:**

| Method | Mô tả |
|--------|-------|
| `detect(frame)` | Phát hiện line, trả về LineDetectionResult |
| `visualize(frame, result)` | Vẽ visualization |
| `reset()` | Reset state (khi mất line quá lâu) |

**Output - LineDetectionResult:**

```python
@dataclass
class LineDetectionResult:
    line_detected: bool      # Có tìm thấy line không
    position_error: float    # Lệch ngang [-1, +1], dương = line bên phải
    heading_error: float     # Góc lệch (radians), dương = line nghiêng phải
    heading_error_degrees: float
    line_center_x: float     # Tọa độ X của line
    line_center_y: float     # Tọa độ Y của line
    confidence: float        # Độ tin cậy [0, 1]
    centerline_points: List  # Các điểm tạo thành line
    search_direction: int    # Hướng tìm khi mất line (-1/0/+1)
    frames_lost: int         # Số frame liên tiếp mất line
```

**Thuật toán:**

```
1. PREPROCESS:
   - Grayscale → Gaussian Blur
   - Adaptive Threshold + Otsu
   - Morphological Close/Open
   - Apply ROI mask (hình thang)

2. HORIZONTAL SLICING:
   - Chia ROI thành 10 lát ngang
   - Tìm centroid (tâm) của line trong mỗi lát
   - Nối các centroid → centerline

3. CALCULATE ERRORS:
   - position_error = (line_x - center_x) / (width/2)
   - heading_error = arctan(slope of fitted line)

4. LINE RECOVERY (khi mất line):
   - Chờ 3 frame
   - Bắt đầu quay tìm theo hướng line biến mất
   - Oscillating search với amplitude tăng dần
```

**Vùng xử lý (ROI):**

```
Frame 640×480:
┌──────────────────────────────────────────┐ 0%
│            (Bỏ qua phần trên)            │
│                                          │
├──────────────────────────────────────────┤ 55% (ROI_TOP_Y)
│         ╱─────────────────────╲          │
│        ╱   VÙNG XỬ LÝ (ROI)    ╲         │
│       ╱                         ╲        │
│      ╱                           ╲       │
│     ╱                             ╲      │
├────╱───────────────────────────────╲─────┤ 95% (ROI_BOTTOM_Y)
└──────────────────────────────────────────┘ 100%
```

**Lưu ý:**
- Chỉ xử lý vùng phía trước robot (55-95% chiều cao)
- ROI hình thang để phù hợp perspective
- Adaptive threshold tự động điều chỉnh theo ánh sáng

---

### 4.3 object_detector.py - Phát Hiện Vật Thể (YOLO)

**Vai trò:** Sử dụng YOLOv8 để phát hiện vật thể (người, xe, v.v.) và ước lượng khoảng cách.

**Chức năng chính:**

| Method | Mô tả |
|--------|-------|
| `detect(color_frame, depth_frame)` | Phát hiện vật thể |
| `visualize(frame, result)` | Vẽ bounding box |

**Output - ObjectDetectionResult:**

```python
@dataclass
class ObjectDetectionResult:
    objects: List[DetectedObject]      # Tất cả vật thể
    obstacles: List[DetectedObject]    # Vật cản (gần)
    closest_obstacle: DetectedObject   # Vật cản gần nhất
    emergency_stop: bool               # Cần dừng khẩn cấp?

@dataclass
class DetectedObject:
    class_id: int           # ID class trong COCO
    class_name: str         # Tên class (person, car, ...)
    confidence: float       # Độ tin cậy [0, 1]
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[int, int] # Tâm bounding box
    depth: float            # Khoảng cách (meters)
    is_obstacle: bool       # Có phải vật cản không
```

**Model YOLO:**

- File: `data/models/yolov8n.pt` (YOLOv8 nano - nhanh)
- 80 classes từ COCO dataset (person, car, bicycle, ...)
- Confidence threshold: 0.3 (có thể điều chỉnh)

**Multi-point Depth Sampling:**

```python
# Lấy depth từ 5 điểm trong bounding box
sample_points = [
    (cx, cy),           # Tâm
    (cx, cy + h//4),    # Dưới tâm (chân người)
    (cx - w//4, cy),    # Trái tâm
    (cx + w//4, cy),    # Phải tâm
    (cx, cy - h//6),    # Trên tâm
]
# Lấy percentile 25 → robust với outlier
```

---

### 4.4 depth_estimator.py - Ước Lượng Khoảng Cách

**Vai trò:** Đo khoảng cách từ depth frame tại vị trí bounding box.

**Chức năng:**

| Method | Mô tả |
|--------|-------|
| `get_depth_at_center(depth_frame, bbox)` | Depth tại tâm box |
| `get_depth_multi_point(depth_frame, bbox)` | Depth từ nhiều điểm |

**Output - BoxDepthResult:**

```python
@dataclass
class BoxDepthResult:
    bbox: Tuple[int, int, int, int]
    center: Tuple[int, int]
    center_depth: float  # Depth tại tâm
    avg_depth: float     # Trung bình
    min_depth: float     # Điểm gần nhất
    max_depth: float     # Điểm xa nhất
    valid: bool          # Có hợp lệ không
```

**Lọc nhiễu:**
- Median filter 3×3 để giảm noise
- Loại bỏ giá trị < 0.1m hoặc > 10m
- Calibration: `depth_corrected = depth * factor + offset`

---

### 4.5 terrain_analyzer.py - Phân Tích Địa Hình

**Vai trò:** Phát hiện trần thấp và chướng ngại vật trên mặt đất để điều chỉnh độ cao gầm.

**Chức năng:**

| Method | Mô tả |
|--------|-------|
| `analyze(depth_frame)` | Phân tích địa hình |
| `visualize(frame, result)` | Vẽ overlay |

**Output - TerrainAnalysisResult:**

```python
@dataclass
class TerrainAnalysisResult:
    ceiling_detected: bool       # Có phát hiện trần không
    ceiling_distance: float      # Khoảng cách đến trần (m)
    ground_obstacle: bool        # Có chướng ngại vật không
    obstacle_height: float       # Chiều cao chướng ngại vật (m)
    obstacle_distance: float     # Khoảng cách đến chướng ngại vật (m)
    action: ClearanceAction      # NORMAL / RAISE / LOWER / STOP
    recommended_height: float    # Độ cao gầm đề xuất (m)
    confidence: float
    message: str

class ClearanceAction(Enum):
    NORMAL = auto()   # Giữ nguyên (13cm)
    RAISE = auto()    # Nâng gầm (18cm)
    LOWER = auto()    # Hạ gầm (7cm)
    STOP = auto()     # Dừng (không thể qua)
```

**Vùng phân tích:**

```
Frame 640×480:
┌──────────────────────────────────────────┐ 0%
│░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│
│░░░░░░░░░ CEILING ZONE (0-30%) ░░░░░░░░░░░│
│░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│
├──────────────────────────────────────────┤ 30%
│                                          │
│              (Không xử lý)               │
│                                          │
├──────────────────────────────────────────┤ 55%
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│
│▓▓▓▓▓▓▓▓▓ GROUND ZONE (55-90%) ▓▓▓▓▓▓▓▓▓▓▓│
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│
├──────────────────────────────────────────┤ 90%
└──────────────────────────────────────────┘ 100%
```

**Công thức ước lượng chiều cao:**

```
Với camera nghiêng 15° xuống, FOV dọc 58°:
- Top frame nhìn lên: 15° - 29° = -14° (trên horizon)
- Bottom frame nhìn xuống: 15° + 29° = 44° (dưới horizon)

Chiều cao chướng ngại vật:
h = Δd × sin(θ_avg)
Trong đó:
- Δd = depth_obstacle - depth_ground
- θ_avg = 15° + 58° × 0.25 = 29.5° (góc trung bình của ground zone)
```

**Thông số độ cao gầm:**

| Trạng thái | Độ cao |
|------------|--------|
| Bình thường | 13cm |
| Nâng gầm (vượt chướng ngại vật) | 18cm |
| Hạ gầm (qua trần thấp) | 7cm |

---

## 5. Module Communication (Giao Tiếp)

### 5.1 uart_controller.py - Giao Tiếp UART với STM32

**Vai trò:** Gửi lệnh điều khiển đến STM32 qua UART, nhận feedback.

**Giao thức lệnh (ASCII):**

| Lệnh | Format | Ví dụ | Ý nghĩa |
|------|--------|-------|---------|
| Enable | `E1\n` | `E1` | Bật điều khiển motor |
| Disable | `E0\n` | `E0` | Tắt điều khiển motor |
| Velocity | `Vxxx\n` | `V800` | Vận tốc 0.8 m/s |
| Yaw Rate | `Yxxx\n` | `Y500` | Quay 0.5 rad/s |
| Height | `Hxxx\n` | `H130` | Độ cao gầm 13cm |
| Jump | `J1\n` | `J1` | Nhảy |
| Heartbeat | `?\n` | `?` | Kiểm tra kết nối |

**Chức năng chính:**

| Method | Mô tả |
|--------|-------|
| `connect()` | Kết nối UART |
| `disconnect()` | Ngắt kết nối |
| `enable_control()` | Bật điều khiển motor |
| `disable_control()` | Tắt điều khiển motor |
| `send_motion_command(velocity, yaw_rate)` | Gửi lệnh di chuyển |
| `send_emergency_stop()` | Dừng khẩn cấp |

**Watchdog/Heartbeat:**

```python
# Tự động gửi heartbeat mỗi 500ms
# Nếu không nhận được phản hồi 3 lần liên tiếp → emergency stop
HEARTBEAT_INTERVAL = 0.5  # seconds
MAX_MISSED_HEARTBEATS = 3
```

**Cấu hình UART:**

```python
UART_PORT = "/dev/ttyACM0"  # Linux
UART_BAUDRATE = 115200
UART_TIMEOUT = 0.1
UART_COMMAND_RATE_HZ = 10   # 10 lệnh/giây
```

**MockUARTController:**
- Giả lập UART để test không cần phần cứng
- Sử dụng: `python run_line_follower.py --mock-uart`

---

## 6. Module Control (Điều Khiển)

### 6.1 motion_controller.py - Điều Khiển Chuyển Động

**Vai trò:** Tạo lệnh điều khiển (velocity, yaw_rate) từ input.

**Chế độ:** Direct control - truyền thẳng lệnh không qua PID phức tạp.

```python
@dataclass
class MotionCommand:
    velocity: float           # Vận tốc tiến (m/s)
    yaw_rate: float          # Tốc độ quay (rad/s)
    is_emergency_stop: bool  # Dừng khẩn cấp
```

**Lưu ý:** Logic điều khiển chính nằm trong các Mode và file chạy.

---

## 7. Module Modes (Chế Độ Hoạt Động)

### 7.1 base_mode.py - Lớp Cơ Sở

**Vai trò:** Định nghĩa interface chung cho tất cả modes.

**States:**

```python
class ModeState(Enum):
    IDLE = auto()       # Chờ
    RUNNING = auto()    # Đang chạy
    SEARCHING = auto()  # Đang tìm
    PAUSED = auto()     # Tạm dừng
    ERROR = auto()      # Lỗi
    COMPLETED = auto()  # Hoàn thành
```

**Output:**

```python
@dataclass
class ModeOutput:
    velocity: float = 0.0      # Vận tốc (m/s)
    yaw_rate: float = 0.0      # Tốc độ quay (rad/s)
    state: ModeState           # Trạng thái
    confidence: float = 0.0    # Độ tin cậy
    message: str = ""          # Thông báo
    viz_frame: np.ndarray      # Frame visualization
    emergency_stop: bool       # Dừng khẩn cấp
```

**Interface:**

```python
class BaseMode(ABC):
    @abstractmethod
    def process(self, color_frame, depth_frame, feedback) -> ModeOutput:
        """Xử lý frame và trả về lệnh điều khiển."""
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """Reset state."""
        pass
    
    @abstractmethod
    def get_name(self) -> str:
        """Tên mode."""
        pass
```

---

### 7.2 line_following_mode.py - Chế Độ Dò Line

**Vai trò:** Theo dõi đường line đen, tính toán velocity và yaw_rate.

**Config:**

```python
@dataclass
class LineFollowingConfig:
    base_speed: float = 1.2      # Tốc độ cơ bản (m/s)
    max_speed: float = 2.0       # Tốc độ tối đa
    min_speed: float = 0.5       # Tốc độ tối thiểu (khi cua)
    steering_gain: float = 3.0   # Gain cho lái (pos_error → yaw_rate)
    heading_gain: float = 0.8    # Gain cho heading
    search_yaw_rate: float = 0.6 # Tốc độ quay khi tìm line
    max_frames_lost: int = 30    # Dừng sau 30 frame mất line
```

**Pipeline:**

```
┌─────────┐    ┌──────────────┐    ┌──────────────┐    ┌─────────────┐
│  Frame  │ →  │ Line Detect  │ →  │ Calculate    │ →  │ ModeOutput  │
│         │    │              │    │ Control      │    │             │
│         │    │ pos_error    │    │              │    │ velocity    │
│         │    │ heading_err  │    │ yaw_rate =   │    │ yaw_rate    │
│         │    │ confidence   │    │ K_p×pos +    │    │             │
│         │    │              │    │ K_h×heading  │    │             │
└─────────┘    └──────────────┘    └──────────────┘    └─────────────┘
```

**Công thức điều khiển:**

```python
yaw_rate = STEERING_GAIN * position_error + HEADING_GAIN * heading_error
velocity = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * speed_factor * confidence

# speed_factor giảm khi error lớn (cua gấp → chậm lại)
```

---

### 7.3 patrol_mode.py - Chế Độ Tuần Tra

**Vai trò:** Di chuyển tuần tra và phát hiện người xâm nhập.

**States:**

```python
class PatrolState(Enum):
    PATROLLING = auto()  # Đi tuần
    ROTATING = auto()    # Đang quay
    ALERT = auto()       # Phát hiện người
    TRACKING = auto()    # Theo dõi người
    RETURNING = auto()   # Quay lại tuyến
```

**Config:**

```python
@dataclass
class PatrolConfig:
    patrol_velocity: float = 0.3    # Tốc độ tuần tra (m/s)
    rotate_yaw_rate: float = 0.5    # Tốc độ quay (rad/s)
    patrol_forward_time: float = 5.0  # Đi thẳng 5s
    patrol_rotate_time: float = 3.0   # Quay 3s
    
    detect_class: str = "person"    # Class phát hiện
    min_confidence: float = 0.5     # Confidence tối thiểu
    alert_distance: float = 3.0     # Khoảng cách cảnh báo (m)
    
    track_intruder: bool = True     # Có theo dõi không
    tracking_distance: float = 2.0  # Khoảng cách giữ (m)
```

**Pattern tuần tra:**

```
        ┌───────────────────────────────────────────┐
        │                                           │
        │    ┌─────────────────────────────────┐    │
  START │    │                                 │    │
    ●───┼────┼──────→ FORWARD 5s ──────→ ●    │    │
        │    │                             │    │    │
        │    │                      ROTATE │    │    │
        │    │                       3s ↓  │    │    │
        │    │                             ●────┼────┼→ FORWARD 5s...
        │    │                                 │    │
        │    └─────────────────────────────────┘    │
        │                                           │
        └───────────────────────────────────────────┘
```

---

### 7.4 object_tracking_mode.py - Chế Độ Theo Dõi Vật Thể

**Vai trò:** Khóa mục tiêu và giữ khoảng cách (gimbal-style).

**Config:**

```python
@dataclass
class ObjectTrackingConfig:
    target_class: str = "person"      # Class theo dõi
    target_distance: float = 1.5      # Khoảng cách mong muốn (m)
    
    max_speed: float = 1.0            # Tốc độ tiến tối đa
    approach_speed: float = 0.5       # Tốc độ tiếp cận
    
    steering_gain: float = 2.0        # Gain lái
    steering_deadband: float = 0.05   # Vùng chết (không quay)
    
    distance_deadband: float = 0.2    # Vùng chết khoảng cách
    backup_on_too_close: bool = True  # Lùi nếu quá gần
    
    yaw_smoothing: float = 0.3        # Làm mịn quay
    velocity_smoothing: float = 0.3   # Làm mịn tốc độ
```

**Nguyên lý Gimbal-style:**

```
Target ở bên trái:                    Target ở giữa:
┌───────────────────────┐             ┌───────────────────────┐
│                       │             │                       │
│  ┌───┐                │             │         ┌───┐         │
│  │ T │     CENTER     │             │         │ T │         │
│  └───┘        ↓       │             │         └───┘         │
│              ┼        │             │           ↓           │
│                       │             │           ┼           │
│   ← QUAY TRÁI         │             │      ĐI THẲNG         │
└───────────────────────┘             └───────────────────────┘

Target quá gần:                       Target quá xa:
┌───────────────────────┐             ┌───────────────────────┐
│                       │             │                       │
│         ┌───┐         │             │         ┌───┐         │
│         │ T │         │             │         │ T │         │
│         └───┘         │             │         └───┘         │
│           │           │             │           │           │
│           ▼ (< 0.5m)  │             │           │ (> 1.5m)  │
│        LÙI LẠI        │             │       TIẾN TỚI        │
└───────────────────────┘             └───────────────────────┘
```

**Smoothing (Low-pass filter):**

```python
# Làm mịn yaw_rate để tránh giật
smoothed_yaw = α × current_yaw + (1-α) × prev_smoothed_yaw
# α = 0.3 → phản hồi chậm nhưng mượt
# α = 0.7 → phản hồi nhanh nhưng giật
```

---

## 8. Module Core (Cấu Hình)

### 8.1 config.py - Cấu Hình Hệ Thống

**Vai trò:** Tập trung tất cả thông số cấu hình, hỗ trợ load từ YAML.

**Các nhóm cấu hình:**

```python
# Camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_OFFSET_X = 0  # Offset camera so với tâm robot

# ROI (vùng quan tâm cho line detection)
ROI_TOP_Y = 0.55
ROI_BOTTOM_Y = 0.95
ROI_TOP_LEFT_X = 0.30
ROI_TOP_RIGHT_X = 0.70
ROI_BOTTOM_LEFT_X = 0.10
ROI_BOTTOM_RIGHT_X = 0.90

# Line Detection
BLACK_THRESHOLD = 80
MORPH_KERNEL_SIZE = 3
MORPH_CLOSE_ITERATIONS = 3
MORPH_OPEN_ITERATIONS = 1

# Object Detection
YOLO_MODEL_PATH = "data/models/yolov8n.pt"
YOLO_CONFIDENCE_THRESHOLD = 0.3
YOLO_NMS_THRESHOLD = 0.4

# Depth
DEPTH_MEDIAN_FILTER_SIZE = 3
DEPTH_MIN_VALID = 0.1
DEPTH_MAX_VALID = 10.0

# Obstacle
D_SAFE = 2.0       # Khoảng cách an toàn (m)
D_EMERGENCY = 0.5  # Khoảng cách khẩn cấp (m)

# UART
UART_PORT = "/dev/ttyACM0"
UART_BAUDRATE = 115200
UART_COMMAND_RATE_HZ = 10

# System
MAIN_LOOP_RATE_HZ = 30
```

**Load từ YAML:**

```python
config.load_config("configs/default_config.yaml")
```

---

## 9. Các File Chạy Chính

### 9.1 run_line_follower.py - Dò Line + Điều Chỉnh Gầm

**Vai trò:** Chạy robot dò line, tích hợp TerrainAnalyzer để nâng/hạ gầm.

**Cách chạy:**

```bash
# Chạy bình thường
python run_line_follower.py

# Test không cần phần cứng
python run_line_follower.py --mock-uart

# Không hiển thị cửa sổ
python run_line_follower.py --no-viz

# Debug mode
python run_line_follower.py --debug
```

**Thông số điều khiển:**

```python
BASE_SPEED = 1.2       # Tốc độ cơ bản (m/s)
MAX_SPEED = 2.0        # Tốc độ tối đa
MIN_SPEED = 0.8        # Tốc độ tối thiểu
STEERING_GAIN = 3.0    # Gain lái
HEADING_GAIN = 0.8     # Gain heading
SEARCH_YAW_RATE = 0.6  # Tốc độ quay khi tìm line
MAX_FRAMES_LOST = 30   # Dừng sau 30 frame mất line
```

**Pipeline:**

```
                    ┌──────────────────────────────────────────────┐
                    │            run_line_follower.py              │
                    └───────────────────┬──────────────────────────┘
                                        │
         ┌──────────────────────────────┼──────────────────────────────┐
         ▼                              ▼                              ▼
┌─────────────────┐           ┌─────────────────┐           ┌─────────────────┐
│ RealSenseCamera │           │ SimpleLineDetect│           │ TerrainAnalyzer │
│                 │           │                 │           │                 │
│ color_frame     │──────────→│ position_error  │           │ action          │
│ depth_frame     │           │ heading_error   │           │ recommended_hgt │
└────────┬────────┘           └────────┬────────┘           └────────┬────────┘
         │                             │                             │
         │                             ▼                             ▼
         │                    ┌─────────────────┐           ┌─────────────────┐
         │                    │ Calculate       │           │ Send Clearance  │
         │                    │ yaw_rate, vel   │           │ Command         │
         │                    └────────┬────────┘           │ HEIGHT xxx      │
         │                             │                    └─────────────────┘
         │                             ▼
         │                    ┌─────────────────┐
         └───────────────────→│ UART Controller │
                              │                 │
                              │ Vxxx, Yxxx      │
                              └────────┬────────┘
                                       │
                                       ▼
                              ┌─────────────────┐
                              │     STM32       │
                              │   → Motors      │
                              └─────────────────┘
```

---

### 9.2 examples/patrol_example.py - Tuần Tra

**Cách chạy:**

```bash
python examples/patrol_example.py
```

**Hành vi:**
1. Đi thẳng 5 giây
2. Quay 3 giây
3. Lặp lại
4. Khi phát hiện người → cảnh báo và theo dõi

---

### 9.3 examples/object_tracking_example.py - Theo Dõi Vật Thể

**Cách chạy:**

```bash
python examples/object_tracking_example.py
```

**Phím tắt:**

| Phím | Chức năng |
|------|-----------|
| 1-9 | Chọn class (1=person, 2=car, ...) |
| r | Reset |
| q | Thoát |

**Trackbars:**
- Steering Gain: 0-50 (×0.1)
- Target Distance: 50-300 (cm)
- Max Speed: 0-20 (×0.1 m/s)

---

## 10. Tools - Công Cụ Hỗ Trợ

### 10.1 Testing Tools

| File | Chức năng |
|------|-----------|
| `test_camera.py` | Test camera RealSense |
| `test_line_detector.py` | Test phát hiện line |
| `test_object_detector.py` | Test YOLO detection |
| `test_terrain_analyzer.py` | Test phân tích địa hình |
| `test_modules_interactive.py` | Test tất cả module với trackbar |
| `test_uart_simple.py` | Test UART cơ bản |
| `test_depth_estimator.py` | Test ước lượng depth |

### 10.2 Calibration Tools

| File | Chức năng |
|------|-----------|
| `camera_calibration.py` | Calibration camera intrinsic |
| `depth_calibration.py` | Calibration depth |
| `lane_calibration.py` | Calibration lane detection |

### 10.3 Cách Sử Dụng Test Tools

```bash
# Test camera
python tools/testing/test_camera.py

# Test line detector với trackbar
python tools/testing/test_line_detector.py

# Test object detector
python tools/testing/test_object_detector.py

# Test terrain analyzer
python tools/testing/test_terrain_analyzer.py

# Test tất cả module
python tools/testing/test_modules_interactive.py
```

---

## 11. Luồng Dữ Liệu (Data Flow)

### 11.1 Luồng Chính Khi Dò Line

```
┌─────┐   ┌──────────┐   ┌───────────┐   ┌─────────┐   ┌───────┐   ┌───────┐
│CAMERA│→ │PREPROCESS│→ │LINE DETECT│→ │CALCULATE│→ │  UART │→ │ STM32 │
│     │   │          │   │           │   │ CONTROL │   │       │   │       │
│color│   │grayscale │   │pos_error  │   │velocity │   │V, Y   │   │motors │
│depth│   │threshold │   │heading_err│   │yaw_rate │   │       │   │       │
└─────┘   │morphology│   │confidence │   │         │   │       │   │       │
          └──────────┘   └───────────┘   └─────────┘   └───────┘   └───────┘
                30 FPS            ↓              ↓           10 Hz
                              DISPLAY        TERRAIN
                              OpenCV         ANALYZER
                                               ↓
                                            HEIGHT
                                            COMMAND
```

### 11.2 Luồng Khi Theo Dõi Vật Thể

```
┌─────┐   ┌──────────┐   ┌───────────┐   ┌─────────┐   ┌───────┐
│CAMERA│→ │   YOLO   │→ │  TARGET   │→ │CALCULATE│→ │ UART  │
│     │   │          │   │ SELECTION │   │ CONTROL │   │       │
│color│   │objects   │   │           │   │         │   │       │
│depth│   │with depth│   │target_obj │   │velocity │   │V, Y   │
└─────┘   └──────────┘   └───────────┘   │yaw_rate │   └───────┘
                                          │smoothed │
                                          └─────────┘
```

---

## 12. Lưu Ý Quan Trọng

### 12.1 Về Camera

- ✅ **Phải gắn đúng góc**: 15° cúi xuống so với phương ngang
- ✅ **Độ cao camera**: 20cm so với mặt đất
- ✅ **Align depth**: Luôn sử dụng aligned depth (đã căn với color)
- ⚠️ **Warm-up**: Chờ camera ổn định trước khi sử dụng
- ⚠️ **Depth range**: Chỉ tin cậy trong 0.1m - 10m

### 12.2 Về Line Detection

- ✅ **Line đen trên nền sáng** (mặc định)
- ✅ **ROI hình thang** để phù hợp perspective
- ⚠️ **Ánh sáng**: Adaptive threshold tự động điều chỉnh
- ⚠️ **Line recovery**: Khi mất line sẽ tự quay tìm

### 12.3 Về UART

- ✅ **Baud rate**: 115200
- ✅ **Format**: ASCII, kết thúc bằng `\n`
- ✅ **Heartbeat**: Tự động gửi mỗi 500ms
- ⚠️ **Emergency stop**: Nếu mất heartbeat 3 lần → dừng

### 12.4 Về Terrain Analyzer

- ✅ **Ceiling zone**: Top 30% frame
- ✅ **Ground zone**: 55-90% frame
- ⚠️ **Độ cao gầm**:
  - Bình thường: 13cm
  - Nâng gầm: 18cm
  - Hạ gầm: 7cm

### 12.5 Về Object Detection

- ✅ **Model**: YOLOv8n (nhanh, phù hợp realtime)
- ✅ **Confidence threshold**: 0.3 (có thể điều chỉnh)
- ⚠️ **Multi-point depth**: Lấy percentile 25 cho robust

---

## 13. Câu Hỏi Thường Gặp

### Q1: Tại sao dùng RealSense mà không dùng webcam thường?

**A:** RealSense D435i cung cấp:
- Depth sensing để đo khoảng cách chính xác
- Stereo camera tích hợp
- Depth align với color
- SDK hỗ trợ tốt cho Python

### Q2: Tại sao chọn YOLOv8n?

**A:** 
- **Nano version**: Nhanh, phù hợp realtime (~30 FPS)
- **Accuracy**: Đủ tốt cho detection cơ bản
- **COCO pretrained**: 80 classes sẵn có

### Q3: Làm sao để thay đổi class detection?

**A:**
```python
# Trong config.py
DETECT_CLASSES = [0, 2, 5]  # person, car, bus
# Hoặc None để detect all
```

### Q4: Line detection không hoạt động tốt?

**A:** Kiểm tra:
1. Ánh sáng có đủ không?
2. Line có đủ contrast không?
3. Camera có đúng góc không?
4. Điều chỉnh BLACK_THRESHOLD trong config

### Q5: UART không kết nối được?

**A:** Kiểm tra:
1. Port đúng chưa (`/dev/ttyACM0` hoặc `COM3`)
2. Quyền truy cập (`sudo chmod 666 /dev/ttyACM0`)
3. Baud rate đúng (115200)
4. Dùng `--mock-uart` để test không cần phần cứng

### Q6: Làm sao để test từng module?

**A:**
```bash
python tools/testing/test_modules_interactive.py
# Chọn module bằng phím 1-5
```

### Q7: Độ cao gầm nghĩa là gì?

**A:**
- **Gầm (ground clearance)**: Khoảng cách từ thân robot đến mặt đất
- **Nâng gầm**: Robot "đứng cao hơn" để vượt chướng ngại vật
- **Hạ gầm**: Robot "cúi thấp" để qua trần/gầm cầu

### Q8: Tại sao có smoothing trong object tracking?

**A:**
- Tránh giật khi target di chuyển
- Gimbal-style tracking cần chuyển động mượt
- Low-pass filter giúp ổn định

---

## PHỤ LỤC

### A. Danh Sách Thư Viện (requirements.txt)

```
numpy>=1.21.0
opencv-python>=4.5.0
pyrealsense2>=2.50.0
pyserial>=3.5
ultralytics>=8.0.0
PyYAML>=6.0
```

### B. Cài Đặt

```bash
# Clone project
git clone <repo>

# Tạo virtual environment
python -m venv venv
source venv/bin/activate  # Linux
venv\Scripts\activate     # Windows

# Cài thư viện
pip install -r requirements.txt

# Chạy test camera
python tools/testing/test_camera.py
```

### C. Troubleshooting

| Vấn đề | Giải pháp |
|--------|-----------|
| Camera not found | Kiểm tra USB, cài RealSense SDK |
| YOLO slow | Dùng GPU (CUDA) hoặc giảm resolution |
| UART permission | `sudo chmod 666 /dev/ttyACM0` |
| Low FPS | Giảm resolution hoặc tắt visualization |

---

*Tài liệu được tạo cho dự án Autonomous Robot*  
*Liên hệ: [Project Team]*
