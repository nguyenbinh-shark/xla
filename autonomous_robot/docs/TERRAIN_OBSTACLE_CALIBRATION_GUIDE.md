# Hướng dẫn Calibration Phát hiện Vật cản Địa hình

## Tổng quan

Tool calibration này giúp điều chỉnh các thông số phát hiện vật cản trên mặt đất khi camera được gắn nghiêng. Điểm đặc biệt là sử dụng **baseline tuyến tính** thay vì baseline cố định để phù hợp với hình học camera nghiêng.

---

## 1. Vấn đề với Camera Nghiêng

### 1.1 Tại sao cần baseline tuyến tính?

Khi camera được gắn nghiêng xuống (ví dụ 15°), depth của mặt đất **thay đổi theo vị trí y trong frame**:

```
Camera nhìn nghiêng 15° so với phương ngang:

    ════════════════════════════════════════  Horizon
              ╲
               ╲  15°
                ╲
                 ╲ ← Tia nhìn center
                  ╲
    ┌──────────────╲───────────────┐ Frame
    │   TOP         ╲              │ ← Depth LỚN (xa)
    │                ╲             │
    │                 ╲            │
    │   CENTER         ╲           │
    │                   ╲          │
    │                    ╲         │
    │   BOTTOM            ╲        │ ← Depth NHỎ (gần)
    └──────────────────────╲───────┘
    ════════════════════════╲══════════════  Mặt đất
```

**Kết quả**: 
- Phía **TOP của frame** → nhìn xa → depth **LỚN**
- Phía **BOTTOM của frame** → nhìn gần → depth **NHỎ**
- Depth thay đổi **tuyến tính** theo y

### 1.2 Vấn đề với baseline cố định (cách cũ)

```python
# Cách cũ - SAI với camera nghiêng:
baseline = np.median(all_depths)  # Lấy 1 giá trị cho toàn bộ ROI

# So sánh:
obstacle = depth < (baseline - threshold)
# → Phía TOP: false positive (depth lớn hơn baseline)
# → Phía BOTTOM: false negative (depth nhỏ hơn baseline là bình thường)
```

### 1.3 Giải pháp: Baseline tuyến tính (cách mới)

```python
# Cách mới - ĐÚNG:
# 1. Tính median depth cho TỪNG HÀNG
for i in range(roi_height):
    row_baseline[i] = np.median(row[i])

# 2. Fit đường thẳng: depth = slope * y + intercept
fit = np.polyfit(y_indices, row_baseline, 1)
baseline_line = np.polyval(fit, all_y)

# 3. So sánh từng điểm với baseline TƯƠNG ỨNG tại vị trí y đó
obstacle[i, j] = depth[i, j] < (baseline_line[i] - threshold)
```

---

## 2. Giải thích các Đồ thị

### 2.1 Ground ROI Depth Heatmap (Góc trái trên)

```
┌─────────────────────────────┐
│  ████████████████████████   │ ← Màu đỏ = depth lớn (xa)
│  ████████████████████████   │
│  ████████████████████████   │
│  ████████████████████████   │ ← Gradient từ đỏ → xanh
│  ████████████████████████   │
│  ████████████████████████   │ ← Màu xanh = depth nhỏ (gần)
└─────────────────────────────┘
   X (pixel) →
```

**Cách đọc**:
- Màu **đỏ/vàng** = xa camera (depth lớn)
- Màu **xanh/tím** = gần camera (depth nhỏ)
- Nếu mặt đất phẳng: gradient màu đều từ trên xuống
- Nếu có vật cản: **vùng xanh đậm bất thường** giữa vùng đỏ

### 2.2 Baseline Fit (Góc phải trên)

```
Depth (m)
    │
2.5 │  ○ ○ ○                    ← Row Median (điểm xanh)
    │      ○ ○ ──────────────── ← Baseline fit (đường đỏ)
2.0 │          ○ ○ ○
    │              ○ ○
1.5 │                  ○ ○ ○
    │- - - - - - - - - - - - -  ← Threshold (đường xanh lá)
1.0 │                      ○ ○
    │
    └────────────────────────── Y in ROI
    0                        168
```

**Cách đọc**:
- **Điểm xanh**: Median depth thực tế của từng hàng
- **Đường đỏ**: Baseline tuyến tính đã fit
- **Đường xanh lá đứt**: Ngưỡng threshold (baseline - obstacle_threshold)
- **Slope**: Cho biết depth thay đổi bao nhiêu cm mỗi hàng pixel

**Slope bình thường**: -0.005 đến -0.02 cm/row (âm vì depth giảm khi y tăng)

### 2.3 Obstacle Mask (Góc trái dưới)

```
┌─────────────────────────────┐
│  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   │ ← Xám = depth bình thường
│  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   │
│  ▒▒▒▒▒▒████████▒▒▒▒▒▒▒▒▒   │ ← ĐỎ = VẬT CẢN
│  ▒▒▒▒▒▒████████▒▒▒▒▒▒▒▒▒   │
│  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   │
│  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   │
└─────────────────────────────┘
```

**Cách đọc**:
- **Xám**: Vùng mặt đất bình thường
- **Đỏ**: Vật cản được phát hiện (depth nhỏ hơn baseline - threshold)
- Số pixel đỏ và chiều cao ước tính hiển thị trong title

### 2.4 Depth Profile (Góc phải dưới)

```
Depth (m)
    │
2.0 │  ○ ○ ○ ─────────────────  ← Baseline
    │      ○ ○
1.5 │- - - - - - - - - - - - -  ← Threshold
    │          ○ ○
1.0 │              ╳ ╳         ← Vật cản (X đỏ)
    │                  ○ ○ ○
    └────────────────────────── Y in ROI
```

**Cách đọc**:
- **Điểm xanh**: Depth thực tế tại cột được chọn
- **Đường đỏ**: Baseline
- **X đỏ**: Điểm được đánh dấu là vật cản
- Click vào Main View để chọn cột khác nhau

---

## 3. Các Thông số Calibration

### 3.1 Camera Setup

| Thông số | Mô tả | Giá trị mặc định | Cách điều chỉnh |
|----------|-------|------------------|-----------------|
| `camera_tilt_angle` | Góc nghiêng camera so với phương ngang | 15° | Đo bằng thước đo góc hoặc ứng dụng điện thoại |
| `camera_height` | Chiều cao camera so với mặt đất | 0.20m | Đo trực tiếp |

### 3.2 Detection Zones

| Thông số | Mô tả | Giá trị mặc định |
|----------|-------|------------------|
| `ground_zone_top` | Điểm bắt đầu vùng scan (% frame height) | 0.55 (55%) |
| `ground_zone_bottom` | Điểm kết thúc vùng scan | 0.90 (90%) |

**Cách chọn zone**:
```
Frame 480px height:
┌────────────────┐  0%
│                │
│   (Bỏ qua)     │
│                │  
├────────────────┤  55% = y_start
│                │
│   GROUND ROI   │  ← Vùng scan vật cản
│                │
├────────────────┤  90% = y_end
│   (Bỏ qua)     │
└────────────────┘  100%
```

### 3.3 Obstacle Detection

| Thông số | Mô tả | Giá trị mặc định | Ảnh hưởng |
|----------|-------|------------------|-----------|
| `obstacle_threshold` | Chênh lệch depth để coi là vật cản | 0.08m (8cm) | Nhỏ = nhạy hơn, nhiều false positive |
| `max_step_height` | Chiều cao tối đa có thể bước qua | 0.05m (5cm) | Quyết định RAISE vs STOP |

### 3.4 Depth Filtering

| Thông số | Mô tả | Giá trị mặc định |
|----------|-------|------------------|
| `depth_min_valid` | Depth tối thiểu hợp lệ | 0.1m |
| `depth_max_valid` | Depth tối đa hợp lệ | 5.0m |

---

## 4. Quy trình Calibration

### Bước 1: Setup môi trường test

1. Đặt robot trên mặt đất phẳng, không có vật cản
2. Chạy tool: `python tools/calibration/terrain_obstacle_calibration.py`
3. Kiểm tra đồ thị Baseline Fit - nên thấy đường thẳng đều

### Bước 2: Điều chỉnh Ground Zone

1. Quan sát Main View - vùng xanh lá là ground ROI
2. Điều chỉnh `Ground Top (%)` và `Ground Bottom (%)` sao cho:
   - ROI chỉ chứa mặt đất phía trước robot
   - Không bao gồm chân robot hoặc vật thể cố định

### Bước 3: Kiểm tra Baseline

1. Nhìn đồ thị Baseline Fit
2. Điểm xanh (row median) nên nằm gần đường đỏ (fit)
3. Nếu slope > 0 (dương): camera nghiêng NGƯỢC → kiểm tra lại góc camera

### Bước 4: Điều chỉnh Threshold

1. Đặt vật cản nhỏ (5-10cm) trước robot
2. Điều chỉnh `Obstacle Thresh (cm)`:
   - Tăng nếu có quá nhiều false positive
   - Giảm nếu không phát hiện được vật cản
3. Kiểm tra Obstacle Mask - chỉ vật cản được tô đỏ

### Bước 5: Lưu config

1. Nhấn **S** để save config
2. Config được lưu tại: `data/calibration/terrain_config.json`

---

## 5. Troubleshooting

### Vấn đề: Baseline Fit không chính xác

**Triệu chứng**: Điểm xanh phân tán, không theo đường thẳng

**Nguyên nhân**:
- Mặt đất không phẳng
- Có vật cản trong vùng ROI
- Depth camera bị nhiễu

**Giải pháp**:
- Đảm bảo mặt đất phẳng khi calibrate
- Thu hẹp Ground Zone
- Tăng `smoothing_window`

### Vấn đề: False Positive quá nhiều

**Triệu chứng**: Mặt đất bình thường bị đánh dấu là vật cản

**Nguyên nhân**:
- `obstacle_threshold` quá nhỏ
- Baseline fit sai

**Giải pháp**:
- Tăng `obstacle_threshold` (8-15cm)
- Recalibrate baseline

### Vấn đề: Không phát hiện vật cản

**Triệu chứng**: Vật cản rõ ràng nhưng không được phát hiện

**Nguyên nhân**:
- `obstacle_threshold` quá lớn
- Vật cản nằm ngoài `depth_max_valid`
- Vật cản quá nhỏ

**Giải pháp**:
- Giảm `obstacle_threshold` (5-8cm)
- Kiểm tra khoảng cách vật cản

---

## 6. Công thức Tính toán

### 6.1 Ước tính chiều cao vật cản

```python
# Chênh lệch depth
depth_diff = baseline_at_obstacle - obstacle_depth

# Góc trung bình của tia nhìn đến vùng ground
avg_angle = camera_tilt + camera_vfov * 0.25  # ≈ 29.5° với tilt=15°, vfov=58°

# Chiều cao = depth_diff × sin(angle)
height = depth_diff × sin(avg_angle)
```

**Ví dụ**:
- depth_diff = 0.15m
- avg_angle = 29.5°
- height = 0.15 × sin(29.5°) = 0.15 × 0.49 ≈ **0.074m = 7.4cm**

### 6.2 Baseline Slope

```python
# Fit tuyến tính: depth = slope × y + intercept
slope = (depth_bottom - depth_top) / (y_bottom - y_top)

# Đơn vị: m/pixel hoặc cm/row
```

**Giá trị điển hình**: -0.005 đến -0.02 cm/row

---

## 7. Phím tắt

| Phím | Chức năng |
|------|-----------|
| Q | Thoát |
| S | Save config |
| L | Load config |
| R | Reset về mặc định |
| P / SPACE | Pause/Resume |
| Click | Chọn cột xem depth profile |

---

## 8. File Config

Config được lưu tại: `data/calibration/terrain_config.json`

```json
{
  "camera_setup": {
    "camera_height": 0.20,
    "camera_tilt_angle": 15.0,
    "camera_vfov": 58.0
  },
  "detection_zones": {
    "ground_zone_top": 0.55,
    "ground_zone_bottom": 0.90
  },
  "ground_obstacle_detection": {
    "obstacle_threshold": 0.08,
    "max_step_height": 0.05
  },
  "processing": {
    "depth_min_valid": 0.1,
    "depth_max_valid": 5.0,
    "smoothing_window": 5
  }
}
```
