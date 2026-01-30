# Terrain Obstacle Calibration

## Mục đích

Tinh chỉnh các thông số phát hiện vật cản trên địa hình sử dụng depth camera:
- Phát hiện vật cản trên mặt đất (obstacles)
- Phát hiện trần/giới hạn trên (ceiling)
- Xác định khi nào cần nâng/hạ gầm robot

## Nguyên lý hoạt động

### Ground Obstacle Detection

```
Camera nhìn xuống mặt đất ở góc nghiêng.
Mặt đất phẳng → Depth tăng tuyến tính theo y (từ trên xuống)
Vật cản → Depth nhỏ hơn đường baseline → Detected!
```

```
     Camera
        ╲ (tilt angle)
         ╲
          ╲___________baseline (mặt đất)
           ╲         /
            ╲_______/  ← obstacle (depth nhỏ hơn)
```

### Ceiling Detection

```
Vùng trên của frame → Nếu có depth gần = ceiling detected
```

## Cách sử dụng

```bash
python tools/calibration/terrain_obstacle_calibration.py
```

### Giao diện

Tool hiển thị 4 đồ thị:
1. **Depth Heatmap**: Hiển thị depth trong ROI
2. **Baseline Fit**: Đường baseline tuyến tính vs depth thực
3. **Obstacle Mask**: Pixel được detect là vật cản (đỏ)
4. **Depth Profile**: Profile depth theo cột được chọn

### Phím tắt

| Phím | Chức năng |
|------|-----------|
| Q | Thoát |
| S | Lưu config |
| L | Load config |
| R | Reset về mặc định |
| P / SPACE | Pause/Resume |
| Click | Chọn cột để xem depth profile |

## Thông số

### 1. Camera Setup

| Thông số | Ý nghĩa | Range | Mặc định |
|----------|---------|-------|----------|
| `camera_tilt_angle` | Góc nghiêng camera (độ) | 0-45 | 15° |
| `camera_height` | Chiều cao camera (m) | 0.1-0.5 | 0.20m |
| `camera_vfov` | Vertical FOV (độ) | - | 58° |

**Ảnh hưởng camera_tilt_angle:**

```
Góc nhỏ (0-10°):
→ Camera nhìn ngang
→ Thấy xa hơn
→ Khó phát hiện vật cản thấp

Góc lớn (20-45°):
→ Camera cúi xuống
→ Thấy gần hơn
→ Phát hiện vật cản tốt hơn
```

**Cách đo:**
```
1. Đặt robot trên mặt phẳng
2. Nhìn vào depth map
3. Điều chỉnh cho đến khi baseline fit tốt
```

### 2. Detection Zones

| Thông số | Ý nghĩa | Range | Mặc định |
|----------|---------|-------|----------|
| `ground_zone_top` | Đỉnh vùng ground (%) | 0-100% | 55% |
| `ground_zone_bottom` | Đáy vùng ground (%) | 0-100% | 90% |
| `ceiling_zone_top` | Đỉnh vùng ceiling (%) | 0-100% | 0% |
| `ceiling_zone_bottom` | Đáy vùng ceiling (%) | 0-100% | 30% |

```
Frame:
  0% ┌──────────────────┐
     │ CEILING ZONE     │  ← ceiling_zone: 0-30%
 30% ├──────────────────┤
     │ (không xử lý)    │
 55% ├──────────────────┤
     │ GROUND ZONE      │  ← ground_zone: 55-90%
 90% ├──────────────────┤
     │ (quá gần)        │
100% └──────────────────┘
```

**Ảnh hưởng ground_zone:**

```
ground_zone_top thấp (40%):
→ Phát hiện vật cản xa hơn
→ Nhưng depth kém chính xác ở xa

ground_zone_bottom cao (95%):
→ Phát hiện gần robot
→ Phản ứng nhanh hơn
→ Nhưng có thể detect thân robot
```

### 3. Obstacle Detection

| Thông số | Ý nghĩa | Range | Mặc định |
|----------|---------|-------|----------|
| `obstacle_threshold` | Ngưỡng phát hiện (m) | 0.02-0.20 | 0.08m |
| `max_step_height` | Chiều cao bước qua được (m) | 0.02-0.10 | 0.05m |

**obstacle_threshold:**

```
depth_at_pixel < baseline - obstacle_threshold
        ↓
   Vật cản detected!
```

```
Threshold nhỏ (2-4cm):
→ Nhạy với vật cản thấp
→ False positive do nhiễu depth

Threshold lớn (10-15cm):
→ Chỉ detect vật cản rõ ràng
→ Có thể miss vật cản thấp
```

**max_step_height:**

```
obstacle_height < max_step_height → Robot có thể bước qua
obstacle_height > max_step_height → Cần tránh hoặc nâng gầm
```

### 4. Ceiling Detection

| Thông số | Ý nghĩa | Range | Mặc định |
|----------|---------|-------|----------|
| `ceiling_min_clearance` | Khoảng cách tối thiểu (m) | 0.3-1.0 | 0.50m |
| `ceiling_warning_distance` | Khoảng cách cảnh báo (m) | 0.5-2.0 | 1.50m |

```
depth_ceiling < ceiling_min_clearance → STOP! Không đủ không gian
depth_ceiling < ceiling_warning_distance → WARNING! Cân nhắc hạ gầm
```

### 5. Depth Validity

| Thông số | Ý nghĩa | Range | Mặc định |
|----------|---------|-------|----------|
| `depth_min_valid` | Depth tối thiểu (m) | 0.1-0.5 | 0.10m |
| `depth_max_valid` | Depth tối đa (m) | 2.0-10.0 | 5.0m |

```
Depth ngoài range [min, max] → Bỏ qua (invalid)
```

**Lý do:**
```
depth < 0.1m: Quá gần, nhiễu nhiều
depth > 5.0m: Quá xa, không chính xác
```

## File Output

`data/calibration/terrain_config.json`:

```json
{
  "robot_dimensions": {
    "robot_height": 0.3,
    "min_ground_clearance": 0.07,
    "max_ground_clearance": 0.18,
    "normal_ground_clearance": 0.08
  },
  "camera_setup": {
    "camera_height": 0.20,
    "camera_tilt_angle": 15.0,
    "camera_vfov": 58.0
  },
  "detection_zones": {
    "ceiling_zone_top": 0.0,
    "ceiling_zone_bottom": 0.30,
    "ground_zone_top": 0.55,
    "ground_zone_bottom": 0.90
  },
  "ceiling_detection": {
    "ceiling_min_clearance": 0.50,
    "ceiling_warning_distance": 1.50
  },
  "ground_obstacle_detection": {
    "ground_baseline_distance": 1.0,
    "obstacle_threshold": 0.08,
    "max_step_height": 0.05
  },
  "processing": {
    "depth_min_valid": 0.10,
    "depth_max_valid": 5.0,
    "smoothing_window": 5
  }
}
```

## Baseline Fit

### Thuật toán

```python
# 1. Cho mỗi hàng y trong ground zone:
row_baseline[y] = median(valid_depths_in_row)

# 2. Fit tuyến tính:
baseline = slope * y + intercept

# 3. Phát hiện vật cản:
obstacle_mask = depth < (baseline - threshold)
```

### Đọc đồ thị Baseline Fit

```
Depth (m)
   │
   │    ● ● ●           ← Row median (blue dots)
   │   ●     ● ●
   │  ────────────      ← Baseline fit (red line)
   │  - - - - - - -     ← Threshold (green dashed)
   │       ×            ← Obstacle detected!
   └─────────────── Y in ROI
```

- **Blue dots**: Median depth của mỗi hàng
- **Red line**: Đường fit tuyến tính
- **Green dashed**: Ngưỡng (baseline - threshold)
- **Below green**: Là vật cản

### Slope ý nghĩa

```
slope = cm depth thay đổi / mỗi row pixel

slope > 0: Depth tăng theo y (bình thường khi camera cúi)
slope ≈ 0: Camera nhìn ngang hoặc mặt đất dốc
slope < 0: Camera ngửa lên (không bình thường)
```

**Giá trị điển hình:** 0.002 - 0.010 cm/row

## Workflow Điều chỉnh

```
1. Đặt robot trên mặt phẳng (không có vật cản)
   ↓
2. Điều chỉnh camera_tilt_angle
   → Cho đến khi baseline fit tốt (dots nằm trên line)
   ↓
3. Điều chỉnh ground_zone
   → Bao phủ vùng mặt đất phía trước robot
   ↓
4. Đặt vật cản test (hộp nhỏ ~5cm)
   ↓
5. Điều chỉnh obstacle_threshold
   → Vật cản được detect (đỏ) nhưng nhiễu thì không
   ↓
6. Test ceiling detection
   → Đưa vật che phía trên camera
   ↓
7. Điều chỉnh ceiling thresholds
   ↓
8. Save config (S)
```

## Troubleshooting

| Vấn đề | Nguyên nhân | Giải pháp |
|--------|-------------|-----------|
| Baseline fit lệch | Camera angle sai | Điều chỉnh camera_tilt_angle |
| Nhiều false positive | Threshold quá nhỏ | Tăng obstacle_threshold |
| Miss vật cản thấp | Threshold quá lớn | Giảm obstacle_threshold |
| Không có valid depth | min/max sai | Kiểm tra depth_min/max_valid |
| Detect thân robot | ground_zone_bottom quá cao | Giảm ground_zone_bottom |
| Miss vật cản xa | ground_zone_top quá cao | Giảm ground_zone_top |

## Tips

1. **Calibrate trên mặt phẳng**: Đảm bảo baseline chính xác trước
2. **Test với vật cản thực**: Sử dụng vật có chiều cao biết trước
3. **Kiểm tra nhiều khoảng cách**: 0.5m, 1m, 1.5m
4. **Lưu config thường xuyên**: Mỗi khi tìm được thông số tốt
5. **Quan sát slope**: Slope ổn định = calibration tốt

## Kết hợp với hệ thống

### ClearanceAction

```python
if ceiling_detected and ceiling_distance < ceiling_min_clearance:
    action = ClearanceAction.STOP
    
elif ceiling_distance < ceiling_warning_distance:
    action = ClearanceAction.LOWER  # Hạ gầm
    
elif obstacle_detected and obstacle_height > max_step_height:
    action = ClearanceAction.RAISE  # Nâng gầm
    
else:
    action = ClearanceAction.NORMAL
```
