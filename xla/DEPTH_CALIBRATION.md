# Depth Calibration

## Mục đích

Hiệu chỉnh độ chính xác của depth camera bằng cách so sánh giá trị depth đo được với khoảng cách thực tế. Tạo hệ số hiệu chỉnh để depth map chính xác hơn.

## Khi nào cần calibrate?

- Lần đầu sử dụng depth camera
- Khi phát hiện depth không khớp với thực tế
- Sau khi thay đổi cài đặt camera (resolution, fps)
- Định kỳ kiểm tra độ chính xác

## Cách sử dụng

```bash
python tools/calibration/depth_calibration.py
```

### Chuẩn bị

1. **Thước đo**: Thước dây hoặc laser đo khoảng cách
2. **Vật thể phẳng**: Tường, hộp, hoặc bề mặt phẳng
3. **Đánh dấu khoảng cách**: Đánh dấu các vị trí 0.5m, 1.0m, 1.5m, 2.0m

### Quy trình

1. Đặt vật thể phẳng tại khoảng cách đã biết (VD: 1.0m)
2. **Click** vào vật thể trên màn hình để đo depth
3. Nhấn **A** để thêm điểm calibration
4. Nhập khoảng cách thực tế (VD: 100 cho 100cm)
5. Lặp lại ở 3-5 khoảng cách khác nhau
6. Nhấn **S** để lưu calibration

### Phím tắt

| Phím | Chức năng |
|------|-----------|
| Click chuột | Chọn điểm đo depth |
| A | Thêm điểm calibration |
| C | Xóa tất cả điểm |
| S | Lưu calibration |
| L | Load calibration có sẵn |
| Q | Thoát |

## Thông số

### Correction Factor

```
depth_corrected = depth_measured × correction_factor + depth_offset
```

| Thông số | Ý nghĩa | Giá trị điển hình |
|----------|---------|-------------------|
| `correction_factor` | Hệ số nhân | 0.95 - 1.05 |
| `depth_offset` | Độ lệch cố định | -0.05 đến +0.05 m |

### Ảnh hưởng của Correction Factor

```
factor > 1.0 → Camera đo ngắn hơn thực tế
              → Nhân lên để đúng

factor < 1.0 → Camera đo xa hơn thực tế  
              → Giảm xuống để đúng
```

**Ví dụ:**
```
Camera đo: 0.95m
Thực tế:   1.00m
→ factor = 1.00 / 0.95 = 1.053
```

### Ảnh hưởng của Depth Offset

```
offset > 0 → Camera đo gần hơn thực tế một lượng cố định
offset < 0 → Camera đo xa hơn thực tế một lượng cố định
```

**Khi nào cần offset?**
- Khi error không tỷ lệ với khoảng cách
- Lỗi hệ thống cố định trong hardware

## Cách tính Calibration

### Với 1 điểm

```python
correction_factor = actual_distance / measured_distance
depth_offset = 0
```

### Với nhiều điểm (Linear Regression)

```python
# actual = factor × measured + offset
# Sử dụng least squares fit

import numpy as np
measured = [0.95, 1.45, 1.93]  # Depth đo được
actual = [1.00, 1.50, 2.00]    # Khoảng cách thực

A = np.vstack([measured, np.ones(len(measured))]).T
factor, offset = np.linalg.lstsq(A, actual)[0]
```

## File Output

`data/calibration/depth_calibration.json`:

```json
{
  "correction_factor": 1.032,
  "depth_offset": -0.015,
  "calibration_points": [
    [0.48, 0.50],
    [0.97, 1.00],
    [1.46, 1.50],
    [1.94, 2.00]
  ],
  "timestamp": "2024-01-15T11:00:00"
}
```

## Ảnh hưởng đến hệ thống

### Terrain Analyzer

```
Depth sai → Ước tính chiều cao vật cản sai
         → Robot có thể va vào vật cản
         → Hoặc dừng không cần thiết
```

### Lane Detection

```
Depth sai → Tính toán khoảng cách lane sai
         → Điều khiển steering không chính xác
```

### Object Tracking

```
Depth sai → Khoảng cách đến object sai
         → Quyết định dừng/tránh không đúng
```

## Tips

1. **Calibrate ở nhiều khoảng cách**: 0.5m, 1.0m, 1.5m, 2.0m
2. **Dùng bề mặt phẳng vuông góc**: Giảm sai số do góc
3. **Đo chính xác**: Dùng laser measure nếu có
4. **Tránh bề mặt phản quang**: Kính, kim loại bóng
5. **Kiểm tra nhiều vùng ảnh**: Center, góc

## Troubleshooting

| Vấn đề | Nguyên nhân | Giải pháp |
|--------|-------------|-----------|
| Factor thay đổi theo khoảng cách | Nonlinear distortion | Calibrate ở nhiều khoảng cách hơn |
| Depth = 0 hoặc NaN | Vật quá gần/xa | Đảm bảo trong range 0.3-5m |
| Depth nhảy lung tung | Bề mặt phản quang | Đổi vật thể calibrate |
| Factor >> 1.1 hoặc << 0.9 | Camera có vấn đề | Kiểm tra firmware/settings |

## Validation

Sau khi calibrate, kiểm tra:

```python
# Test ở khoảng cách mới (VD: 1.25m)
corrected = measured * factor + offset
error = abs(corrected - actual)

# Error chấp nhận được: < 2cm ở khoảng cách 1m
```
