# Lane Calibration

## Mục đích

Tinh chỉnh các thông số cho thuật toán phát hiện lane (đường line) bao gồm:
- Vùng quan tâm (ROI)
- Ngưỡng phát hiện đường đen
- Tham số phát hiện cạnh (Canny)
- Tham số phát hiện đường thẳng (Hough)

## Khi nào cần calibrate?

- Thay đổi góc gắn camera
- Thay đổi chiều cao camera
- Thay đổi môi trường (indoor/outdoor)
- Thay đổi loại đường line (màu, độ rộng)
- Điều kiện ánh sáng thay đổi nhiều

## Cách sử dụng

```bash
python tools/calibration/lane_calibration.py
```

### Quy trình

1. Đặt robot trước đường line
2. Điều chỉnh các trackbar cho đến khi lane được detect tốt
3. Nhấn **S** để lưu thông số
4. Test với nhiều góc nhìn khác nhau

### Phím tắt

| Phím | Chức năng |
|------|-----------|
| S | Lưu thông số |
| C | Capture ảnh |
| R | Reset về mặc định |
| Q | Thoát |

## Thông số

### 1. ROI (Region of Interest)

ROI là vùng hình thang xác định phần ảnh được xử lý.

```
         roi_top_left_x     roi_top_right_x
                ╲_______________╱  ← roi_top_y
                 \             /
                  \           /
                   \_________/     ← roi_bottom_y
         roi_bottom_left_x   roi_bottom_right_x
```

| Thông số | Ý nghĩa | Range | Ảnh hưởng |
|----------|---------|-------|-----------|
| `roi_top_y` | Đỉnh ROI (% chiều cao) | 0-100% | Nhỏ = nhìn xa hơn |
| `roi_bottom_y` | Đáy ROI (% chiều cao) | 0-100% | Lớn = gần robot hơn |
| `roi_top_left_x` | Góc trên trái (% chiều rộng) | 0-100% | Thu hẹp = bỏ góc |
| `roi_top_right_x` | Góc trên phải (% chiều rộng) | 0-100% | Thu hẹp = bỏ góc |
| `roi_bottom_left_x` | Góc dưới trái (% chiều rộng) | 0-100% | Thường = 0% |
| `roi_bottom_right_x` | Góc dưới phải (% chiều rộng) | 0-100% | Thường = 100% |

**Ảnh hưởng:**

```
ROI quá nhỏ → Bỏ lỡ lane ở xa
            → Phản ứng chậm với cua

ROI quá lớn → Nhiễu từ môi trường
            → False positive nhiều
```

**Giá trị khuyến nghị:**
```
roi_top_y: 50-60%        (nhìn ~1-2m phía trước)
roi_bottom_y: 95-100%    (gần robot)
roi_top_left_x: 30-40%   (thu hẹp góc xa)
roi_top_right_x: 60-70%
roi_bottom_left_x: 0-5%  (rộng ở gần)
roi_bottom_right_x: 95-100%
```

### 2. Threshold (Ngưỡng phát hiện đường đen)

| Thông số | Ý nghĩa | Range | Ảnh hưởng |
|----------|---------|-------|-----------|
| `black_threshold` | Ngưỡng grayscale | 0-255 | Pixel < threshold = đường |

```
                    Threshold
Grayscale:    0 ████████████|████████████ 255
              ↑ Đen/Đường   ↑   Sáng/Nền
```

**Ảnh hưởng:**

```
Threshold thấp (30-50):
→ Chỉ detect đường đen rất đậm
→ Ít false positive
→ Có thể miss đường mờ

Threshold cao (100-150):
→ Detect cả đường mờ
→ Nhiều false positive
→ Nhạy với bóng đổ
```

**Điều chỉnh theo môi trường:**
```
Indoor sáng:    80-100
Indoor tối:     50-80
Outdoor nắng:   60-90
Outdoor râm:    80-120
```

### 3. Morphological Kernel

| Thông số | Ý nghĩa | Range | Ảnh hưởng |
|----------|---------|-------|-----------|
| `morph_kernel` | Kích thước kernel | 1-15 (lẻ) | Xử lý nhiễu |

**Hoạt động:**
```
Close (đóng): Lấp đầy lỗ hổng trong đường
Open (mở):    Loại bỏ nhiễu nhỏ

     Input         Close         Open
    ■■ ■■         ■■■■■        ■■■■■
    ■■■■■   →     ■■■■■   →    ■■■■■
    ■■ ■■         ■■■■■        ■■■■■
      ■                         
    (nhiễu)                   (sạch)
```

**Ảnh hưởng:**

```
Kernel nhỏ (1-3):
→ Giữ nguyên chi tiết
→ Vẫn còn nhiễu

Kernel lớn (7-15):
→ Loại nhiễu tốt
→ Làm mờ đường hẹp
→ Có thể mất đường
```

**Giá trị khuyến nghị:** 3-5

### 4. Canny Edge Detection

| Thông số | Ý nghĩa | Range | Ảnh hưởng |
|----------|---------|-------|-----------|
| `canny_low` | Ngưỡng dưới | 0-255 | Loại cạnh yếu |
| `canny_high` | Ngưỡng trên | 0-255 | Giữ cạnh mạnh |

**Thuật toán:**
```
Gradient > canny_high → Chắc chắn là cạnh
Gradient < canny_low  → Không phải cạnh
Ở giữa → Cạnh nếu nối với cạnh mạnh
```

**Tỷ lệ khuyến nghị:** `canny_high = 2-3 × canny_low`

**Ảnh hưởng:**

```
canny_low/high thấp (20/60):
→ Nhiều cạnh được detect
→ Bao gồm cả nhiễu

canny_low/high cao (80/200):
→ Chỉ cạnh rõ ràng
→ Có thể miss đường mờ
```

**Giá trị khuyến nghị:**
```
canny_low:  50-80
canny_high: 100-200
```

### 5. Hough Transform

| Thông số | Ý nghĩa | Range | Ảnh hưởng |
|----------|---------|-------|-----------|
| `hough_threshold` | Số vote tối thiểu | 1-100 | Xác định đường thẳng |
| `hough_min_length` | Độ dài tối thiểu (px) | 1-200 | Loại đoạn ngắn |
| `hough_max_gap` | Khoảng cách nối (px) | 1-100 | Nối đoạn đứt |

**Hough Transform:**
```
Mỗi điểm cạnh "vote" cho các đường thẳng đi qua
Đường có nhiều vote nhất = đường thực
```

**Ảnh hưởng:**

```
hough_threshold thấp:
→ Detect nhiều đường
→ Bao gồm false positive

hough_threshold cao:
→ Chỉ đường rõ ràng
→ Có thể miss đường
```

```
hough_min_length nhỏ:
→ Giữ đoạn ngắn (nhiễu)

hough_min_length lớn:
→ Loại đoạn ngắn
→ Có thể miss lane xa
```

```
hough_max_gap nhỏ:
→ Đường đứt = nhiều đoạn

hough_max_gap lớn:
→ Nối qua khoảng trống
→ Có thể nối sai 2 lane
```

**Giá trị khuyến nghị:**
```
hough_threshold: 20-40
hough_min_length: 30-50
hough_max_gap: 10-30
```

## File Output

`data/calibration/lane_params.json`:

```json
{
  "roi": {
    "top_y": 0.55,
    "bottom_y": 0.95,
    "top_left_x": 0.35,
    "top_right_x": 0.65,
    "bottom_left_x": 0.0,
    "bottom_right_x": 1.0
  },
  "threshold": {
    "black_threshold": 80,
    "morph_kernel": 5
  },
  "canny": {
    "low": 50,
    "high": 150
  },
  "hough": {
    "threshold": 30,
    "min_length": 40,
    "max_gap": 20
  }
}
```

## Workflow Điều chỉnh

```
1. ROI trước
   ↓
2. Threshold (xem binary image)
   ↓
3. Morph kernel (xem morphed image)
   ↓
4. Canny (xem edges image)
   ↓
5. Hough (xem lines overlay)
```

## Troubleshooting

| Vấn đề | Nguyên nhân | Giải pháp |
|--------|-------------|-----------|
| Không detect được lane | Threshold sai | Tăng threshold nếu nền sáng |
| Detect quá nhiều line | Threshold quá cao | Giảm threshold |
| Lane bị đứt đoạn | max_gap nhỏ | Tăng hough_max_gap |
| Nhiều false positive | ROI quá lớn | Thu hẹp ROI |
| Miss lane ở xa | ROI bottom quá cao | Giảm roi_top_y |
| Đường cong bị bỏ qua | min_length quá lớn | Giảm hough_min_length |

## Tips

1. **Test nhiều điều kiện ánh sáng**: Sáng, tối, bóng đổ
2. **Test nhiều góc**: Thẳng, cua trái, cua phải
3. **Bắt đầu từ ROI**: ROI đúng giảm 80% vấn đề
4. **Quan sát từng bước**: Binary → Morphed → Edges → Lines
5. **Ưu tiên ổn định**: Chấp nhận miss đôi khi thay vì false positive nhiều
