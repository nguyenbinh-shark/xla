# Nguyên Lý Tính Toán Hệ Thống Phát Hiện Đường Line
## Simple Line Detector - Centerline Method

**Tác giả:** Autonomous Robot Project  
**Phiên bản:** 1.0  
**Ngày:** Tháng 1, 2026

---

## Mục Lục

1. [Tổng Quan Hệ Thống](#1-tổng-quan-hệ-thống)
2. [Tiền Xử Lý Ảnh](#2-tiền-xử-lý-ảnh)
3. [Phương Pháp Horizontal Slicing](#3-phương-pháp-horizontal-slicing)
4. [Tính Toán Position Error](#4-tính-toán-position-error)
5. [Tính Toán Heading Error](#5-tính-toán-heading-error)
6. [Chế Độ Khôi Phục Line](#6-chế-độ-khôi-phục-line)
7. [Bộ Lọc Làm Mịn Thích Ứng](#7-bộ-lọc-làm-mịn-thích-ứng)
8. [Ví Dụ Tính Toán Cụ Thể](#8-ví-dụ-tính-toán-cụ-thể)
9. [Tích Hợp Với Bộ Điều Khiển PID](#9-tích-hợp-với-bộ-điều-khiển-pid)
10. [Giới Hạn và Cải Tiến](#10-giới-hạn-và-cải-tiến)

---

## 1. Tổng Quan Hệ Thống

### 1.1 Mục Tiêu

Hệ thống Simple Line Detector được thiết kế để:

1. **Phát hiện đường line đen** trên nền sáng (hoặc ngược lại)
2. **Tính toán Position Error** - Độ lệch ngang của robot so với line
3. **Tính toán Heading Error** - Góc lệch hướng của robot so với line
4. **Khôi phục tự động** khi mất line (line recovery mode)

### 1.2 Đầu Vào/Đầu Ra

```
┌──────────────────────────────────────────────────────────────┐
│                   SIMPLE LINE DETECTOR                        │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  INPUT:                         OUTPUT:                      │
│  ┌─────────────────┐           ┌─────────────────────────┐  │
│  │ Color Frame     │           │ line_detected: True     │  │
│  │ (640×480, BGR)  │    →      │ position_error: +0.15   │  │
│  │                 │           │ heading_error: -5.2°    │  │
│  │                 │           │ confidence: 0.8         │  │
│  └─────────────────┘           │ centerline_points: [...] │  │
│                                └─────────────────────────┘  │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### 1.3 Ý Nghĩa Các Đầu Ra

| Đầu ra | Phạm vi | Ý nghĩa |
|--------|---------|---------|
| `position_error` | [-1, +1] | -1 = line ở trái, +1 = line ở phải |
| `heading_error` | [-90°, +90°] | Dương = line nghiêng sang phải |
| `confidence` | [0, 1] | Tỷ lệ điểm centerline phát hiện được |
| `line_detected` | True/False | Có phát hiện được line không |

### 1.4 Sơ Đồ Pipeline

```
┌─────────┐    ┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│  Frame  │ →  │ Preprocess  │ →  │ Find Center  │ →  │ Calculate   │
│  (BGR)  │    │ (Binary)    │    │ line Points  │    │ Errors      │
└─────────┘    └─────────────┘    └──────────────┘    └─────────────┘
                     │                   │                   │
                     ▼                   ▼                   ▼
              ┌─────────────┐    ┌──────────────┐    ┌─────────────┐
              │ Grayscale   │    │ 10 Horizontal│    │ pos_error   │
              │ + Threshold │    │ Slices       │    │ heading_err │
              │ + Morphology│    │ Centroid/row │    │ confidence  │
              └─────────────┘    └──────────────┘    └─────────────┘
```

---

## 2. Tiền Xử Lý Ảnh

### 2.1 Vùng Quan Tâm (ROI - Region of Interest)

Chỉ xử lý một vùng hình thang phía trước robot để giảm nhiễu và tăng tốc độ.

```
Frame gốc (640×480):
┌────────────────────────────────────────────────────────────┐
│                                                            │
│                     (Bỏ qua)                               │
│                                                            │
├──────────────┬────────────────────────────┬───────────────┤ y = ROI_TOP_Y × H
│              │░░░░░░░░░░░░░░░░░░░░░░░░░░░░│               │   = 0.55 × 480
│              │░░░░░░░░░░░░░░░░░░░░░░░░░░░░│               │   = 264
│              │░░░░░░░ ROI ZONE ░░░░░░░░░░│               │
│              │░░░░░░░░░░░░░░░░░░░░░░░░░░░░│               │
├──────────────┴────────────────────────────┴───────────────┤ y = ROI_BOTTOM_Y × H
│                                                            │   = 0.95 × 480
└────────────────────────────────────────────────────────────┘   = 456
```

**Thông số ROI (hình thang):**

| Thông số | Giá trị | Vị trí |
|----------|---------|--------|
| `ROI_TOP_LEFT_X` | 0.30 | 30% width từ trái |
| `ROI_TOP_RIGHT_X` | 0.70 | 70% width từ trái |
| `ROI_BOTTOM_LEFT_X` | 0.10 | 10% width từ trái |
| `ROI_BOTTOM_RIGHT_X` | 0.90 | 90% width từ trái |
| `ROI_TOP_Y` | 0.55 | 55% height từ trên |
| `ROI_BOTTOM_Y` | 0.95 | 95% height từ trên |

**Code tạo ROI mask:**

```python
vertices = np.array([[
    (int(640 * 0.10), int(480 * 0.95)),  # Bottom left
    (int(640 * 0.90), int(480 * 0.95)),  # Bottom right
    (int(640 * 0.70), int(480 * 0.55)),  # Top right
    (int(640 * 0.30), int(480 * 0.55)),  # Top left
]], dtype=np.int32)
cv2.fillPoly(roi_mask, vertices, 255)
```

### 2.2 Chuyển Đổi Grayscale

```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
```

Chuyển từ ảnh màu BGR (3 kênh) sang grayscale (1 kênh) để giảm lượng dữ liệu và đơn giản hóa xử lý.

### 2.3 Làm Mờ Gaussian

```python
gray = cv2.GaussianBlur(gray, (5, 5), 0)
```

Áp dụng bộ lọc Gaussian 5×5 để:
- Giảm nhiễu salt-and-pepper
- Làm mịn các cạnh
- Cải thiện kết quả threshold

### 2.4 Phân Ngưỡng Thích Ứng (Adaptive Thresholding)

Sử dụng **3 phương pháp kết hợp** để robust với điều kiện ánh sáng thay đổi:

#### Phương pháp 1: Adaptive Threshold

```python
binary_adaptive = cv2.adaptiveThreshold(
    gray,
    255,
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,  # Sử dụng trung bình Gaussian
    cv2.THRESH_BINARY_INV,           # Đảo ngược (line đen → trắng)
    blockSize=51,                     # Kích thước vùng lân cận
    C=10                              # Hằng số trừ đi
)
```

**Công thức:**
$$T(x,y) = \text{GaussianMean}(x,y, 51) - 10$$

Pixel tại $(x,y)$ được đặt thành:
- 255 nếu $I(x,y) < T(x,y)$ (pixel tối = line)
- 0 nếu $I(x,y) \geq T(x,y)$ (pixel sáng = nền)

**Ưu điểm:** Tự động điều chỉnh theo ánh sáng cục bộ.

#### Phương pháp 2: Otsu's Method

```python
_, binary_otsu = cv2.threshold(
    gray, 0, 255, 
    cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
)
```

Otsu tự động tìm ngưỡng tối ưu bằng cách minimize variance giữa 2 lớp (foreground/background).

**Công thức Otsu:**
$$\sigma^2_w(t) = \omega_0(t)\sigma^2_0(t) + \omega_1(t)\sigma^2_1(t)$$

Trong đó:
- $\omega_0, \omega_1$ = trọng số 2 lớp
- $\sigma^2_0, \sigma^2_1$ = variance 2 lớp

Ngưỡng $t^*$ được chọn sao cho $\sigma^2_w(t)$ nhỏ nhất.

#### Phương pháp 3: Kết Hợp (Intersection)

```python
binary = cv2.bitwise_and(binary_adaptive, binary_otsu)
```

Chỉ giữ pixel nào **cả hai phương pháp đều cho là line** → giảm false positive.

#### Fallback: Fixed Threshold

```python
if roi_pixels < 100:  # Không đủ pixel
    _, binary = cv2.threshold(gray, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    # BLACK_THRESHOLD = 80
```

Nếu kết hợp cho quá ít pixel, dùng ngưỡng cố định.

### 2.5 Phép Toán Hình Thái (Morphological Operations)

#### Close: Đóng lỗ hổng

```python
binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=3)
```

```
Trước CLOSE:           Sau CLOSE:
██  ██  ██             ██████████
██  ██  ██      →      ██████████
██  ██  ██             ██████████
```

**Công thức:** $\text{Close}(A) = \text{Erode}(\text{Dilate}(A))$

#### Open: Loại bỏ nhiễu nhỏ

```python
binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
```

```
Trước OPEN:            Sau OPEN:
██████████  ●●         ██████████
██████████      →      ██████████
██████████  ●          ██████████
  noise                  (đã loại)
```

**Công thức:** $\text{Open}(A) = \text{Dilate}(\text{Erode}(A))$

---

## 3. Phương Pháp Horizontal Slicing

### 3.1 Nguyên Lý

Chia ROI thành **10 lát ngang** (horizontal slices), với mỗi lát:
1. Tìm tất cả pixel trắng (thuộc line)
2. Tính **centroid** (tâm) theo trục X
3. Lưu tọa độ (x_center, y_center)

```
Binary image sau threshold:
┌────────────────────────────────────────┐
│                                        │
├───────────────█████████────────────────┤ Slice 0: centroid = (320, 270)
│               ████████                 │
├───────────────████████─────────────────┤ Slice 1: centroid = (315, 290)
│              ████████                  │
├──────────────████████──────────────────┤ Slice 2: centroid = (310, 310)
│             ████████                   │
├─────────────████████───────────────────┤ Slice 3: centroid = (305, 330)
│            ████████                    │
├────────────████████────────────────────┤ Slice 4: centroid = (300, 350)
│           ████████                     │
├───────────████████─────────────────────┤ Slice 5: ...
│          ████████                      │
├──────────████████──────────────────────┤ Slice 6: ...
│         ████████                       │
├─────────████████───────────────────────┤ Slice 7: ...
│        ████████                        │
├────────████████────────────────────────┤ Slice 8: ...
│       ████████                         │
├───────████████─────────────────────────┤ Slice 9: centroid = (275, 430)
│                                        │
└────────────────────────────────────────┘
```

### 3.2 Thuật Toán Chi Tiết

```
Algorithm: Find Centerline Points
────────────────────────────────────────────────────────────────

Input:  binary[H×W], num_slices=10
Output: centerline_points = [(x1,y1), (x2,y2), ..., (xn,yn)]

1. DEFINE ROI boundaries:
   roi_top    ← H × ROI_TOP_Y      // 480 × 0.55 = 264
   roi_bottom ← H × ROI_BOTTOM_Y   // 480 × 0.95 = 456

2. CALCULATE slice height:
   slice_height ← (roi_bottom - roi_top) / num_slices
                = (456 - 264) / 10 = 19.2 ≈ 19 pixels

3. FOR each slice i = 0 to 9:
   
   a. Calculate slice boundaries:
      y_start  ← roi_top + i × slice_height
      y_end    ← y_start + slice_height
      y_center ← (y_start + y_end) / 2
   
   b. Extract slice:
      slice_img ← binary[y_start:y_end, :]
   
   c. Find white pixels:
      white_pixels ← np.where(slice_img > 0)
      // white_pixels[0] = y coordinates (local)
      // white_pixels[1] = x coordinates
   
   d. Calculate centroid:
      IF count(white_pixels[1]) > 10:  // Need enough pixels
          x_center ← mean(white_pixels[1])
          centerline_points.append((x_center, y_center))

4. RETURN centerline_points
```

### 3.3 Ví Dụ Tính Toán

**Slice 5 (giữa ROI):**

```python
y_start = 264 + 5 × 19 = 359
y_end = 359 + 19 = 378
y_center = (359 + 378) / 2 = 368

slice_img = binary[359:378, :]

# Giả sử tìm được các pixel trắng tại x = [280, 285, 290, 295, 300, 305, 310]
x_center = mean([280, 285, 290, 295, 300, 305, 310]) = 295

centerline_point = (295, 368)
```

### 3.4 Tại Sao Dùng 10 Slices?

| Số slice | Ưu điểm | Nhược điểm |
|----------|---------|------------|
| Ít (5) | Nhanh, robust với nhiễu | Độ chính xác thấp |
| Trung bình (10) | Cân bằng | - |
| Nhiều (20) | Chính xác | Chậm, nhạy với nhiễu |

10 slices cho **19 pixel/slice** - đủ để có centroid ổn định.

---

## 4. Tính Toán Position Error

### 4.1 Định Nghĩa

**Position Error** = Độ lệch ngang của line so với tâm robot.

```
                 Camera View
    ┌────────────────────────────────────┐
    │                │                   │
    │                │ center_x          │
    │                │ (robot center)    │
    │                │                   │
    │                │        ●          │
    │                │     bottom_point  │
    │                │     (line center) │
    │                │                   │
    │                │←───error_px────→  │
    │                │                   │
    └────────────────────────────────────┘
    
    position_error = error_px / (width/2)
                   = (line_x - center_x) / (width/2)
```

### 4.2 Công Thức

**Bước 1: Xác định điểm bottom của centerline**

```python
# Sắp xếp theo y giảm dần (bottom = y lớn nhất)
sorted_points = sorted(centerline_points, key=lambda p: p[1], reverse=True)
bottom_point = sorted_points[0]  # Điểm gần robot nhất
```

**Bước 2: Tính tâm ảnh (có offset camera)**

```python
# CAMERA_OFFSET_X: offset nếu camera không ở giữa robot
# Dương = camera lệch phải, Âm = camera lệch trái
image_center_x = width / 2 + CAMERA_OFFSET_X
# Ví dụ: 640/2 + 0 = 320
```

**Bước 3: Tính error (pixel)**

```python
position_error_pixels = bottom_point[0] - image_center_x
# Ví dụ: 380 - 320 = +60 pixels (line ở bên phải)
```

**Bước 4: Normalize về [-1, +1]**

```python
position_error = position_error_pixels / (width / 2)
# Ví dụ: 60 / 320 = +0.1875

position_error = np.clip(position_error, -1, 1)
```

### 4.3 Ý Nghĩa Giá Trị

| Position Error | Ý nghĩa | Hành động robot |
|----------------|---------|-----------------|
| -1.0 | Line ở sát mép trái | Rẽ trái mạnh |
| -0.5 | Line ở nửa trái | Rẽ trái vừa |
| 0.0 | Line ở giữa | Đi thẳng |
| +0.5 | Line ở nửa phải | Rẽ phải vừa |
| +1.0 | Line ở sát mép phải | Rẽ phải mạnh |

### 4.4 Camera Offset

Nếu camera không đặt ở giữa robot:

```
        Top View
    ┌─────────────────┐
    │      ┌───┐      │
    │      │CAM│      │ ← Camera lệch phải 5cm
    │      └───┘      │    CAMERA_OFFSET_X = +50 (pixels)
    │                 │
    │    ┌───────┐    │
    │    │ ROBOT │    │
    │    │ CENTER│    │
    │    └───────┘    │
    └─────────────────┘
```

Khi đó `image_center_x = 320 + 50 = 370`, và line ở pixel 370 sẽ có `position_error = 0`.

---

## 5. Tính Toán Heading Error

### 5.1 Định Nghĩa

**Heading Error** = Góc giữa hướng line và hướng robot.

```
                 Camera View
    ┌────────────────────────────────────┐
    │                                    │
    │               ●                    │
    │              ╱ top point           │
    │             ╱                      │
    │            ╱                       │
    │           ╱                        │
    │          ╱  ← Line direction       │
    │         ╱                          │
    │        ╱     θ (heading error)     │
    │       ●─────────┬──────────────    │
    │    bottom       │                  │
    │    point        │ Robot direction  │
    │                 │ (straight ahead) │
    │                 ▼                  │
    └────────────────────────────────────┘
```

### 5.2 Phương Pháp Linear Fitting

Fit một đường thẳng qua các centerline points bằng **phương pháp bình phương tối thiểu**.

**Mô hình:** $x = m \cdot y + b$

Trong đó:
- $x$ = tọa độ X của điểm trên line
- $y$ = tọa độ Y của điểm
- $m$ = slope (độ dốc)
- $b$ = intercept

**Tại sao dùng $x = f(y)$ thay vì $y = f(x)$?**

Vì line thường gần thẳng đứng trong ảnh, dùng $y = f(x)$ có thể gây chia cho 0 khi line thẳng đứng.

### 5.3 Công Thức Tính

**Bước 1: Chuẩn bị dữ liệu**

```python
points_array = np.array(centerline_points)
y_vals = points_array[:, 1]  # Tất cả y
x_vals = points_array[:, 0]  # Tất cả x
```

**Bước 2: Linear fit**

```python
coeffs = np.polyfit(y_vals, x_vals, 1)
# coeffs = [m, b] với x = m*y + b
slope = coeffs[0]  # m = dx/dy
```

**Công thức polyfit (Least Squares):**

$$m = \frac{n\sum x_i y_i - \sum x_i \sum y_i}{n\sum y_i^2 - (\sum y_i)^2}$$

**Bước 3: Tính heading error**

```python
heading_error = np.arctan(slope)  # radians
heading_error_degrees = np.degrees(heading_error)
```

**Công thức:**
$$\theta = \arctan(m) = \arctan\left(\frac{dx}{dy}\right)$$

### 5.4 Ý Nghĩa Slope

| Slope (m) | Heading Error | Ý nghĩa |
|-----------|---------------|---------|
| -0.5 | -26.6° | Line nghiêng sang trái |
| -0.2 | -11.3° | Line hơi nghiêng trái |
| 0.0 | 0° | Line thẳng đứng |
| +0.2 | +11.3° | Line hơi nghiêng phải |
| +0.5 | +26.6° | Line nghiêng sang phải |

### 5.5 Ví Dụ Tính Toán

**Dữ liệu centerline:**

```python
points = [(320, 270), (315, 290), (310, 310), (305, 330), (300, 350)]
#          ↑top                                            ↑bottom
```

**Tính slope:**

```python
y_vals = [270, 290, 310, 330, 350]
x_vals = [320, 315, 310, 305, 300]

# Các tổng:
n = 5
Σxy = 320×270 + 315×290 + 310×310 + 305×330 + 300×350
    = 86400 + 91350 + 96100 + 100650 + 105000 = 479500
Σx = 320 + 315 + 310 + 305 + 300 = 1550
Σy = 270 + 290 + 310 + 330 + 350 = 1550
Σy² = 72900 + 84100 + 96100 + 108900 + 122500 = 484500

# Slope:
m = (5×479500 - 1550×1550) / (5×484500 - 1550²)
  = (2397500 - 2402500) / (2422500 - 2402500)
  = -5000 / 20000
  = -0.25

# Heading error:
θ = arctan(-0.25) = -14.04°
```

**Kết luận:** Line nghiêng sang trái 14°, robot cần rẽ trái để căn chỉnh.

---

## 6. Chế Độ Khôi Phục Line

### 6.1 Vấn Đề

Khi robot mất line (ra khỏi đường, giao lộ, v.v.), cần có cơ chế tự động tìm lại.

### 6.2 State Machine

```
                    ┌───────────────┐
                    │   TRACKING    │ ← Trạng thái bình thường
                    │  (đang theo)  │
                    └───────┬───────┘
                            │
                    Line mất 1 frame
                            │
                            ▼
                    ┌───────────────┐
                    │   WAITING     │ ← Chờ 3 frame
                    │ (frames_lost  │   (có thể false negative)
                    │   = 1,2,3)    │
                    └───────┬───────┘
                            │
                    Line vẫn mất sau 3 frame
                            │
                            ▼
                    ┌───────────────┐
                    │  SEARCHING    │ ← Bắt đầu xoay tìm
                    │ (frames_lost  │
                    │   > 3)        │
                    └───────┬───────┘
                            │
                       Tìm lại line
                            │
                            ▼
                    ┌───────────────┐
                    │   TRACKING    │
                    └───────────────┘
```

### 6.3 Thuật Toán Tìm Kiếm

```
Algorithm: Line Recovery Search
────────────────────────────────────────────────────────────────

1. WHEN line first lost (frames_lost = 1):
   IF last_known_position > 0.1:
       search_direction ← +1  // Line ở phải, tìm về phải
   ELSE IF last_known_position < -0.1:
       search_direction ← -1  // Line ở trái, tìm về trái
   ELSE:
       search_direction ← +1  // Mặc định tìm phải

2. WAIT for max_frames_before_search = 3 frames

3. IF frames_lost > 3:
   // Bắt đầu oscillating search
   
   cycles ← (frames_lost - 3) / 10
   amplitude ← min(0.8, base_amplitude + cycles × 0.1)
   
   phase ← (frames_lost - 3) % 20
   IF phase < 10:
       search_error ← amplitude × search_direction
   ELSE:
       search_error ← -amplitude × search_direction
   
   // Robot sẽ xoay theo search_error

4. RETURN search_error as position_error
```

### 6.4 Pattern Tìm Kiếm

```
Time →

search_error:
     0.8 ┤          ╭──╮              ╭──╮
         │         ╱    ╲            ╱    ╲
     0.4 ┤        ╱      ╲          ╱      ╲
         │       ╱        ╲        ╱        ╲
     0.0 ┼──────╱──────────╲──────╱──────────╲──────
         │                  ╲    ╱            ╲    ╱
    -0.4 ┤                   ╲  ╱              ╲  ╱
         │                    ╲╱                ╲╱
    -0.8 ┤
         └─────┬─────┬─────┬─────┬─────┬─────┬─────→ frames
              10    20    30    40    50    60
              
         │wait│←─── period 1 ───→│←─── period 2 ───→
```

**Giải thích:**
- Frame 1-3: Chờ (wait), không xoay
- Frame 4-13: Xoay phải với amplitude tăng dần
- Frame 14-23: Xoay trái
- Frame 24-33: Xoay phải với amplitude lớn hơn
- ...

### 6.5 Thông Số Cấu Hình

| Thông số | Giá trị | Ý nghĩa |
|----------|---------|---------|
| `max_frames_before_search` | 3 | Chờ 3 frame trước khi tìm |
| `search_amplitude` | 0.3 | Biên độ ban đầu |
| `search_increment` | 0.1 | Tăng biên độ mỗi 10 frame |
| Max amplitude | 0.8 | Giới hạn biên độ |

---

## 7. Bộ Lọc Làm Mịn Thích Ứng

### 7.1 Vấn Đề

Các giá trị error có thể nhảy đột ngột do:
- Nhiễu trong ảnh
- Line không đều
- Thay đổi ánh sáng

### 7.2 Low-Pass Filter (Exponential Smoothing)

**Công thức:**

$$y_t = \alpha \cdot x_t + (1 - \alpha) \cdot y_{t-1}$$

Trong đó:
- $x_t$ = giá trị mới (raw)
- $y_t$ = giá trị đã lọc
- $y_{t-1}$ = giá trị lọc trước đó
- $\alpha$ = hệ số smoothing (0 < α < 1)

**Ý nghĩa α:**
- α = 0: Không đổi (100% giá trị cũ)
- α = 0.3: Smooth nhiều (30% mới, 70% cũ)
- α = 0.7: Responsive (70% mới, 30% cũ)
- α = 1: Không smooth (100% giá trị mới)

### 7.3 Adaptive Smoothing

Hệ số α **thay đổi theo điều kiện**:

```python
def compute_adaptive_smoothing(result):
    alpha = base_smoothing_factor  # 0.4
    
    # Giảm smoothing khi confidence thấp
    if result.confidence < 0.5:
        alpha *= 0.5  # → 0.2
    
    # Giảm smoothing khi cua gấp
    if abs(result.heading_error_degrees) > 15:
        alpha *= 0.6  # → 0.24
    elif abs(result.heading_error_degrees) > 30:
        alpha *= 0.3  # → 0.12
    
    # Giảm smoothing khi vừa tìm lại line
    if frames_lost > 0:
        alpha *= 0.3  # → 0.12
    
    return max(0.1, min(0.6, alpha))
```

### 7.4 Bảng Tóm Tắt

| Điều kiện | α | Phản hồi |
|-----------|---|----------|
| Bình thường | 0.4 | Vừa phải |
| Confidence thấp | 0.2 | Nhanh hơn |
| Cua > 15° | 0.24 | Nhanh hơn |
| Cua > 30° | 0.12 | Rất nhanh |
| Vừa recovery | 0.12 | Rất nhanh |

**Lý do:**
- Cua gấp: Cần phản hồi nhanh, không nên trễ
- Confidence thấp: Có thể là nhiễu, cần phản hồi để điều chỉnh
- Vừa recovery: Robot có thể lệch nhiều, cần căn chỉnh nhanh

---

## 8. Ví Dụ Tính Toán Cụ Thể

### 8.1 Tình Huống: Robot Đang Theo Line Cong Sang Phải

**Input frame:**

```
Binary sau threshold:
┌────────────────────────────────────────┐
│                                        │
│                    ████                │
│                   ████                 │
│                  ████                  │
│                 ████                   │
│                ████                    │
│               ████                     │
│              ████                      │
│             ████                       │
│            ████                        │
└────────────────────────────────────────┘
```

**Bước 1: Tìm centerline points**

| Slice | y_center | White pixels X | x_center |
|-------|----------|----------------|----------|
| 0 | 270 | [395, 398, 401, 404, 407] | 401 |
| 1 | 289 | [375, 378, 381, 384, 387] | 381 |
| 2 | 308 | [355, 358, 361, 364, 367] | 361 |
| 3 | 327 | [335, 338, 341, 344, 347] | 341 |
| 4 | 346 | [315, 318, 321, 324, 327] | 321 |
| 5 | 365 | [295, 298, 301, 304, 307] | 301 |
| 6 | 384 | [275, 278, 281, 284, 287] | 281 |
| 7 | 403 | [255, 258, 261, 264, 267] | 261 |
| 8 | 422 | [235, 238, 241, 244, 247] | 241 |
| 9 | 441 | [215, 218, 221, 224, 227] | 221 |

**Centerline points:**
```python
points = [
    (401, 270), (381, 289), (361, 308), (341, 327), (321, 346),
    (301, 365), (281, 384), (261, 403), (241, 422), (221, 441)
]
```

**Bước 2: Tính Position Error**

```python
bottom_point = (221, 441)  # Điểm y lớn nhất
image_center_x = 640 / 2 = 320

position_error_pixels = 221 - 320 = -99
position_error = -99 / 320 = -0.309
```

**Kết quả:** Line ở bên **trái** robot 0.309 (31% về phía trái)

**Bước 3: Tính Heading Error**

```python
y_vals = [270, 289, 308, 327, 346, 365, 384, 403, 422, 441]
x_vals = [401, 381, 361, 341, 321, 301, 281, 261, 241, 221]

# Linear fit: x = m*y + b
coeffs = np.polyfit(y_vals, x_vals, 1)
# m = -1.052 (dx/dy)

heading_error = arctan(-1.052) = -46.4°
```

**Kết quả:** Line nghiêng sang **trái** 46.4° (cua gấp!)

**Bước 4: Confidence**

```python
confidence = 10 / 10 = 1.0  # 100% slices có centerline
```

**Bước 5: Adaptive Smoothing**

```python
# Cua gấp > 30° → giảm smoothing
alpha = 0.4 × 0.3 = 0.12

# Giả sử frame trước: pos_error = -0.25, heading = -40°
smoothed_pos = 0.12 × (-0.309) + 0.88 × (-0.25) = -0.257
smoothed_heading = 0.12 × (-46.4) + 0.88 × (-40) = -40.8°
```

**Output cuối cùng:**

```python
LineDetectionResult(
    line_detected = True,
    position_error = -0.257,
    position_error_pixels = -82.2,
    heading_error = -0.712,  # radians
    heading_error_degrees = -40.8,
    line_center_x = 221,
    line_center_y = 441,
    confidence = 1.0
)
```

---

## 9. Tích Hợp Với Bộ Điều Khiển PID

### 9.1 Cấu Trúc Điều Khiển

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│ Line        │     │ PID          │     │ Motor       │
│ Detector    │ ──→ │ Controller   │ ──→ │ Commands    │
│             │     │              │     │             │
│ pos_error   │     │ yaw_rate     │     │ left_motor  │
│ heading_err │     │              │     │ right_motor │
└─────────────┘     └──────────────┘     └─────────────┘
```

### 9.2 Kết Hợp 2 Error

**Weighted combination:**

$$\text{combined\_error} = k_p \cdot \text{pos\_error} + k_h \cdot \text{heading\_error}$$

Thường:
- $k_p = 0.6$ (weight cho position)
- $k_h = 0.4$ (weight cho heading)

Hoặc sử dụng công thức look-ahead:

$$\text{error} = \text{pos\_error} + L \cdot \tan(\text{heading\_error})$$

Trong đó $L$ = look-ahead distance (pixels).

### 9.3 PID Output

```python
# Trong MotionController
def compute_yaw_rate(self, line_result):
    # Combine errors
    error = (self.kp_weight * line_result.position_error + 
             self.kh_weight * np.tan(line_result.heading_error))
    
    # PID
    yaw_rate = (self.Kp * error + 
                self.Ki * self.integral + 
                self.Kd * (error - self.prev_error))
    
    # Clamp
    yaw_rate = np.clip(yaw_rate, -self.max_yaw, self.max_yaw)
    
    return yaw_rate
```

---

## 10. Giới Hạn và Cải Tiến

### 10.1 Giới Hạn Hiện Tại

| Giới hạn | Mô tả | Ảnh hưởng |
|----------|-------|-----------|
| **Single line only** | Chỉ detect 1 line | Không xử lý được giao lộ |
| **Black line on light** | Mặc định line đen | Cần điều chỉnh cho line trắng |
| **Fixed ROI** | ROI cố định | Không thích ứng với camera position |
| **No curve fitting** | Chỉ linear fit | Sai với curve gấp |
| **Shadow sensitive** | Nhạy với bóng | False positive khi có bóng |

### 10.2 Hướng Cải Tiến

**1. Polynomial Curve Fitting:**

```python
# Thay vì linear fit
coeffs = np.polyfit(y_vals, x_vals, 2)  # Quadratic
# x = a*y² + b*y + c
```

**2. Multi-line Support:**

```python
# Dùng clustering để tách multiple lines
from sklearn.cluster import DBSCAN
clusters = DBSCAN(eps=50).fit(white_pixels)
```

**3. Color-based Detection:**

```python
# HSV thresholding cho màu cụ thể
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower_color, upper_color)
```

**4. Deep Learning:**

```python
# Semantic segmentation cho line detection
model = load_model('line_segmentation.h5')
mask = model.predict(frame)
```

---

## Phụ Lục A: Bảng Thông Số

### A.1 Thông Số Camera

| Thông số | Giá trị | Đơn vị |
|----------|---------|--------|
| Resolution | 640 × 480 | pixel |
| FPS | 30 | Hz |
| FOV Horizontal | 87 | độ |
| FOV Vertical | 58 | độ |

### A.2 Thông Số ROI

| Thông số | Giá trị |
|----------|---------|
| ROI_TOP_Y | 0.55 |
| ROI_BOTTOM_Y | 0.95 |
| ROI_TOP_LEFT_X | 0.30 |
| ROI_TOP_RIGHT_X | 0.70 |
| ROI_BOTTOM_LEFT_X | 0.10 |
| ROI_BOTTOM_RIGHT_X | 0.90 |

### A.3 Thông Số Thuật Toán

| Thông số | Giá trị | Mô tả |
|----------|---------|-------|
| num_slices | 10 | Số lát ngang |
| min_pixels | 10 | Min pixels để tính centroid |
| BLACK_THRESHOLD | 80 | Ngưỡng fallback |
| MORPH_KERNEL_SIZE | 3 | Kích thước kernel |
| MORPH_CLOSE_ITERATIONS | 3 | Số lần close |
| MORPH_OPEN_ITERATIONS | 1 | Số lần open |

### A.4 Thông Số Recovery

| Thông số | Giá trị |
|----------|---------|
| max_frames_before_search | 3 |
| search_amplitude | 0.3 |
| search_increment | 0.1 |
| max_amplitude | 0.8 |

---

## Phụ Lục B: Code Reference

### B.1 File Structure

```
src/perception/
├── simple_line_detector.py  # Module chính
├── __init__.py             # Export

tools/testing/
├── test_modules_interactive.py  # Test trực quan
```

### B.2 API Usage

```python
from src.perception import SimpleLineDetector

# Khởi tạo
detector = SimpleLineDetector()

# Detect
result = detector.detect(color_frame)

# Sử dụng kết quả
if result.line_detected:
    yaw_rate = pid.compute(result.position_error, result.heading_error)
    robot.set_yaw_rate(yaw_rate)
else:
    # Line lost - sử dụng search error
    robot.set_yaw_rate(result.position_error * search_gain)

# Visualization
vis_frame = detector.visualize(color_frame, result)
cv2.imshow("Line", vis_frame)
```

---

*Tài liệu này được tạo cho dự án Autonomous Robot - Line Detector Module*
