# Nguyên Lý Tính Toán Hệ Thống Phân Tích Địa Hình
## Terrain Analyzer - Adaptive Ground Clearance System

**Tác giả:** Autonomous Robot Project  
**Phiên bản:** 1.0  
**Ngày:** Tháng 1, 2026

---

## Mục Lục

1. [Tổng Quan Hệ Thống](#1-tổng-quan-hệ-thống)
2. [Nguyên Lý Hoạt Động Depth Camera](#2-nguyên-lý-hoạt-động-depth-camera)
3. [Mô Hình Hình Học Camera](#3-mô-hình-hình-học-camera)
4. [Thuật Toán Phát Hiện Trần Thấp](#4-thuật-toán-phát-hiện-trần-thấp)
5. [Thuật Toán Phát Hiện Chướng Ngại Vật](#5-thuật-toán-phát-hiện-chướng-ngại-vật)
6. [Công Thức Ước Tính Chiều Cao](#6-công-thức-ước-tính-chiều-cao)
7. [Logic Quyết Định Hành Động](#7-logic-quyết-định-hành-động)
8. [Xử Lý Nhiễu và Làm Mịn](#8-xử-lý-nhiễu-và-làm-mịn)
9. [Ví Dụ Tính Toán Cụ Thể](#9-ví-dụ-tính-toán-cụ-thể)
10. [Giới Hạn và Cải Tiến](#10-giới-hạn-và-cải-tiến)

---

## 1. Tổng Quan Hệ Thống

### 1.1 Mục Tiêu

Hệ thống Terrain Analyzer được thiết kế để:

1. **Phát hiện trần thấp/giới hạn chiều cao phía trên** → Tự động hạ gầm xe để đi qua
2. **Phát hiện chướng ngại vật nhỏ trên mặt đất** → Tự động nâng gầm xe để vượt qua
3. **Đề xuất độ cao gầm tối ưu** dựa trên phân tích địa hình realtime

### 1.2 Đầu Vào/Đầu Ra

```
┌──────────────────────────────────────────────────────────────┐
│                    TERRAIN ANALYZER                          │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  INPUT:                         OUTPUT:                      │
│  ┌─────────────────┐           ┌─────────────────────────┐  │
│  │ Depth Frame     │           │ action: NORMAL/RAISE/   │  │
│  │ (640×480, m)    │    →      │         LOWER/STOP      │  │
│  │                 │           │ recommended_height: 0.05m│  │
│  │ Color Frame     │           │ ceiling_distance: 1.2m   │  │
│  │ (optional)      │           │ obstacle_height: 0.03m   │  │
│  └─────────────────┘           └─────────────────────────┘  │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### 1.3 Các Trạng Thái Đầu Ra

| Trạng thái | Điều kiện | Hành động Robot |
|------------|-----------|-----------------|
| `NORMAL` | Địa hình bình thường | Giữ gầm 5cm |
| `RAISE` | Có chướng ngại vật ≤ 5cm | Nâng gầm để vượt qua |
| `LOWER` | Trần thấp, không đủ headroom | Hạ gầm xuống 2cm |
| `STOP` | Chướng ngại vật quá cao | Dừng lại hoặc tránh |

---

## 2. Nguyên Lý Hoạt Động Depth Camera

### 2.1 Công Nghệ Stereo Vision

Intel RealSense D435 sử dụng công nghệ **Stereo Infrared** để đo khoảng cách:

```
┌────────────────────────────────────────────────────────────┐
│                    REALSENSE D435                          │
│  ┌──────┐      ┌──────────┐      ┌──────┐    ┌──────┐    │
│  │IR Cam│      │IR        │      │IR Cam│    │RGB   │    │
│  │Left  │      │Projector │      │Right │    │Cam   │    │
│  └──────┘      └──────────┘      └──────┘    └──────┘    │
│     ↓               ↓                ↓                    │
│     │          Chiếu pattern         │                    │
│     │          chấm IR               │                    │
│     ▼               ▼                ▼                    │
│  ┌─────────────────────────────────────┐                  │
│  │        Vật thể trong scene          │                  │
│  └─────────────────────────────────────┘                  │
└────────────────────────────────────────────────────────────┘
```

### 2.2 Nguyên Lý Tính Khoảng Cách (Triangulation)

Hai camera IR đặt cách nhau một khoảng **baseline** (b ≈ 50mm). Khi nhìn cùng một điểm P trong không gian:

```
                         P (điểm cần đo)
                         ●
                        ╱╲
                       ╱  ╲
                      ╱    ╲
                     ╱      ╲
                    ╱   Z    ╲
                   ╱    ↑     ╲
                  ╱     │      ╲
                 ╱      │       ╲
    ┌───────────●───────┼────────●───────────┐
    │  Camera   │       │        │  Camera   │
    │   Left    │←──────┼───────→│   Right   │
    │           │   b (baseline) │           │
    └───────────┴───────┴────────┴───────────┘
                        
    x_L = vị trí P trên ảnh trái
    x_R = vị trí P trên ảnh phải
    d = x_L - x_R = disparity (độ lệch pixel)
```

**Công thức tính depth:**

$$Z = \frac{f \times b}{d}$$

Trong đó:
- $Z$ = khoảng cách đến điểm P (mét)
- $f$ = focal length của camera (pixel)
- $b$ = baseline giữa 2 camera (mét)
- $d$ = disparity (pixel)

### 2.3 Depth Frame

Kết quả là một **depth map** có cùng kích thước với ảnh:

```python
depth_frame.shape = (480, 640)   # Height × Width
depth_frame.dtype = float32      # Giá trị là mét

# Mỗi pixel chứa khoảng cách từ camera đến điểm đó
depth_frame[y, x] = 1.52  # Điểm tại (x,y) cách camera 1.52m
```

**Ví dụ depth frame:**

```
Depth Frame (đơn vị: mét)
┌────────────────────────────────────────────────────┐
│ 0.00  0.00  0.00  2.10  2.15  2.20  2.18  ...     │ ← Hàng 0 (top)
│ 0.00  1.85  1.90  2.05  2.10  2.12  2.15  ...     │   0.00 = invalid
│ 1.80  1.82  1.88  1.95  2.00  2.05  2.08  ...     │
│ ...                                               │
│ 1.20  1.22  1.25  1.28  1.30  1.32  1.35  ...     │ ← Hàng 400
│ 0.95  0.98  1.00  1.02  1.05  1.08  1.10  ...     │   (gần hơn vì
│ 0.80  0.82  0.85  0.88  0.90  0.92  0.95  ...     │    nhìn xuống đất)
└────────────────────────────────────────────────────┘
                                                Hàng 479 (bottom)
```

---

## 3. Mô Hình Hình Học Camera

### 3.1 Cấu Hình Lắp Đặt Camera

Camera được lắp trên robot với các thông số:

| Thông số | Ký hiệu | Giá trị | Mô tả |
|----------|---------|---------|-------|
| Chiều cao camera | $h_{cam}$ | 0.20 m | Khoảng cách từ camera đến mặt đất |
| Góc nghiêng | $\theta_{tilt}$ | 15° | Góc so với phương ngang, dương = cúi xuống |
| Vertical FOV | $FOV_v$ | 58° | Góc nhìn theo chiều dọc (RealSense D435) |
| Horizontal FOV | $FOV_h$ | 87° | Góc nhìn theo chiều ngang |

### 3.2 Sơ Đồ Góc Nhìn (Side View)

```
                                    Phương ngang (horizon)
            ════════════════════════════════════════════════════
                                          ↑
                                          │ θ_top = +14°
                              ╱───────────┼───────────╲
                             ╱            │            ╲
                            ╱   θ_tilt    │             ╲
                           ╱     = 15°    │              ╲
                          ╱       ↘       ●───────────────→ Tia center
                         ╱         ╲     ╱╲   Camera      
                        ╱           ╲   ╱  ╲              
                       ╱  Tia top    ╲ ╱    ╲ Tia bottom  
                      ╱               ╳      ╲            
                     ╱               ╱ ╲      ╲           
                    ╱               ╱   ╲      ╲          
                   ╱               ╱     ╲      ╲         
                  ╱               ╱       ╲      ╲        
                 ╱               ╱         ╲      ╲ θ_bottom = -44°
                ╱               ╱           ╲      ╲      
    ═══════════════════════════════════════════════════════
                            Mặt đất
                            
    h_cam = 0.20m
```

### 3.3 Tính Toán Góc Nhìn Từng Phần Frame

Với camera nghiêng $\theta_{tilt} = 15°$ và $FOV_v = 58°$:

**Góc nhìn tại top của frame:**
$$\theta_{top} = -\theta_{tilt} + \frac{FOV_v}{2} = -15° + 29° = +14°$$

**Góc nhìn tại center của frame:**
$$\theta_{center} = -\theta_{tilt} = -15°$$

**Góc nhìn tại bottom của frame:**
$$\theta_{bottom} = -\theta_{tilt} - \frac{FOV_v}{2} = -15° - 29° = -44°$$

**Góc nhìn tại vị trí y bất kỳ trong frame:**
$$\theta(y) = -\theta_{tilt} + FOV_v \times \left(\frac{1}{2} - \frac{y}{H}\right)$$

Trong đó:
- $y$ = vị trí pixel theo chiều dọc (0 = top, H-1 = bottom)
- $H$ = chiều cao frame (480 pixel)

### 3.4 Ánh Xạ Góc Nhìn Lên Frame

```
┌─────────────────────────────────────────┐
│ y = 0      │  θ = +14° (nhìn LÊN)       │ ← Có thể thấy trần
│            │                             │
│ y = 120    │  θ = +1.5°                  │
├────────────┼─────────────────────────────┤
│ y = 240    │  θ = -15° (center)          │ ← Nhìn thẳng về trước
├────────────┼─────────────────────────────┤
│ y = 360    │  θ = -30°                   │
│            │                             │
│ y = 480    │  θ = -44° (nhìn XUỐNG)      │ ← Thấy mặt đất gần
└─────────────────────────────────────────┘
```

---

## 4. Thuật Toán Phát Hiện Trần Thấp

### 4.1 Định Nghĩa Vùng Ceiling Zone

Ceiling Zone là vùng **phía trên** của frame, nơi có thể nhìn thấy trần hoặc vật cản phía trên.

```
┌─────────────────────────────────────────┐
│░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│  
│░░░░░░░░░░ CEILING ZONE ░░░░░░░░░░░░░░░░│  y = 0 đến 0.30×H
│░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│  Góc: +14° đến -2.6°
├─────────────────────────────────────────┤  ← y = 0.30 × 480 = 144
│                                         │
│           (Không phân tích)             │
│                                         │
└─────────────────────────────────────────┘
```

**Thông số:**
- `ceiling_zone_top` = 0.0 (bắt đầu từ top)
- `ceiling_zone_bottom` = 0.30 (đến 30% chiều cao frame)

### 4.2 Thuật Toán Phát Hiện

```
Algorithm: Ceiling Detection
────────────────────────────────────────────────────────────────

Input:  depth_frame[H×W], config
Output: ceiling_detected, ceiling_distance, clearance_ok

1. EXTRACT ceiling zone:
   y_start ← 0
   y_end   ← H × ceiling_zone_bottom
   ceiling_zone ← depth_frame[y_start:y_end, margin:W-margin]

2. FILTER valid depths:
   valid_mask ← (ceiling_zone > 0.1) AND (ceiling_zone < 5.0)
   valid_depths ← ceiling_zone[valid_mask]
   
   IF count(valid_depths) < 100:
       RETURN (detected=False, distance=-1, clearance_ok=True)

3. COMPUTE closest ceiling point:
   // Sử dụng percentile 10 để lấy điểm gần nhất
   // (robust hơn min, tránh outlier)
   ceiling_distance ← percentile(valid_depths, 10)

4. SMOOTH with history:
   history.append(ceiling_distance)
   IF len(history) > window_size:
       history ← history[-window_size:]
   smoothed_distance ← median(history)

5. DETERMINE status:
   ceiling_detected ← (smoothed_distance < ceiling_warning_distance)
   clearance_ok     ← (smoothed_distance >= ceiling_min_clearance)

6. RETURN (ceiling_detected, smoothed_distance, clearance_ok)
```

### 4.3 Giải Thích Chi Tiết

**Tại sao dùng percentile 10?**

```
Depth values trong ceiling zone (ví dụ):
[2.5, 2.3, 0.8, 2.4, 2.6, 0.75, 2.2, 2.5, 0.82, 2.3]
      ↑           ↑               ↑
   tường xa    TRẦN GẦN        TRẦN GẦN

sorted: [0.75, 0.78, 0.80, 0.82, 2.2, 2.3, 2.3, 2.4, 2.5, 2.6]
                  ↑
           percentile 10 ≈ 0.79m
           
→ Phát hiện có trần ở khoảng cách 0.79m
```

- `min()` có thể bị ảnh hưởng bởi noise → sai
- `median()` lấy giá trị giữa → có thể miss trần nếu trần chỉ chiếm 1 phần nhỏ
- `percentile(10)` lấy giá trị gần nhất mà vẫn robust với noise

---

## 5. Thuật Toán Phát Hiện Chướng Ngại Vật

### 5.1 Định Nghĩa Vùng Ground Zone

Ground Zone là vùng **phía dưới** của frame, nơi camera nhìn xuống mặt đất.

```
┌─────────────────────────────────────────┐
│                                         │
│           (Không phân tích)             │
│                                         │
├─────────────────────────────────────────┤  ← y = 0.55 × 480 = 264
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│
│▓▓▓▓▓▓▓▓▓▓ GROUND ZONE ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│  y = 0.55×H đến 0.90×H
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│  Góc: -27° đến -42°
├─────────────────────────────────────────┤  ← y = 0.90 × 480 = 432
│         (Bỏ qua edge effect)            │
└─────────────────────────────────────────┘
```

### 5.2 Nguyên Lý Phát Hiện

**Ý tưởng cốt lõi:** Trên mặt đất phẳng, tất cả pixel trong ground zone có depth tương đương. Nếu có chướng ngại vật, một số pixel sẽ có depth **nhỏ hơn** (gần camera hơn).

```
Mặt đất phẳng:                    Có gờ/vật cản:
                                  
       Camera                           Camera
          │                                │
          ▼                                ▼
    ══════════════  1.2m          ══════┬═════  1.2m
     (tất cả depth ≈ 1.2m)              █│█
                                   ════█│█════  0.9m (gờ gần hơn)
                                       
Depth frame:                      Depth frame:
[1.2, 1.2, 1.2, 1.2, 1.2]        [1.2, 1.2, 0.9, 0.9, 1.2]
      ↑                                      ↑
   baseline                              obstacle
```

### 5.3 Thuật Toán Phát Hiện

```
Algorithm: Ground Obstacle Detection
────────────────────────────────────────────────────────────────

Input:  depth_frame[H×W], config
Output: obstacle, obstacle_height, obstacle_distance, can_step_over

1. EXTRACT ground zone:
   y_start ← H × ground_zone_top
   y_end   ← H × ground_zone_bottom
   ground_zone ← depth_frame[y_start:y_end, :]

2. FILTER valid depths:
   valid_mask ← (ground_zone > 0.1) AND (ground_zone < 5.0)
   valid_depths ← ground_zone[valid_mask]
   
   IF count(valid_depths) < 100:
       RETURN (obstacle=False, height=0, distance=-1, can_step_over=True)

3. COMPUTE baseline (mặt đất bình thường):
   // Median robust với outlier
   baseline ← median(valid_depths)

4. DETECT obstacle points:
   // Điểm nào gần hơn baseline một ngưỡng → là chướng ngại vật
   threshold ← obstacle_threshold  // 0.08m
   obstacle_mask ← valid_depths < (baseline - threshold)
   
   IF NOT any(obstacle_mask):
       RETURN (obstacle=False, height=0, distance=baseline, can_step_over=True)

5. GET obstacle characteristics:
   obstacle_depths ← valid_depths[obstacle_mask]
   obstacle_distance ← min(obstacle_depths)
   depth_diff ← baseline - obstacle_distance

6. ESTIMATE obstacle height:
   // Công thức chi tiết ở Section 6
   angle ← camera_tilt_angle + camera_vfov × 0.25
   estimated_height ← depth_diff × sin(angle)
   estimated_height ← clamp(estimated_height, 0, 0.3)

7. SMOOTH with history:
   history.append(estimated_height)
   smoothed_height ← median(history)

8. DETERMINE if can step over:
   can_step_over ← (smoothed_height <= max_step_height)

9. RETURN (obstacle=True, smoothed_height, obstacle_distance, can_step_over)
```

---

## 6. Công Thức Ước Tính Chiều Cao

### 6.1 Mô Hình Hình Học

Khi phát hiện có chướng ngại vật (depth nhỏ hơn baseline), cần ước tính chiều cao của vật.

```
                    Camera (tại gốc tọa độ)
                         ●
                        ╱ ╲
                       ╱   ╲
                      ╱ θ   ╲
                     ╱       ╲
                    ╱    d_o  ╲ d_g
                   ╱           ╲
                  ╱      ┌──────╲───────┐
                 ╱       │ VẬT   ╲      │
                ╱        │        ╲     │ h = ?
               ╱         └─────────╲────┘
              ╱                     ╲
    ═══════════════════════════════════════════
                    Mặt đất
                    
    Ký hiệu:
    θ   = góc nhìn (so với phương ngang)
    d_o = depth đến đỉnh chướng ngại vật
    d_g = depth đến mặt đất (baseline)
    h   = chiều cao chướng ngại vật
```

### 6.2 Phân Tích Chi Tiết

Xét hệ tọa độ với gốc tại camera:
- Trục X: hướng về phía trước (theo phương ngang)
- Trục Y: hướng xuống dưới

**Điểm trên mặt đất:**
$$x_g = d_g \times \cos(\theta)$$
$$y_g = d_g \times \sin(\theta)$$

**Điểm trên đỉnh vật:**
$$x_o = d_o \times \cos(\theta)$$
$$y_o = d_o \times \sin(\theta)$$

**Chiều cao vật (giả sử vật thẳng đứng):**

Trong trường hợp lý tưởng (vật ở cùng vị trí X với điểm mặt đất):
$$h = y_g - y_o = d_g \sin(\theta) - d_o \sin(\theta) = (d_g - d_o) \sin(\theta)$$

### 6.3 Công Thức Đơn Giản Hóa

Do vật và mặt đất phía sau nó không hoàn toàn cùng góc nhìn, ta dùng **góc trung bình** của ground zone:

$$\theta_{avg} = \theta_{tilt} + FOV_v \times 0.25$$

**Với các giá trị thực tế:**
- $\theta_{tilt} = 15°$
- $FOV_v = 58°$

$$\theta_{avg} = 15° + 58° \times 0.25 = 15° + 14.5° = 29.5°$$

**Công thức cuối cùng:**

$$h_{estimated} = \Delta d \times \sin(\theta_{avg})$$

$$h_{estimated} = (d_{baseline} - d_{obstacle}) \times \sin(29.5°)$$

$$h_{estimated} = \Delta d \times 0.492$$

### 6.4 Bảng Tra Cứu Nhanh

| Δd (chênh lệch depth) | Chiều cao ước tính | Ví dụ thực tế |
|-----------------------|-------------------|---------------|
| 0.05 m | 2.5 cm | Dây cáp nhỏ |
| 0.10 m | 4.9 cm | Gờ giảm tốc |
| 0.15 m | 7.4 cm | Bậc thềm thấp |
| 0.20 m | 9.8 cm | Bậc cầu thang |
| 0.30 m | 14.8 cm | Hộp/thùng |

### 6.5 Nguồn Sai Số

| Nguồn sai số | Ảnh hưởng | Cách khắc phục |
|--------------|-----------|----------------|
| Noise từ depth camera | ±5% | Smoothing với median filter |
| Góc nghiêng camera không chính xác | ±10% | Calibration chính xác |
| Vật không thẳng đứng | Biến đổi | Chấp nhận sai số |
| Vật ở rìa FOV | Góc khác với center | Dùng góc theo vị trí pixel |

---

## 7. Logic Quyết Định Hành Động

### 7.1 Sơ Đồ Luồng Quyết Định

```
                        ┌─────────────────┐
                        │ Bắt đầu phân    │
                        │ tích frame      │
                        └────────┬────────┘
                                 │
                        ┌────────▼────────┐
                        │ Phát hiện trần? │
                        └────────┬────────┘
                                 │
              ┌──────────────────┼──────────────────┐
              │ CÓ               │                  │ KHÔNG
              ▼                  │                  ▼
    ┌─────────────────┐          │        ┌─────────────────┐
    │ Đủ headroom?    │          │        │ Có obstacle?    │
    └────────┬────────┘          │        └────────┬────────┘
             │                   │                 │
      ┌──────┴──────┐            │          ┌──────┴──────┐
   KHÔNG           CÓ            │        CÓ            KHÔNG
      │             │            │          │              │
      ▼             ▼            │          │              ▼
  ┌───────┐   ┌─────────┐        │          │        ┌─────────┐
  │ LOWER │   │ Giảm    │        │          │        │ NORMAL  │
  │       │   │ height  │        │          │        │ (5cm)   │
  │ 2cm   │   │ tùy theo│        │          │        └─────────┘
  └───────┘   │ không   │        │          │
              │ gian    │        │          │
              └─────────┘        │   ┌──────▼──────┐
                                 │   │ Bước qua    │
                                 │   │ được?       │
                                 │   └──────┬──────┘
                                 │          │
                                 │   ┌──────┴──────┐
                                 │ CÓ            KHÔNG
                                 │   │              │
                                 │   ▼              ▼
                                 │ ┌───────┐   ┌───────┐
                                 │ │ RAISE │   │ STOP  │
                                 │ │       │   │       │
                                 │ │ h+2cm │   │       │
                                 │ └───────┘   └───────┘
                                 │
                                 └───────────────────────
```

### 7.2 Bảng Quyết Định

| Trần thấp? | Đủ headroom? | Obstacle? | Bước qua được? | Action | Height |
|------------|--------------|-----------|----------------|--------|--------|
| ❌ | - | ❌ | - | NORMAL | 5cm |
| ❌ | - | ✅ | ✅ | RAISE | h + 2cm |
| ❌ | - | ✅ | ❌ | STOP | 5cm |
| ✅ | ❌ | - | - | LOWER | 2cm |
| ✅ | ✅ | - | - | LOWER* | tùy |

*LOWER với height được tính để vừa đủ qua trần

### 7.3 Tính Toán Height Đề Xuất

**Trường hợp RAISE (nâng gầm):**
```python
needed_clearance = obstacle_height + 0.02  # Buffer 2cm
recommended_height = min(needed_clearance, max_ground_clearance)
```

**Trường hợp LOWER (hạ gầm):**
```python
available_space = ceiling_distance - robot_height - 0.1  # Buffer 10cm
recommended_height = max(min_ground_clearance, available_space)
```

---

## 8. Xử Lý Nhiễu và Làm Mịn

### 8.1 Nguồn Nhiễu Từ Depth Camera

| Loại nhiễu | Nguyên nhân | Biểu hiện |
|------------|-------------|-----------|
| Salt noise | Phản xạ kém | Depth = 0 hoặc rất lớn |
| Pepper noise | Nhiễu điện tử | Depth dao động ngẫu nhiên |
| Edge noise | Stereo matching fail | Sai ở biên vật thể |
| Temporal noise | Không ổn định | Giá trị thay đổi giữa các frame |

### 8.2 Chiến Lược Lọc Nhiễu

**Bước 1: Lọc giá trị không hợp lệ**
```python
valid_mask = (depth > 0.1) & (depth < 5.0)
# Loại bỏ: depth = 0, depth < 0.1m (quá gần), depth > 5m (quá xa)
```

**Bước 2: Sử dụng percentile thay vì min/max**
```python
# Thay vì min(depths) - dễ bị noise
ceiling_distance = np.percentile(valid_depths, 10)
# 10% giá trị nhỏ nhất → robust hơn
```

**Bước 3: Temporal smoothing với sliding window**
```python
class TemporalSmoother:
    def __init__(self, window_size=5):
        self.history = []
        self.window_size = window_size
    
    def smooth(self, value):
        self.history.append(value)
        if len(self.history) > self.window_size:
            self.history = self.history[-self.window_size:]
        return np.median(self.history)  # Median robust với outlier
```

### 8.3 Hiệu Quả Của Smoothing

```
Raw signal (noisy):
  ────╱╲────╱────╲───╱╲╱╲───────╱╲────
     spike      spike    noise    spike
     
After median smoothing (window=5):
  ─────────────────────────────────────
            Smooth và ổn định
```

---

## 9. Ví Dụ Tính Toán Cụ Thể

### 9.1 Ví Dụ 1: Phát Hiện Gờ Giảm Tốc

**Tình huống:** Robot đang di chuyển, phía trước có gờ giảm tốc cao 5cm.

**Input:**
```python
depth_frame = [...]  # 640×480 frame
# Ground zone (y=264 đến y=432):
# - Phần lớn depth ≈ 1.5m (mặt đường)
# - Một vùng nhỏ depth ≈ 1.4m (gờ)
```

**Bước 1: Trích xuất ground zone**
```python
ground_zone = depth_frame[264:432, :]
# Shape: (168, 640)
```

**Bước 2: Lọc valid**
```python
valid_mask = (ground_zone > 0.1) & (ground_zone < 5.0)
valid_depths = ground_zone[valid_mask]
# Giả sử có 80,000 điểm valid
```

**Bước 3: Tính baseline**
```python
baseline = np.median(valid_depths)
# baseline = 1.50m
```

**Bước 4: Phát hiện obstacle**
```python
threshold = 0.08  # 8cm
obstacle_mask = valid_depths < (1.50 - 0.08)
# obstacle_mask tìm được 5,000 điểm < 1.42m
```

**Bước 5: Tính depth obstacle**
```python
obstacle_depths = valid_depths[obstacle_mask]
obstacle_distance = np.min(obstacle_depths)
# obstacle_distance = 1.40m
```

**Bước 6: Ước tính chiều cao**
```python
depth_diff = 1.50 - 1.40 = 0.10m
angle = 29.5°
height = 0.10 × sin(29.5°) = 0.10 × 0.492 = 0.049m ≈ 5cm
```

**Bước 7: Quyết định**
```python
max_step_height = 0.05m  # 5cm
can_step_over = (0.049 <= 0.05)  # True!

action = RAISE
recommended_height = 0.05 + 0.02 = 0.07m  # 7cm gầm
# Nhưng clamp tại max_ground_clearance = 0.10m
```

**Output:**
```python
TerrainAnalysisResult(
    action = ClearanceAction.RAISE,
    recommended_height = 0.07,  # 7cm
    ground_obstacle = True,
    obstacle_height = 0.049,    # 4.9cm
    obstacle_distance = 1.40,   # 1.4m phía trước
    can_step_over = True,
    message = "Chướng ngại 4.9cm - NÂNG GẦM lên 7cm"
)
```

---

### 9.2 Ví Dụ 2: Đi Qua Cửa Thấp

**Tình huống:** Robot cần đi qua cửa có độ cao giới hạn 35cm.

**Input:**
```python
# Ceiling zone (y=0 đến y=144):
# - Depth trung bình ≈ 0.6m (thấy mép trên cửa)
```

**Bước 1: Trích xuất ceiling zone**
```python
ceiling_zone = depth_frame[0:144, 64:576]  # Bỏ margin 2 bên
```

**Bước 2: Tính khoảng cách trần**
```python
valid_depths = ceiling_zone[(ceiling_zone > 0.1) & (ceiling_zone < 5.0)]
ceiling_distance = np.percentile(valid_depths, 10)
# ceiling_distance = 0.55m
```

**Bước 3: Kiểm tra clearance**
```python
robot_height = 0.25m  # Khi gầm 5cm, tổng cao 30cm
ceiling_min_clearance = 0.5m

clearance_ok = (0.55 >= 0.5)  # True, nhưng sát ngưỡng!
ceiling_detected = (0.55 < 1.5)  # True
```

**Bước 4: Tính height cần thiết**
```python
# Khoảng cách đến trần: 0.55m
# Buffer: 0.10m (10cm)
# Cho phép robot cao: 0.55 - 0.10 = 0.45m

# Robot body cao 0.20m
# Còn lại cho gầm: 0.45 - 0.20 = 0.25m
# Nhưng max gầm = 0.10m

# Vậy robot cao nhất = 0.20 + 0.10 = 0.30m
# So với trần 0.55m → đủ!

# Nhưng để an toàn, hạ xuống gầm thấp:
recommended_height = 0.02m  # 2cm (tối thiểu)
```

**Output:**
```python
TerrainAnalysisResult(
    action = ClearanceAction.LOWER,
    recommended_height = 0.02,  # 2cm
    ceiling_detected = True,
    ceiling_distance = 0.55,
    ceiling_clearance_ok = True,  # Vẫn đủ
    message = "Trần thấp 0.55m - hạ gầm 2cm cho an toàn"
)
```

---

## 10. Giới Hạn và Cải Tiến

### 10.1 Giới Hạn Hiện Tại

| Giới hạn | Mô tả | Ảnh hưởng |
|----------|-------|-----------|
| **Góc nghiêng cố định** | Giả sử camera nghiêng đúng 15° | Sai số nếu robot nghiêng |
| **Vật thẳng đứng** | Công thức giả sử vật thẳng đứng | Sai với vật nghiêng/tròn |
| **Mặt phẳng đều** | Giả sử mặt đất phẳng | Sai trên địa hình lồi lõm |
| **Depth camera range** | RealSense: 0.1m - 10m | Không detect ngoài range |
| **IR interference** | Ánh sáng mặt trời mạnh | Depth sai ngoài trời |

### 10.2 Hướng Cải Tiến

**1. Kết hợp IMU:**
```python
# Lấy góc nghiêng thực từ IMU
actual_tilt = imu.get_pitch()  # Thay vì giả định 15°
angle = actual_tilt + camera_vfov * 0.25
```

**2. Pixel-wise angle calculation:**
```python
# Tính góc chính xác cho từng pixel
def get_angle_at_pixel(y, H, tilt, fov):
    return -tilt + fov * (0.5 - y/H)

# Áp dụng cho từng điểm obstacle
for y, depth in obstacle_points:
    angle = get_angle_at_pixel(y, H, tilt, fov)
    height += depth_diff * sin(angle)
```

**3. Machine Learning approach:**
```python
# Train model để ước tính chiều cao chính xác hơn
model = HeightEstimationNet()
model.train(depth_patches, ground_truth_heights)

# Inference
height = model.predict(obstacle_depth_patch)
```

**4. Sensor fusion:**
```python
# Kết hợp nhiều sensor
height_depth = terrain_analyzer.get_height()  # Từ depth camera
height_lidar = lidar.get_obstacle_height()     # Từ LIDAR (nếu có)
height_ultrasonic = ultrasonic.get_height()    # Từ ultrasonic

# Weighted fusion
final_height = weighted_average([height_depth, height_lidar, height_ultrasonic],
                                weights=[0.5, 0.3, 0.2])
```

---

## Phụ Lục A: Bảng Thông Số

### A.1 Thông Số Camera

| Thông số | Giá trị | Đơn vị |
|----------|---------|--------|
| Model | Intel RealSense D435 | - |
| Depth Resolution | 640 × 480 | pixel |
| Depth FPS | 30 | Hz |
| Vertical FOV | 58 | độ |
| Horizontal FOV | 87 | độ |
| Depth Range | 0.1 - 10 | mét |
| Baseline | 50 | mm |

### A.2 Thông Số Robot

| Thông số | Giá trị | Đơn vị |
|----------|---------|--------|
| Chiều cao body | 0.20 | mét |
| Chiều cao camera | 0.20 | mét |
| Góc nghiêng camera | 15 | độ |
| Gầm tối thiểu | 0.02 | mét |
| Gầm tối đa | 0.10 | mét |
| Gầm bình thường | 0.05 | mét |

### A.3 Thông Số Thuật Toán

| Thông số | Giá trị | Mô tả |
|----------|---------|-------|
| ceiling_zone_bottom | 0.30 | Tỷ lệ frame cho ceiling zone |
| ground_zone_top | 0.55 | Tỷ lệ frame bắt đầu ground zone |
| obstacle_threshold | 0.08 m | Ngưỡng chênh lệch depth |
| max_step_height | 0.05 m | Chiều cao tối đa bước qua |
| smoothing_window | 5 | Số frame cho median filter |

---

## Phụ Lục B: Code Reference

### B.1 File Structure

```
src/perception/
├── terrain_analyzer.py      # Module chính
├── __init__.py             # Export

tools/testing/
├── test_terrain_analyzer.py # Tool test trực quan
```

### B.2 API Usage

```python
from src.perception import TerrainAnalyzer, TerrainConfig, ClearanceAction

# Khởi tạo
config = TerrainConfig(
    camera_tilt_angle=15.0,
    max_step_height=0.05
)
analyzer = TerrainAnalyzer(config)

# Phân tích
result = analyzer.analyze(depth_frame, color_frame)

# Sử dụng kết quả
if result.action == ClearanceAction.RAISE:
    robot.set_leg_height(result.recommended_height)
elif result.action == ClearanceAction.LOWER:
    robot.set_leg_height(result.recommended_height)
elif result.action == ClearanceAction.STOP:
    robot.emergency_stop()
```

---

*Tài liệu này được tạo cho dự án Autonomous Robot - Terrain Analyzer Module*
