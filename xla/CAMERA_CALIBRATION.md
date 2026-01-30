# Camera Intrinsic Calibration

## Mục đích

Hiệu chỉnh các thông số nội tại (intrinsic parameters) của camera để sửa méo ống kính (lens distortion). Đây là bước quan trọng đầu tiên vì méo ống kính ảnh hưởng đến tất cả các thuật toán xử lý ảnh sau đó.

## Khi nào cần calibrate?

- Lần đầu sử dụng camera mới
- Sau khi thay đổi lens
- Khi phát hiện đường thẳng bị cong trong ảnh
- Khi kết quả lane detection không chính xác

## Cách sử dụng

```bash
python tools/calibration/camera_calibration.py
```

### Chuẩn bị

1. **In bảng checkerboard**: In bảng có 9x6 ô vuông (A4 khuyến nghị)
2. **Kích thước ô**: Mặc định 25mm, điều chỉnh trong code nếu khác

### Quy trình

1. Đặt bảng checkerboard trước camera
2. Di chuyển bảng đến các vị trí và góc khác nhau
3. Nhấn **SPACE** để capture khi phát hiện được góc (hiện màu xanh)
4. Thu thập 15-20 ảnh ở các góc độ khác nhau
5. Nhấn **C** để bắt đầu calibration
6. Nhấn **S** để lưu kết quả

### Phím tắt

| Phím | Chức năng |
|------|-----------|
| SPACE | Capture ảnh calibration |
| C | Bắt đầu calibration |
| S | Lưu kết quả |
| Q | Thoát |

## Thông số Output

### Camera Matrix (Ma trận camera)

```
     [fx  0  cx]
K =  [0  fy  cy]
     [0   0   1]
```

| Thông số | Ý nghĩa | Đơn vị |
|----------|---------|--------|
| `fx` | Focal length theo trục X | pixel |
| `fy` | Focal length theo trục Y | pixel |
| `cx` | Principal point X (tâm ảnh) | pixel |
| `cy` | Principal point Y (tâm ảnh) | pixel |

### Distortion Coefficients (Hệ số méo)

```
dist_coeffs = [k1, k2, p1, p2, k3]
```

| Thông số | Ý nghĩa | Ảnh hưởng |
|----------|---------|-----------|
| `k1, k2, k3` | Radial distortion | Méo hình thùng/gối |
| `p1, p2` | Tangential distortion | Méo tiếp tuyến |

### Reprojection Error

- **Giá trị tốt**: < 0.5 pixel
- **Chấp nhận được**: < 1.0 pixel  
- **Cần calibrate lại**: > 1.0 pixel

## Ảnh hưởng của thông số

### Focal Length (fx, fy)

```
Focal length lớn → FOV hẹp, zoom in
Focal length nhỏ → FOV rộng, zoom out
```

- Ảnh hưởng đến tỷ lệ pixel-to-meter
- Quan trọng cho tính toán khoảng cách

### Principal Point (cx, cy)

```
Nếu cx ≠ width/2 hoặc cy ≠ height/2:
→ Tâm quang học lệch khỏi tâm ảnh
→ Gây sai số trong lane detection
```

### Radial Distortion (k1, k2, k3)

```
k > 0 → Barrel distortion (phình ra)
k < 0 → Pincushion distortion (lõm vào)

Đường thẳng thực tế → Đường cong trong ảnh
```

**Hình minh họa:**
```
Barrel (k>0):        Pincushion (k<0):
   ╭──────╮             ╰──────╯
   │      │             │      │
   │      │             │      │
   ╰──────╯             ╭──────╮
```

## File Output

`data/calibration/camera_intrinsics.json`:

```json
{
  "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "dist_coeffs": [k1, k2, p1, p2, k3],
  "reprojection_error": 0.35,
  "image_size": [640, 480],
  "timestamp": "2024-01-15T10:30:00"
}
```

## Tips

1. **Đa dạng góc nhìn**: Capture bảng ở góc nghiêng 15-45°
2. **Đa dạng khoảng cách**: Gần (0.3m) đến xa (1.5m)
3. **Chiếu sáng đều**: Tránh bóng đổ trên bảng
4. **Bảng phẳng**: Đảm bảo bảng không bị cong
5. **Focus**: Đảm bảo ảnh rõ nét trước khi capture

## Troubleshooting

| Vấn đề | Nguyên nhân | Giải pháp |
|--------|-------------|-----------|
| Không detect được góc | Ánh sáng kém / blur | Cải thiện chiếu sáng |
| Error cao (>1px) | Ảnh capture không tốt | Capture lại với ảnh rõ hơn |
| fx ≠ fy nhiều | Pixel không vuông | Kiểm tra cài đặt camera |
