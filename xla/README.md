# Hướng dẫn Calibration - Autonomous Robot

## Tổng quan

Hệ thống robot tự động sử dụng nhiều công cụ calibration để tinh chỉnh các thông số cho phù hợp với môi trường hoạt động thực tế. Mỗi công cụ phục vụ một mục đích cụ thể:

| Tool | File | Mục đích |
|------|------|----------|
| Camera Intrinsic | `camera_calibration.py` | Hiệu chỉnh méo ống kính |
| Depth Calibration | `depth_calibration.py` | Hiệu chỉnh độ chính xác depth |
| Lane Calibration | `lane_calibration.py` | Tinh chỉnh phát hiện lane |
| Terrain Obstacle | `terrain_obstacle_calibration.py` | Tinh chỉnh phát hiện vật cản |

## Thứ tự Calibration

Nên thực hiện calibration theo thứ tự sau:

1. **Camera Intrinsic** → Sửa méo ống kính trước
2. **Depth Calibration** → Hiệu chỉnh độ chính xác depth
3. **Terrain Obstacle** → Tinh chỉnh phát hiện vật cản địa hình
4. **Lane Calibration** → Tinh chỉnh phát hiện đường

## Cách chạy

```bash
cd autonomous_robot

# Camera intrinsic calibration
python tools/calibration/camera_calibration.py

# Depth calibration  
python tools/calibration/depth_calibration.py

# Lane calibration
python tools/calibration/lane_calibration.py

# Terrain obstacle calibration
python tools/calibration/terrain_obstacle_calibration.py
```

## File cấu hình được tạo

Các file calibration được lưu trong `data/calibration/`:

```
data/calibration/
├── camera_intrinsics.json    # Ma trận camera và distortion
├── depth_calibration.json    # Hệ số hiệu chỉnh depth
├── lane_params.json          # Thông số phát hiện lane
└── terrain_config.json       # Cấu hình phát hiện vật cản
```

## Chi tiết từng công cụ

- [Camera Intrinsic Calibration](CAMERA_CALIBRATION.md)
- [Depth Calibration](DEPTH_CALIBRATION.md)
- [Lane Calibration](LANE_CALIBRATION.md)
- [Terrain Obstacle Calibration](TERRAIN_CALIBRATION.md)
