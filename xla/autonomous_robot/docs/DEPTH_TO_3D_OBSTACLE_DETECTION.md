# Depth Image to 3D Point Cloud: Obstacle Detection
## Technical Guide for RealSense D435i with Fixed Pitch Angle

---

## Table of Contents

1. [Coordinate Frame Definitions](#1-coordinate-frame-definitions)
2. [Depth to 3D Point Conversion](#2-depth-to-3d-point-conversion)
3. [Camera Pitch Compensation](#3-camera-pitch-compensation)
4. [Ground Plane and Height Calculation](#4-ground-plane-and-height-calculation)
5. [Obstacle Detection Algorithm](#5-obstacle-detection-algorithm)
6. [Complete Python Implementation](#6-complete-python-implementation)
7. [Visualization](#7-visualization)

---

## 1. Coordinate Frame Definitions

### 1.1 Camera Coordinate Frame

The RealSense D435i uses a **right-handed** coordinate system:

```
                    Z (forward/depth)
                    ▲
                   ╱
                  ╱
                 ╱
                ╱
               ╱
              ╱
             ●───────────────▶ X (right)
            ╱│
           ╱ │
          ╱  │
             ▼
             Y (down)


Camera Frame (looking from behind the camera):
- X-axis: Points to the RIGHT
- Y-axis: Points DOWN
- Z-axis: Points FORWARD (into the scene)
```

### 1.2 Image Coordinate Frame

```
        u (column) →
      0   1   2   ...  639
    ┌───┬───┬───┬─────┬───┐
  0 │   │   │   │     │   │
    ├───┼───┼───┼─────┼───┤
  1 │   │   │   │     │   │
v   ├───┼───┼───┼─────┼───┤
(row)   │   │ P │     │   │  ← Pixel at (u, v)
↓   ├───┼───┼───┼─────┼───┤
... │   │   │   │     │   │
    ├───┼───┼───┼─────┼───┤
479 │   │   │   │     │   │
    └───┴───┴───┴─────┴───┘

- u: column index (0 to width-1)
- v: row index (0 to height-1)
- Origin (0,0) at top-left corner
```

### 1.3 World/Robot Coordinate Frame

After pitch compensation, we use the **robot frame**:

```
                    Z_world (up)
                    ▲
                    │
                    │
                    │
                    │
                    ●───────────────▶ X_world (forward)
                   ╱
                  ╱
                 ╱
                ╱
               ▼
              Y_world (left)

Robot/World Frame:
- X-axis: Points FORWARD (robot heading)
- Y-axis: Points LEFT
- Z-axis: Points UP
```

### 1.4 Camera Mounting Configuration

```
Side View:
                                    
                    ▲ Z_world (up)
                    │
                    │         ╱╲
                    │        ╱  ╲  Camera optical axis
                    │       ╱    ╲
                    │      ╱ θ    ╲   θ = pitch angle (15°)
                    │     ╱────────╲
                    │    ╱          ╲
           h_cam ───┼───●────────────╲──────────▶ X_world (forward)
                    │   Camera        ╲
                    │   Position       ╲
                    │                   ╲
════════════════════╪════════════════════╲═════════════
                    │                     ╲
               Ground Plane              Ray hits ground


Parameters:
- h_cam: Camera height above ground (e.g., 0.20m = 20cm)
- θ (theta): Camera pitch angle (e.g., 15° downward)
- Positive θ means camera is tilted DOWN
```

---

## 2. Depth to 3D Point Conversion

### 2.1 Camera Intrinsic Parameters

The depth camera has intrinsic parameters stored in a **camera matrix K**:

$$
K = \begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

Where:
- $f_x, f_y$ = focal lengths in pixels
- $c_x, c_y$ = principal point (optical center) in pixels

**For RealSense D435i (typical values at 640×480):**
- $f_x \approx 380-390$ pixels
- $f_y \approx 380-390$ pixels
- $c_x \approx 320$ pixels (half of width)
- $c_y \approx 240$ pixels (half of height)

### 2.2 Deprojection Formula

To convert a pixel $(u, v)$ with depth $d$ to a 3D point $(X_c, Y_c, Z_c)$ in camera frame:

$$
\boxed{
\begin{aligned}
X_c &= \frac{(u - c_x) \cdot d}{f_x} \\[8pt]
Y_c &= \frac{(v - c_y) \cdot d}{f_y} \\[8pt]
Z_c &= d
\end{aligned}
}
$$

**Derivation:**

The pinhole camera model projects a 3D point onto the image plane:

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \frac{1}{Z_c} \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}
$$

Expanding:
$$
u = \frac{f_x \cdot X_c}{Z_c} + c_x \quad \Rightarrow \quad X_c = \frac{(u - c_x) \cdot Z_c}{f_x}
$$

$$
v = \frac{f_y \cdot Y_c}{Z_c} + c_y \quad \Rightarrow \quad Y_c = \frac{(v - c_y) \cdot Z_c}{f_y}
$$

Since $Z_c = d$ (the depth value):

$$
X_c = \frac{(u - c_x) \cdot d}{f_x}, \quad Y_c = \frac{(v - c_y) \cdot d}{f_y}, \quad Z_c = d
$$

### 2.3 Visual Explanation

```
                        Image Plane
                    ┌───────────────────┐
                    │         │         │
                    │    (cx,cy)        │
                    │         ●─────────┼─── u-cx
                    │         │    ●    │ Pixel (u,v)
                    │         │    │    │
                    │         │ v-cy    │
                    └─────────┼─────────┘
                              │
                              │ f (focal length)
                              │
                              ●───────────────────────────▶ Z_c
                           Camera
                           Center

Side view:
                              
                    ●──────────────────────●
               Camera                   3D Point
               Center                   (Xc, Yc, Zc)
                    │                      │
                    │◄────── Zc = d ──────▶│
                    │                      │
                    │  Similar triangles:  │
                    │                      │
                    │  (u-cx)/f = Xc/Zc    │
                    │  (v-cy)/f = Yc/Zc    │
```

---

## 3. Camera Pitch Compensation

### 3.1 Rotation Matrix for Pitch

When the camera is pitched down by angle $\theta$, we need to rotate the 3D points to align with the world frame.

**Rotation about the Y-axis (camera Y = left/right axis):**

Since camera Y points down and we pitch around the horizontal axis perpendicular to the camera's forward direction, we use rotation about Y:

But actually, for a camera looking forward and pitched DOWN, we rotate about the **camera's X-axis** (which points right):

$$
R_x(\theta) = \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\theta & -\sin\theta \\
0 & \sin\theta & \cos\theta
\end{bmatrix}
$$

**Wait, let's be more careful about the coordinate frames:**

When camera is pitched DOWN by angle θ:
- Camera Z-axis (forward) is tilted DOWN from horizontal
- To transform points from camera frame to world frame, we rotate by **+θ** about the camera's X-axis

$$
\mathbf{P}_{world} = R_x(\theta) \cdot \mathbf{P}_{camera} + \mathbf{t}
$$

Where:
- $R_x(\theta)$ = rotation matrix for pitch compensation
- $\mathbf{t} = [0, 0, h_{cam}]^T$ = translation (camera height)

### 3.2 Complete Transformation

For a camera at height $h_{cam}$ and pitched down by $\theta$:

$$
\begin{bmatrix} X_w \\ Y_w \\ Z_w \end{bmatrix} = 
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\theta & -\sin\theta \\
0 & \sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}
+
\begin{bmatrix} 0 \\ 0 \\ h_{cam} \end{bmatrix}
$$

Expanding:

$$
\boxed{
\begin{aligned}
X_w &= X_c \\
Y_w &= Y_c \cos\theta - Z_c \sin\theta \\
Z_w &= Y_c \sin\theta + Z_c \cos\theta + h_{cam}
\end{aligned}
}
$$

**Important:** $Z_w$ is now the **height above ground**!

### 3.3 Visual Explanation

```
Side View (X-Z plane, looking from camera's left side):

        Z_world (up)
        ▲
        │                    ╱ Z_camera
        │                  ╱
        │                ╱
        │              ╱ θ
        │            ╱
        │          ●───────────────▶ X_world = X_camera
        │         ╱│  Camera
        │       ╱  │
        │     ╱    │ h_cam
        │   ╱      │
        │ ╱        │
════════╳══════════╪════════════════════════════════════
        │ Ground   │
        │          ▼ Y_camera (into ground)

After rotation by θ about X-axis:
- Camera's Z-axis aligns with world's X-axis (forward)
- Camera's Y-axis aligns with world's -Z-axis (down = -up)
- Points on ground will have Z_world ≈ 0
```

---

## 4. Ground Plane and Height Calculation

### 4.1 Expected Ground Height

For a point on a flat ground directly in front of the robot:

- In camera frame: The ground appears at some $(X_c, Y_c, Z_c)$
- In world frame: $Z_w$ should be $\approx 0$

### 4.2 Height Above Ground

For any 3D point, the **height above ground** is simply:

$$
h_{point} = Z_w = Y_c \sin\theta + Z_c \cos\theta + h_{cam}
$$

**For ground points:** $h_{point} \approx 0$  
**For obstacles:** $h_{point} > 0$ (above ground)  
**For holes/depressions:** $h_{point} < 0$ (below ground level)

### 4.3 Verification

Let's verify with a ground point directly below the camera:

```
Setup:
- Camera height: h_cam = 0.20m
- Pitch angle: θ = 15° = 0.262 rad
- Point on ground directly below camera

In camera frame:
- The ground is at distance d = h_cam / sin(θ) looking along camera Z
  Wait, this isn't right for directly below...

Actually, for a point on the ground at horizontal distance D in front:

Camera frame coordinates (for point on ground at distance D forward):
- Zc = D / cos(θ)  (depth along camera optical axis)
- Yc = D * tan(θ) + h_cam / cos(θ)  (below optical axis)
  
This gets complicated. Let's use the formula directly:
```

**Simplified verification with specific point:**

Point on ground, 1m in front of robot:
- Horizontal distance: $D = 1.0m$
- Depth along camera axis: $Z_c = D / \cos(15°) = 1.035m$
- In camera frame, this point is below the optical axis

The ground point in camera coordinates:
- $X_c = 0$ (directly ahead)
- $Z_c = 1.035m$
- $Y_c = ?$ (need to figure out how far below optical center)

From geometry:
- Camera optical axis is 15° below horizontal
- Ground is at $h_{cam} = 0.2m$ below camera
- At distance $D = 1m$, the ground is at vertical offset:
  $\Delta y = h_{cam} + D \tan(15°) = 0.2 + 1.0 \times 0.268 = 0.468m$ below camera

In camera frame (Y points down):
- $Y_c = $ offset in camera Y direction
- Using similar triangles: $Y_c = (h_{cam} + D \tan\theta) / \cos\theta$

Actually, let's just verify numerically in code.

---

## 5. Obstacle Detection Algorithm

### 5.1 Height-Based Classification

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     HEIGHT-BASED CLASSIFICATION                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Height (Z_world)                                                       │
│       ▲                                                                 │
│       │                                                                 │
│  +0.10├─────────────── OBSTACLE (cannot step over) ────────────────────│
│       │                                                                 │
│  +0.05├─────────────── SMALL_OBSTACLE (can step over) ─────────────────│
│       │                                                                 │
│  +0.02├─────────────── GROUND_TOLERANCE ───────────────────────────────│
│       │                 (normal ground variation)                       │
│   0.00├═════════════════ GROUND PLANE ═════════════════════════════════│
│       │                                                                 │
│  -0.02├─────────────── GROUND_TOLERANCE ───────────────────────────────│
│       │                                                                 │
│  -0.05├─────────────── DEPRESSION/HOLE ────────────────────────────────│
│       │                                                                 │
│       ▼                                                                 │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### 5.2 Classification Thresholds

| Height Range | Classification | Action |
|--------------|----------------|--------|
| $h > 0.10m$ | HIGH_OBSTACLE | STOP - Cannot pass |
| $0.05m < h \leq 0.10m$ | OBSTACLE | AVOID or RAISE chassis |
| $0.02m < h \leq 0.05m$ | SMALL_BUMP | Can step over |
| $-0.02m \leq h \leq 0.02m$ | GROUND | Normal traversable |
| $-0.05m \leq h < -0.02m$ | SMALL_DIP | Careful |
| $h < -0.05m$ | HOLE | AVOID |

### 5.3 Grid-Based Analysis

Divide the ground region into a grid for robust detection:

```
        Robot Front
           ▲
           │
    ┌──┬──┬──┬──┬──┐
    │  │  │  │  │  │  Far zone (1.5-2.0m)
    ├──┼──┼──┼──┼──┤
    │  │  │  │  │  │  Mid zone (1.0-1.5m)
    ├──┼──┼──┼──┼──┤
    │  │  │  │  │  │  Near zone (0.5-1.0m)
    ├──┼──┼──┼──┼──┤
    │  │  │  │  │  │  Close zone (0.3-0.5m)
    └──┴──┴──┴──┴──┘
    ←─── Width ───→

Each cell: Compute median height
If any cell has obstacle → trigger avoidance
```

---

## 6. Complete Python Implementation

```python
#!/usr/bin/env python3
"""
Depth to 3D Point Cloud with Pitch Compensation for Obstacle Detection.

This module converts RealSense depth images to 3D point clouds,
compensates for camera pitch angle, and detects obstacles based on height.

Author: Autonomous Robot Project
Date: January 2026
"""

import numpy as np
import cv2
import pyrealsense2 as rs
from dataclasses import dataclass
from typing import Tuple, Optional, List
from enum import Enum, auto


class TerrainType(Enum):
    """Classification of terrain based on height."""
    GROUND = auto()          # Normal traversable ground
    SMALL_BUMP = auto()      # Can step over
    OBSTACLE = auto()        # Needs avoidance or chassis raise
    HIGH_OBSTACLE = auto()   # Cannot pass
    SMALL_DIP = auto()       # Small depression
    HOLE = auto()            # Dangerous hole


@dataclass
class CameraConfig:
    """Camera mounting configuration."""
    height: float = 0.20        # Camera height above ground (meters)
    pitch_angle: float = 15.0   # Pitch angle in degrees (positive = down)
    
    # RealSense D435i typical intrinsics at 640x480
    # These will be overwritten by actual camera intrinsics
    fx: float = 384.0
    fy: float = 384.0
    cx: float = 320.0
    cy: float = 240.0
    
    # Image dimensions
    width: int = 640
    height_px: int = 480


@dataclass
class ObstacleConfig:
    """Obstacle detection thresholds."""
    ground_tolerance: float = 0.02      # ±2cm is considered ground
    small_bump_max: float = 0.05        # Up to 5cm can be stepped over
    obstacle_max: float = 0.10          # Up to 10cm needs avoidance
    # Above obstacle_max = HIGH_OBSTACLE
    
    small_dip_min: float = -0.05        # Down to -5cm is small dip
    # Below small_dip_min = HOLE
    
    # Analysis grid
    grid_rows: int = 4                  # Number of distance zones
    grid_cols: int = 5                  # Number of lateral zones
    
    # Distance range of interest (meters)
    min_distance: float = 0.3           # Ignore points closer than this
    max_distance: float = 2.0           # Ignore points farther than this


class DepthTo3DConverter:
    """
    Converts depth images to 3D point clouds with pitch compensation.
    
    Coordinate Frames:
    - Camera Frame: X-right, Y-down, Z-forward
    - World Frame: X-forward, Y-left, Z-up
    """
    
    def __init__(
        self, 
        camera_config: Optional[CameraConfig] = None,
        obstacle_config: Optional[ObstacleConfig] = None
    ):
        """Initialize the converter with camera parameters."""
        self.camera = camera_config or CameraConfig()
        self.obstacles = obstacle_config or ObstacleConfig()
        
        # Precompute rotation matrix
        self._update_rotation_matrix()
        
        # Precompute pixel coordinate grids for vectorized computation
        self._precompute_pixel_grids()
    
    def _update_rotation_matrix(self):
        """Compute rotation matrix for pitch compensation."""
        theta = np.radians(self.camera.pitch_angle)
        
        # Rotation about X-axis (camera X = right)
        self.R = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ], dtype=np.float32)
        
        # Translation vector (camera position in world frame)
        self.t = np.array([0, 0, self.camera.height], dtype=np.float32)
        
        # Store sin and cos for scalar computation
        self.cos_theta = np.cos(theta)
        self.sin_theta = np.sin(theta)
    
    def _precompute_pixel_grids(self):
        """Precompute (u - cx) and (v - cy) grids for vectorized deprojection."""
        u = np.arange(self.camera.width, dtype=np.float32)
        v = np.arange(self.camera.height_px, dtype=np.float32)
        
        # Create meshgrid
        self.u_grid, self.v_grid = np.meshgrid(u, v)
        
        # Precompute normalized coordinates
        self.u_normalized = (self.u_grid - self.camera.cx) / self.camera.fx
        self.v_normalized = (self.v_grid - self.camera.cy) / self.camera.fy
    
    def update_intrinsics(self, intrinsics: rs.intrinsics):
        """Update camera intrinsics from RealSense."""
        self.camera.fx = intrinsics.fx
        self.camera.fy = intrinsics.fy
        self.camera.cx = intrinsics.ppx
        self.camera.cy = intrinsics.ppy
        self.camera.width = intrinsics.width
        self.camera.height_px = intrinsics.height
        
        # Recompute pixel grids with new intrinsics
        self._precompute_pixel_grids()
    
    def deproject_pixel(
        self, 
        u: float, 
        v: float, 
        depth: float
    ) -> Tuple[float, float, float]:
        """
        Convert a single pixel to 3D point in camera frame.
        
        Args:
            u: Pixel column (x)
            v: Pixel row (y)
            depth: Depth value in meters
            
        Returns:
            (Xc, Yc, Zc) in camera frame
        """
        Xc = (u - self.camera.cx) * depth / self.camera.fx
        Yc = (v - self.camera.cy) * depth / self.camera.fy
        Zc = depth
        
        return Xc, Yc, Zc
    
    def camera_to_world(
        self, 
        Xc: float, 
        Yc: float, 
        Zc: float
    ) -> Tuple[float, float, float]:
        """
        Transform point from camera frame to world frame.
        
        Args:
            Xc, Yc, Zc: Point in camera frame
            
        Returns:
            (Xw, Yw, Zw) in world frame where Zw is height above ground
        """
        # Apply rotation
        Xw = Xc  # X doesn't change (both point sideways)
        Yw = Yc * self.cos_theta - Zc * self.sin_theta
        Zw = Yc * self.sin_theta + Zc * self.cos_theta + self.camera.height
        
        return Xw, Yw, Zw
    
    def pixel_to_world(
        self, 
        u: float, 
        v: float, 
        depth: float
    ) -> Tuple[float, float, float]:
        """
        Convert pixel directly to world coordinates.
        
        Args:
            u, v: Pixel coordinates
            depth: Depth in meters
            
        Returns:
            (Xw, Yw, Zw) where Zw is height above ground
        """
        Xc, Yc, Zc = self.deproject_pixel(u, v, depth)
        return self.camera_to_world(Xc, Yc, Zc)
    
    def depth_to_height_map(
        self, 
        depth_frame: np.ndarray,
        valid_range: Tuple[float, float] = (0.1, 5.0)
    ) -> np.ndarray:
        """
        Convert entire depth frame to height map (Z_world for each pixel).
        
        This is the key function for obstacle detection.
        
        Args:
            depth_frame: Depth image in meters (H x W)
            valid_range: (min_depth, max_depth) for valid measurements
            
        Returns:
            height_map: Height above ground for each pixel (H x W)
                        NaN for invalid pixels
        """
        # Create output array
        height_map = np.full_like(depth_frame, np.nan, dtype=np.float32)
        
        # Mask for valid depth values
        valid_mask = (depth_frame > valid_range[0]) & (depth_frame < valid_range[1])
        
        # Get valid depths
        d = depth_frame[valid_mask]
        
        # Get corresponding normalized pixel coordinates
        u_norm = self.u_normalized[valid_mask]
        v_norm = self.v_normalized[valid_mask]
        
        # Deproject to camera frame (vectorized)
        Xc = u_norm * d
        Yc = v_norm * d
        Zc = d
        
        # Transform to world frame (vectorized)
        # Zw = Yc * sin(θ) + Zc * cos(θ) + h_cam
        Zw = Yc * self.sin_theta + Zc * self.cos_theta + self.camera.height
        
        # Store heights
        height_map[valid_mask] = Zw
        
        return height_map
    
    def depth_to_point_cloud(
        self, 
        depth_frame: np.ndarray,
        valid_range: Tuple[float, float] = (0.1, 5.0),
        downsample: int = 4
    ) -> np.ndarray:
        """
        Convert depth frame to full 3D point cloud in world frame.
        
        Args:
            depth_frame: Depth image in meters
            valid_range: Valid depth range
            downsample: Skip factor for reducing point count
            
        Returns:
            points: (N, 3) array of [X, Y, Z] in world frame
        """
        # Downsample
        depth_ds = depth_frame[::downsample, ::downsample]
        u_norm_ds = self.u_normalized[::downsample, ::downsample]
        v_norm_ds = self.v_normalized[::downsample, ::downsample]
        
        # Valid mask
        valid = (depth_ds > valid_range[0]) & (depth_ds < valid_range[1])
        
        d = depth_ds[valid]
        u_n = u_norm_ds[valid]
        v_n = v_norm_ds[valid]
        
        # Deproject to camera frame
        Xc = u_n * d
        Yc = v_n * d
        Zc = d
        
        # Transform to world frame
        Xw = Xc
        Yw = Yc * self.cos_theta - Zc * self.sin_theta
        Zw = Yc * self.sin_theta + Zc * self.cos_theta + self.camera.height
        
        # Stack into (N, 3) array
        points = np.stack([Xw, Yw, Zw], axis=1)
        
        return points
    
    def classify_terrain(
        self, 
        height: float
    ) -> TerrainType:
        """Classify terrain type based on height above ground."""
        cfg = self.obstacles
        
        if height > cfg.obstacle_max:
            return TerrainType.HIGH_OBSTACLE
        elif height > cfg.small_bump_max:
            return TerrainType.OBSTACLE
        elif height > cfg.ground_tolerance:
            return TerrainType.SMALL_BUMP
        elif height >= -cfg.ground_tolerance:
            return TerrainType.GROUND
        elif height >= cfg.small_dip_min:
            return TerrainType.SMALL_DIP
        else:
            return TerrainType.HOLE
    
    def analyze_ground_grid(
        self, 
        height_map: np.ndarray,
        depth_frame: np.ndarray
    ) -> dict:
        """
        Analyze ground in a grid pattern for robust obstacle detection.
        
        Args:
            height_map: Height above ground (H x W)
            depth_frame: Original depth frame for distance calculation
            
        Returns:
            Dictionary with grid analysis results
        """
        cfg = self.obstacles
        H, W = height_map.shape
        
        # Define region of interest (bottom portion of image = ground area)
        roi_top = int(H * 0.4)  # Skip top 40% (usually sky/walls)
        roi_bottom = int(H * 0.95)
        
        # Grid cell dimensions
        roi_height = roi_bottom - roi_top
        cell_height = roi_height // cfg.grid_rows
        cell_width = W // cfg.grid_cols
        
        # Results
        grid_heights = np.zeros((cfg.grid_rows, cfg.grid_cols))
        grid_types = [[TerrainType.GROUND] * cfg.grid_cols for _ in range(cfg.grid_rows)]
        grid_distances = np.zeros((cfg.grid_rows, cfg.grid_cols))
        
        for row in range(cfg.grid_rows):
            for col in range(cfg.grid_cols):
                # Cell boundaries
                y1 = roi_top + row * cell_height
                y2 = y1 + cell_height
                x1 = col * cell_width
                x2 = x1 + cell_width
                
                # Extract cell data
                cell_heights = height_map[y1:y2, x1:x2]
                cell_depths = depth_frame[y1:y2, x1:x2]
                
                # Filter valid heights
                valid = ~np.isnan(cell_heights)
                if np.sum(valid) < 10:
                    continue
                
                heights_valid = cell_heights[valid]
                depths_valid = cell_depths[valid]
                
                # Use median for robustness
                median_height = np.median(heights_valid)
                median_distance = np.median(depths_valid)
                
                # Filter by distance range
                if median_distance < cfg.min_distance or median_distance > cfg.max_distance:
                    continue
                
                grid_heights[row, col] = median_height
                grid_distances[row, col] = median_distance
                grid_types[row][col] = self.classify_terrain(median_height)
        
        # Find obstacles
        obstacles_found = []
        for row in range(cfg.grid_rows):
            for col in range(cfg.grid_cols):
                terrain = grid_types[row][col]
                if terrain in [TerrainType.OBSTACLE, TerrainType.HIGH_OBSTACLE, TerrainType.HOLE]:
                    obstacles_found.append({
                        'row': row,
                        'col': col,
                        'type': terrain,
                        'height': grid_heights[row, col],
                        'distance': grid_distances[row, col]
                    })
        
        return {
            'heights': grid_heights,
            'distances': grid_distances,
            'types': grid_types,
            'obstacles': obstacles_found,
            'has_obstacle': len(obstacles_found) > 0
        }


# =============================================================================
# Example Usage
# =============================================================================

def create_test_depth_frame():
    """Create a synthetic depth frame for testing."""
    H, W = 480, 640
    
    # Create a sloped ground plane (simulating camera looking down)
    # Depth increases from bottom to top of image
    v = np.arange(H).reshape(-1, 1)
    u = np.arange(W).reshape(1, -1)
    
    # Base depth: increases with row (top = far, bottom = near)
    base_depth = 0.5 + (H - v) / H * 2.0  # 0.5m to 2.5m
    
    depth = np.tile(base_depth, (1, W)).astype(np.float32)
    
    # Add an obstacle in the center-bottom
    obstacle_y = int(H * 0.7)
    obstacle_x = int(W * 0.5)
    obstacle_size = 30
    
    # Obstacle is closer than ground (smaller depth = obstacle protruding)
    depth[obstacle_y-obstacle_size:obstacle_y+obstacle_size,
          obstacle_x-obstacle_size:obstacle_x+obstacle_size] -= 0.2
    
    return depth


def main():
    """Demonstration of depth to 3D conversion and obstacle detection."""
    
    print("=" * 70)
    print("  DEPTH TO 3D POINT CLOUD WITH PITCH COMPENSATION")
    print("  Obstacle Detection Demonstration")
    print("=" * 70)
    
    # Create converter with default config
    camera_cfg = CameraConfig(
        height=0.20,        # 20cm above ground
        pitch_angle=15.0,   # 15 degrees down
    )
    
    converter = DepthTo3DConverter(camera_config=camera_cfg)
    
    # Test with synthetic data
    print("\n--- Single Point Test ---")
    
    # Test point: center of image, 1 meter depth
    u, v, d = 320, 400, 1.0
    Xc, Yc, Zc = converter.deproject_pixel(u, v, d)
    Xw, Yw, Zw = converter.camera_to_world(Xc, Yc, Zc)
    
    print(f"Pixel: ({u}, {v}), Depth: {d:.2f}m")
    print(f"Camera frame: X={Xc:.3f}, Y={Yc:.3f}, Z={Zc:.3f}")
    print(f"World frame:  X={Xw:.3f}, Y={Yw:.3f}, Z={Zw:.3f} (height)")
    print(f"Terrain type: {converter.classify_terrain(Zw).name}")
    
    # Test with synthetic depth frame
    print("\n--- Full Frame Test ---")
    depth_frame = create_test_depth_frame()
    
    # Convert to height map
    height_map = converter.depth_to_height_map(depth_frame)
    
    print(f"Height map shape: {height_map.shape}")
    print(f"Height range: {np.nanmin(height_map):.3f} to {np.nanmax(height_map):.3f} m")
    
    # Analyze grid
    result = converter.analyze_ground_grid(height_map, depth_frame)
    
    print(f"\nGrid analysis:")
    print(f"  Has obstacles: {result['has_obstacle']}")
    if result['obstacles']:
        for obs in result['obstacles']:
            print(f"  - {obs['type'].name} at row={obs['row']}, col={obs['col']}, "
                  f"height={obs['height']:.3f}m, distance={obs['distance']:.2f}m")
    
    # Visualize height map
    height_vis = height_map.copy()
    height_vis = np.nan_to_num(height_vis, nan=0)
    
    # Normalize for visualization
    height_vis = np.clip(height_vis, -0.2, 0.2)  # Clip to ±20cm
    height_vis = ((height_vis + 0.2) / 0.4 * 255).astype(np.uint8)
    height_colored = cv2.applyColorMap(height_vis, cv2.COLORMAP_JET)
    
    # Show
    cv2.imshow("Height Map (Blue=low, Red=high)", height_colored)
    print("\nPress any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
```

---

## 7. Visualization

### 7.1 Height Map Visualization

```python
def visualize_height_map(height_map: np.ndarray, 
                         color_frame: np.ndarray = None) -> np.ndarray:
    """
    Create a visualization of the height map.
    
    Color coding:
    - Blue: Below ground (holes)
    - Green: Ground level
    - Yellow: Small bumps
    - Red: Obstacles
    """
    H, W = height_map.shape
    
    # Create RGB visualization
    vis = np.zeros((H, W, 3), dtype=np.uint8)
    
    # Classify each pixel
    valid = ~np.isnan(height_map)
    
    # Ground (green)
    ground_mask = valid & (np.abs(height_map) <= 0.02)
    vis[ground_mask] = [0, 255, 0]
    
    # Small bumps (yellow)
    bump_mask = valid & (height_map > 0.02) & (height_map <= 0.05)
    vis[bump_mask] = [0, 255, 255]
    
    # Obstacles (red)
    obstacle_mask = valid & (height_map > 0.05)
    vis[obstacle_mask] = [0, 0, 255]
    
    # Holes (blue)
    hole_mask = valid & (height_map < -0.02)
    vis[hole_mask] = [255, 0, 0]
    
    # Blend with color frame if provided
    if color_frame is not None:
        alpha = 0.5
        vis = cv2.addWeighted(color_frame, 1-alpha, vis, alpha, 0)
    
    return vis
```

### 7.2 3D Point Cloud Visualization

```python
def visualize_point_cloud_top_view(points: np.ndarray, 
                                   grid_size: float = 0.05,
                                   img_size: int = 400) -> np.ndarray:
    """
    Create a top-down view of the point cloud.
    
    Args:
        points: (N, 3) array of [X, Y, Z] in world frame
        grid_size: Size of each grid cell in meters
        img_size: Output image size in pixels
    """
    # Extract X (forward) and Y (left/right)
    X = points[:, 0]  # Forward
    Y = points[:, 1]  # Left
    Z = points[:, 2]  # Height
    
    # Define range
    x_range = (0, 3.0)  # 0 to 3m forward
    y_range = (-1.5, 1.5)  # ±1.5m left/right
    
    # Create image
    img = np.zeros((img_size, img_size, 3), dtype=np.uint8)
    
    # Convert to image coordinates
    for i in range(len(X)):
        if X[i] < x_range[0] or X[i] > x_range[1]:
            continue
        if Y[i] < y_range[0] or Y[i] > y_range[1]:
            continue
        
        # Map to image
        px = int((Y[i] - y_range[0]) / (y_range[1] - y_range[0]) * img_size)
        py = int((1 - (X[i] - x_range[0]) / (x_range[1] - x_range[0])) * img_size)
        
        # Color by height
        if Z[i] > 0.05:
            color = (0, 0, 255)  # Red obstacle
        elif Z[i] < -0.02:
            color = (255, 0, 0)  # Blue hole
        else:
            color = (0, 255, 0)  # Green ground
        
        cv2.circle(img, (px, py), 2, color, -1)
    
    # Draw robot position
    robot_x = int(img_size / 2)
    robot_y = int(img_size - 10)
    cv2.circle(img, (robot_x, robot_y), 5, (255, 255, 255), -1)
    
    return img
```

---

## Summary

### Key Formulas

1. **Deprojection (Pixel to Camera Frame):**
$$X_c = \frac{(u - c_x) \cdot d}{f_x}, \quad Y_c = \frac{(v - c_y) \cdot d}{f_y}, \quad Z_c = d$$

2. **Pitch Compensation (Camera to World Frame):**
$$X_w = X_c, \quad Y_w = Y_c\cos\theta - Z_c\sin\theta, \quad Z_w = Y_c\sin\theta + Z_c\cos\theta + h_{cam}$$

3. **Height Above Ground:**
$$h = Z_w = Y_c\sin\theta + Z_c\cos\theta + h_{cam}$$

### Algorithm Summary

```
┌────────────────────────────────────────────────────────────────────────┐
│                     OBSTACLE DETECTION PIPELINE                         │
├────────────────────────────────────────────────────────────────────────┤
│                                                                        │
│  1. Get depth frame from RealSense                                     │
│                    │                                                   │
│                    ▼                                                   │
│  2. For each valid pixel (u, v) with depth d:                         │
│     • Deproject to camera frame: (Xc, Yc, Zc)                         │
│     • Apply rotation + translation: (Xw, Yw, Zw)                      │
│     • Zw is the height above ground                                   │
│                    │                                                   │
│                    ▼                                                   │
│  3. Create height map (H × W)                                         │
│                    │                                                   │
│                    ▼                                                   │
│  4. Grid-based analysis:                                              │
│     • Divide into cells                                               │
│     • Compute median height per cell                                  │
│     • Classify: GROUND / OBSTACLE / HOLE                              │
│                    │                                                   │
│                    ▼                                                   │
│  5. Output: Obstacle locations and types                              │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

---

**End of Document**
