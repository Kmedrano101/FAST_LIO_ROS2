# Extrinsic Calibration Guide: Dual MID-360 LiDAR Setup

## Table of Contents
1. [Overview](#overview)
2. [FAST_LIO_MULTI Extrinsic Approach](#fast_lio_multi-extrinsic-approach)
3. [Current System Configuration](#current-system-configuration)
4. [Coordinate Frame Analysis](#coordinate-frame-analysis)
5. [Converting Livox to FAST-LIO Extrinsics](#converting-livox-to-fast-lio-extrinsics)
6. [Verification and Validation](#verification-and-validation)

---

## Overview

This document analyzes how [FAST_LIO_MULTI](https://github.com/engcang/FAST_LIO_MULTI) handles extrinsic calibration for dual LiDAR setups and provides a conversion guide for our dual MID-360 configuration.

### Key Concepts

**Extrinsic Parameters**: The spatial relationship (translation + rotation) between sensor coordinate frames.

**Reference Frame**: The coordinate system to which all other frames are expressed relative to.

---

## FAST_LIO_MULTI Extrinsic Approach

### Reference Frame: IMU Body Frame

FAST-LIO uses the **IMU body frame** as the primary reference frame. All extrinsic parameters describe sensor poses **relative to the IMU**.

From the [original FAST-LIO documentation](https://github.com/hku-mars/FAST_LIO):
> "The extrinsic parameters represent the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame)."

### Multi-LiDAR Extrinsic Structure

FAST_LIO_MULTI extends FAST-LIO with three sets of extrinsic parameters:

```yaml
mapping:
  # 1. PRIMARY LIDAR → IMU/Base Frame
  extrinsic_T: [x, y, z]        # Translation in meters
  extrinsic_R: [r11, r12, r13,  # 3x3 Rotation matrix (row-major)
                r21, r22, r23,
                r31, r32, r33]

  # 2. SECONDARY LIDAR → IMU/Base Frame
  extrinsic_T2: [x, y, z]
  extrinsic_R2: [r11, r12, r13,
                 r21, r22, r23,
                 r31, r32, r33]

  # 3. RELATIVE TRANSFORM: LiDAR2 w.r.t. LiDAR1
  extrinsic_T_L2_wrt_L1: [x, y, z]
  extrinsic_R_L2_wrt_L1: [r11, r12, r13,
                          r21, r22, r23,
                          r31, r32, r33]

  # 4. OPTIONAL: LiDAR1 w.r.t. Drone (for visualization/TF)
  extrinsic_T_L1_wrt_drone: [x, y, z]
  extrinsic_R_L1_wrt_drone: [...]
```

### FAST_LIO_MULTI Example Configuration

From their `config/multi.yaml`:

```yaml
# Primary LiDAR to IMU
extrinsic_T: [-0.011, -0.02329, 0.04412]
extrinsic_R: [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]  # Identity (no rotation)

# Secondary LiDAR to IMU
# (Configuration shows they use relative transform L2_wrt_L1)
extrinsic_T_L2_wrt_L1: [0.128311, 0.0, -0.096689]
extrinsic_R_L2_wrt_L1: [-0.2756374,  0.0,  0.9612617,    # ~16° pitch rotation
                         0.0,       -1.0,  0.0,
                         0.9612617,  0.0,  0.2756374]
```

**Key Observations:**
- Their sensors are tilted ±143° for wide FOV coverage
- They provide both absolute (to IMU) and relative (L2 w.r.t. L1) transforms
- Rotation matrices are in row-major format

---

## Current System Configuration

### Hardware Setup

- **Platform**: NVIDIA Jetson ORIN
- **Sensors**: 2x Livox MID-360
- **Reference Frame**: **IMU body frame** (LiDAR 1's built-in IMU) - Following FAST_LIO_MULTI convention
- **Sensor IPs**: 192.168.1.10 (LiDAR 1 with IMU reference), 192.168.1.18 (LiDAR 2)

### Livox Driver Configuration

From `/home/jetson/ros2_ws/src/livox_ros_driver2/config/multiple_netconfigs.json`:

```json
"lidar_configs": [
  {
    "ip": "192.168.1.10",
    "extrinsic_parameter": {
      "roll": 90.0,      // Euler angle (degrees)
      "pitch": 0.0,
      "yaw": 180.0,
      "x": 0,            // Translation (millimeters)
      "y": 110,
      "z": 0
    }
  },
  {
    "ip": "192.168.1.18",
    "extrinsic_parameter": {
      "roll": 90.0,
      "pitch": 0.0,
      "yaw": 0.0,
      "x": 0,
      "y": -110,
      "z": 0
    }
  }
]
```

**Key Points:**
- Extrinsics are in **Euler angles (degrees)** + **translation (mm)**
- Reference frame: **`base_link`**
- LiDAR 1: 110mm to the left (+Y), rotated 90° roll + 180° yaw (facing backward)
- LiDAR 2: 110mm to the right (-Y), rotated 90° roll only (facing forward)
- Total baseline: 220mm lateral separation

### Current FAST-LIO Configuration

From `/home/kmedrano/ros2_ws/src/fast_lio_ros2/config/dual_mid360_mine.yaml`:

**Using IMU Body Frame as Reference (FAST_LIO_MULTI approach):**

```yaml
mapping:
  # REFERENCE FRAME: IMU body frame (LiDAR 1's built-in IMU at 192.168.1.10)

  # LiDAR 1 Extrinsics (to IMU body frame)
  # IDENTITY: LiDAR 1 and IMU are co-located
  extrinsic_T: [0.0, 0.0, 0.0]       # Identity transform
  extrinsic_R: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]       # Identity rotation

  # LiDAR 2 Extrinsics (to IMU body frame / LiDAR 1)
  # Relative to LiDAR 1: 22cm lateral, 180° yaw
  extrinsic_T2: [0.0, -0.22, 0.0]    # 22cm to the right of LiDAR 1
  extrinsic_R2: [-1.0,  0.0,  0.0,
                  0.0, -1.0,  0.0,
                  0.0,  0.0,  1.0]   # 180° yaw relative rotation
```

**Configuration Notes:**
- **Reference changed from `base_link` to IMU body frame** to match FAST_LIO_MULTI
- LiDAR 1 serves as the IMU reference (identity transform)
- LiDAR 2 position is now relative to LiDAR 1, not base_link
- This matches the FAST-LIO standard where IMU is the primary reference

---

## Coordinate Frame Analysis

### Livox MID-360 Coordinate Frame Convention

```
Livox Sensor Frame (looking from behind the sensor):
     Z (up)
     |
     |__ Y (right)
    /
   X (forward, beam direction)
```

### Base Link Frame (ROS Standard - REP 103)

```
Base Link Frame (vehicle/robot):
     Z (up)
     |
     |__ Y (left)
    /
   X (forward)
```

### Transformation Chain

```
World Frame (map/odom)
    ↓
IMU Body Frame (PRIMARY REFERENCE - LiDAR 1's built-in IMU)
    ↓
├── LiDAR 1 Frame (co-located with IMU) → Point Cloud 1
    ↓
└── LiDAR 2 Frame (22cm lateral, 180° yaw) → Point Cloud 2
```

**FAST-LIO Standard Approach:**
- **IMU body frame is the primary reference** for all extrinsics
- All sensor poses are expressed relative to this IMU frame
- For MID-360 with built-in IMU: LiDAR optical center = IMU center

**Our Configuration:**
- Using **LiDAR 1's (192.168.1.10) built-in IMU** as the reference
- LiDAR 1 → IMU: Identity transform (co-located)
- LiDAR 2 → IMU: Computed relative to LiDAR 1's position/orientation

---

## Converting Livox to FAST-LIO Extrinsics (IMU Body Frame)

### Step 1: Understand the Livox Parameters

**NOTE:** Livox driver extrinsics are relative to `base_link`, but FAST-LIO needs them relative to the **IMU body frame**.

**LiDAR 1 (192.168.1.10) - Contains our reference IMU:**
- Position relative to base_link: `[x=0, y=110mm, z=0]` → `[0.0, 0.11, 0.0]` meters
- Orientation: Roll=90°, Pitch=0°, Yaw=180°
- **This is our IMU reference frame**

**LiDAR 2 (192.168.1.18):**
- Position relative to base_link: `[x=0, y=-110mm, z=0]` → `[0.0, -0.11, 0.0]` meters
- Orientation: Roll=90°, Pitch=0°, Yaw=0°

### Step 2: Convert Euler Angles to Rotation Matrices

#### Formula: ZYX Euler Convention (Yaw-Pitch-Roll)

```
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

Where:
```
Rx(α) = [1,      0,       0    ]    Ry(β) = [cos(β),  0, sin(β)]    Rz(γ) = [cos(γ), -sin(γ), 0]
        [0, cos(α), -sin(α)]            [  0,     1,   0   ]            [sin(γ),  cos(γ), 0]
        [0, sin(α),  cos(α)]            [-sin(β), 0, cos(β)]            [  0,       0,    1]
```

#### LiDAR 1: Roll=90°, Pitch=0°, Yaw=180°

```python
import numpy as np

roll1 = np.radians(90)
pitch1 = np.radians(0)
yaw1 = np.radians(180)

Rx1 = np.array([
    [1,              0,               0],
    [0, np.cos(roll1), -np.sin(roll1)],
    [0, np.sin(roll1),  np.cos(roll1)]
])

Ry1 = np.array([
    [ np.cos(pitch1), 0, np.sin(pitch1)],
    [       0,        1,        0       ],
    [-np.sin(pitch1), 0, np.cos(pitch1)]
])

Rz1 = np.array([
    [np.cos(yaw1), -np.sin(yaw1), 0],
    [np.sin(yaw1),  np.cos(yaw1), 0],
    [     0,             0,       1]
])

R1 = Rz1 @ Ry1 @ Rx1

# Result:
# R1 = [-1.0,  0.0,  0.0]
#      [ 0.0,  0.0,  1.0]
#      [ 0.0,  1.0,  0.0]
```

**LiDAR 1 Rotation Matrix:**
```yaml
extrinsic_R: [-1.0,  0.0,  0.0,
               0.0,  0.0,  1.0,
               0.0,  1.0,  0.0]
```

#### LiDAR 2: Roll=90°, Pitch=0°, Yaw=0°

```python
roll2 = np.radians(90)
pitch2 = np.radians(0)
yaw2 = np.radians(0)

# ... (same Rx, Ry, Rz computation)

R2 = Rz2 @ Ry2 @ Rx2

# Result:
# R2 = [1.0,  0.0,  0.0]
#      [0.0,  0.0, -1.0]
#      [0.0,  1.0,  0.0]
```

**LiDAR 2 Rotation Matrix:**
```yaml
extrinsic_R2: [1.0,  0.0,  0.0,
               0.0,  0.0, -1.0,
               0.0,  1.0,  0.0]
```

### Step 3: Transform to IMU Body Frame Reference

**Since we're using LiDAR 1's IMU as reference:**

**LiDAR 1 → IMU:**
- **Translation**: `[0.0, 0.0, 0.0]` - IDENTITY (co-located)
- **Rotation**: Identity matrix (same frame)

```yaml
extrinsic_T: [0.0, 0.0, 0.0]
extrinsic_R: [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
```

**LiDAR 2 → IMU (LiDAR 1):**

Compute relative transform from base_link-referenced poses:

**Translation:**
```
T_L2_to_IMU = T_L2_to_base - T_L1_to_base
            = [0.0, -0.11, 0.0] - [0.0, 0.11, 0.0]
            = [0.0, -0.22, 0.0]
```

**Rotation:**
```
R_L2_to_IMU = R_L2_to_base * R_L1_to_base^T

R_L1 (base): [-1, 0, 0; 0, 0, 1; 0, 1, 0]  (Roll=90°, Yaw=180°)
R_L2 (base): [ 1, 0, 0; 0, 0,-1; 0, 1, 0]  (Roll=90°, Yaw=0°)

R_L2_to_IMU = [-1.0,  0.0,  0.0]
              [ 0.0, -1.0,  0.0]
              [ 0.0,  0.0,  1.0]
```

This represents a **180° yaw rotation** (sensors facing opposite directions).

---

## Updated FAST-LIO Configuration (IMU Body Frame)

### Configuration Philosophy

**Changed Reference Frame:** From `base_link` to **IMU body frame** (LiDAR 1's built-in IMU)

This matches FAST_LIO_MULTI's standard approach where:
- IMU body frame is the primary reference
- Primary LiDAR (with IMU) has identity transform
- Secondary LiDAR(s) positioned relative to IMU/primary LiDAR

### Updated Configuration

```yaml
mapping:
  # ======================================================================
  # REFERENCE FRAME: IMU BODY FRAME (LiDAR 1 Built-in IMU)
  # ======================================================================

  # ======================================================================
  # LIDAR 1 EXTRINSICS (Primary LiDAR → IMU Body Frame)
  # ======================================================================
  # IP: 192.168.1.10 (Contains the reference IMU)
  # IDENTITY: LiDAR 1 optical center = IMU center (co-located)
  # ======================================================================
  extrinsic_T: [0.0, 0.0, 0.0]       # Identity translation

  extrinsic_R: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]       # Identity rotation

  # ======================================================================
  # LIDAR 2 EXTRINSICS (Secondary LiDAR → IMU Body Frame)
  # ======================================================================
  # IP: 192.168.1.18
  # Position: 22cm to the RIGHT of LiDAR 1/IMU
  # Orientation: 180° yaw (facing opposite direction)
  # ======================================================================
  extrinsic_T2: [0.0, -0.22, 0.0]    # 22cm lateral offset

  extrinsic_R2: [-1.0,  0.0,  0.0,
                  0.0, -1.0,  0.0,
                  0.0,  0.0,  1.0]   # 180° yaw rotation

  # ======================================================================
  # RELATIVE TRANSFORM: LiDAR 2 w.r.t. LiDAR 1
  # ======================================================================
  # Same as extrinsic_T2 and extrinsic_R2 since LiDAR 1 = IMU reference
  extrinsic_T_L2_wrt_L1: [0.0, -0.22, 0.0]
  extrinsic_R_L2_wrt_L1: [-1.0,  0.0,  0.0,
                           0.0, -1.0,  0.0,
                           0.0,  0.0,  1.0]
```

### Key Changes from Base Link Reference:

| Parameter | Old (base_link) | New (IMU body frame) |
|-----------|----------------|---------------------|
| **Reference** | `base_link` | IMU body (LiDAR 1) |
| **extrinsic_T** | `[0.0, 0.11, 0.0]` | `[0.0, 0.0, 0.0]` (identity) |
| **extrinsic_R** | Roll+Yaw transform | Identity matrix |
| **extrinsic_T2** | `[0.0, -0.11, 0.0]` | `[0.0, -0.22, 0.0]` (relative to L1) |
| **extrinsic_R2** | Roll+Yaw transform | 180° yaw (relative to L1) |

---

## Verification and Validation

### Method 1: Visual Inspection in RViz2

1. Launch Livox driver with dual LiDARs:
   ```bash
   ros2 launch livox_ros_driver2 livox_multi_lidar_launch.py
   ```

2. Launch FAST-LIO with corrected config:
   ```bash
   ros2 launch fast_lio_ros2 mapping.launch.py config_file:=dual_mid360_mine.yaml
   ```

3. Open RViz2 and subscribe to:
   - `/cloud_registered` (global map)
   - `/cloud_registered_body` (body frame scan)
   - `/livox/lidar_192_168_1_10` (raw LiDAR 1)
   - `/livox/lidar_192_168_1_18` (raw LiDAR 2)

4. Check for:
   - ✅ No ghosting or double walls
   - ✅ Smooth alignment at sensor overlap regions
   - ✅ Consistent geometry (corners, edges match)
   - ❌ Misalignment indicates wrong extrinsics

### Method 2: Static Scene Test

1. Place robot in a corridor or room with clear geometric features
2. Keep robot completely stationary
3. Collect data for 10-20 seconds
4. Save point cloud and inspect:
   - Measure wall thickness (should be ~10-20cm, not 50cm+)
   - Check corners (should be sharp 90°, not rounded/doubled)
   - Verify parallel walls are actually parallel

### Method 3: Rotation Test

1. Slowly rotate robot 360° in place
2. Observe map consistency:
   - ✅ Single consistent circular map
   - ❌ Multiple overlapping circles = wrong orientation
   - ❌ Drift/distortion = wrong relative transform

### Method 4: Check TF Tree

```bash
ros2 run tf2_tools view_frames
```

Verify the transform chain:
```
base_link
├── lidar1_frame (192.168.1.10)
└── lidar2_frame (192.168.1.18)
```

Manually check transforms match configuration:
```bash
ros2 run tf2_ros tf2_echo base_link lidar1_frame
ros2 run tf2_ros tf2_echo base_link lidar2_frame
```

### Method 5: Python Verification Script

Use the provided conversion script to double-check math:

```python
import numpy as np

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert Euler angles (degrees) to rotation matrix."""
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx

# LiDAR 1: Roll=90°, Pitch=0°, Yaw=180°
R1 = euler_to_rotation_matrix(90, 0, 180)
print("LiDAR 1 Rotation Matrix:")
print(R1)

# LiDAR 2: Roll=90°, Pitch=0°, Yaw=0°
R2 = euler_to_rotation_matrix(90, 0, 0)
print("\nLiDAR 2 Rotation Matrix:")
print(R2)

# Relative rotation
R_L2_wrt_L1 = R2 @ R1.T
print("\nLiDAR 2 w.r.t. LiDAR 1 Rotation:")
print(R_L2_wrt_L1)
```

---

## Key Differences: FAST_LIO_MULTI vs. Our Setup

| Aspect | FAST_LIO_MULTI | Our Setup |
|--------|----------------|-----------|
| Reference Frame | IMU body frame | **IMU body frame** (LiDAR 1) ✅ |
| Sensor Orientation | Tilted ±143° (wide FOV) | One forward, one backward (180° coverage) |
| Input Format | Rotation matrices | Rotation matrices (converted from Livox Euler) |
| Baseline | ~13cm | 22cm (lateral separation) |
| Primary LiDAR | Identity transform | **Identity transform** ✅ |
| Secondary LiDAR | Relative to primary | **Relative to primary** ✅ |
| Use Case | Drone/aerial mapping | Ground robot (mine navigation) |

**✅ = Now matches FAST_LIO_MULTI convention**

---

## Common Pitfalls and Troubleshooting

### 1. Euler Angle Convention Mismatch
- **Problem**: Different software uses different Euler angle conventions (XYZ, ZYX, etc.)
- **Solution**: Always verify convention and test with known orientations
- **Livox uses**: Intrinsic ZYX (Yaw-Pitch-Roll)

### 2. Rotation Matrix Row/Column Major
- **Problem**: YAML can be ambiguous about matrix layout
- **Solution**: FAST-LIO uses **row-major** format:
  ```yaml
  R: [r11, r12, r13,   # Row 1
      r21, r22, r23,   # Row 2
      r31, r32, r33]   # Row 3
  ```

### 3. Unit Mismatches
- **Livox driver**: Millimeters (mm) and Degrees (°)
- **FAST-LIO**: Meters (m) and Rotation matrices (unitless)
- **Always convert units!**

### 4. Sign Conventions
- **ROS REP-103**: Right-hand coordinate system
- X-forward, Y-left, Z-up (for base_link)
- Double-check signs when measuring physical offsets

### 5. Static vs. Dynamic Calibration
- **Static**: Measure physically with ruler/CAD (what we're using)
- **Dynamic**: Use calibration algorithms (LI-Init, GICP)
- Static is sufficient for well-mounted sensors
- Dynamic recommended for flex/vibration-prone mounts

---

## References

1. [FAST_LIO_MULTI Repository](https://github.com/engcang/FAST_LIO_MULTI)
2. [Original FAST-LIO Documentation](https://github.com/hku-mars/FAST_LIO)
3. [ROS REP-103: Standard Units of Measure](https://www.ros.org/reps/rep-0103.html)
4. [ROS REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
5. [Livox MID-360 User Manual](https://www.livoxtech.com/mid-360)

---

## Appendix: Quick Reference

### Rotation Matrices for Common Orientations

**Identity (no rotation):**
```yaml
R: [1, 0, 0,
    0, 1, 0,
    0, 0, 1]
```

**90° Yaw (left turn):**
```yaml
R: [0, -1, 0,
    1,  0, 0,
    0,  0, 1]
```

**180° Yaw (backward):**
```yaml
R: [-1, 0, 0,
     0, -1, 0,
     0, 0, 1]
```

**-90° Yaw (right turn):**
```yaml
R: [0, 1, 0,
   -1, 0, 0,
    0, 0, 1]
```

**90° Roll (Livox standard mount):**
```yaml
R: [1,  0,  0,
    0,  0, -1,
    0,  1,  0]
```

**90° Roll + 180° Yaw (Livox backward):**
```yaml
R: [-1, 0,  0,
     0, 0,  1,
     0, 1,  0]
```

---

**Document Version**: 1.0
**Last Updated**: 2025-11-24
**Author**: Claude (Analysis of FAST_LIO_MULTI + System Configuration)
