# Extrinsic Calibration Guide for Dual MID-360 Setup

## Overview

This document explains the **complete extrinsic calibration chain** for the dual Livox MID-360 setup with FAST-LIO ROS2, detailing how transformations flow from LiDAR native frames through the Livox driver, slam_tools IMU transformer, and finally to FAST-LIO.

**Critical Understanding**: FAST-LIO uses **LiDAR 1 IMU body frame** as its reference coordinate system, NOT base_link. All transformations must be expressed relative to the L1 IMU body.

---

## Reference Frame Hierarchy

```
┌─────────────────────────────────────────────────────────────────────┐
│             FAST-LIO REFERENCE FRAME: L1 IMU BODY                    │
└─────────────────────────────────────────────────────────────────────┘

                        ┌───────────────┐
                        │  L1 IMU Body  │  ← FAST-LIO Origin
                        │  (0, 0, 0)    │
                        └───────────────┘
                              ↑
                   ┌──────────┼──────────┐
                   │          │          │
             ┌──────────┐ ┌────────┐ ┌────────┐
             │base_link │ │ L1 PCL │ │ L2 PCL │
             └──────────┘ └────────┘ └────────┘
```

**Key Points**:
- **L1 IMU body**: Origin (0,0,0) for FAST-LIO
- **base_link**: Payload physical reference, where Livox driver outputs points
- **Point clouds**: Arrive in base_link, must be transformed to L1 IMU body
- **IMU data**: Rotationally aligned by slam_tools, defines L1 IMU body orientation

---

## System Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                   TRANSFORMATION CHAIN                         │
└───────────────────────────────────────────────────────────────┘

LiDAR 1 Native       Livox Driver         slam_tools IMU
┌──────────┐        ┌──────────┐          ┌──────────┐
│ L1 Frame │  ───→  │ Rotate + │  ─────→  │   IMU    │
│ (native) │        │ Translate│          │ Aligned  │
└──────────┘        └──────────┘          └──────────┘
     ↓                    ↓                      ↓
Roll=90°            base_link             L1 IMU Body
Pitch=0°            Y=+110mm            (Rotation Only)
Yaw=180°                                      ↓
                                        ┌──────────┐
LiDAR 2 Native       Livox Driver      │ FAST-LIO │
┌──────────┐        ┌──────────┐      │  (SLAM)  │
│ L2 Frame │  ───→  │ Rotate + │  ──→ │          │
│ (native) │        │ Translate│      └──────────┘
└──────────┘        └──────────┘            ↑
     ↓                    ↓                  │
Roll=90°            base_link         Reference:
Pitch=0°            Y=-110mm          L1 IMU Body
Yaw=0°
```

---

## Hardware Configuration

### Physical Mounting

- **LiDAR 1 (192.168.1.10)**: Mounted at **Y = +110mm** from base_link
- **LiDAR 2 (192.168.1.18)**: Mounted at **Y = -110mm** from base_link
- **Separation**: 220mm along Y-axis
- **Orientation**: Both rotated 90° roll; L1 has additional 180° yaw

### MID-360 Native IMU Offset

From Livox MID-360 datasheet:
```
P_imu_native = [0.011, 0.02329, -0.04412] meters
```
This is in the LiDAR's **native coordinate frame** before any rotation.

---

## Stage 1: Livox Driver Transformations

### Configuration Files

**LiDAR 1:** `/home/jetson/ros2_ws/src/livox_ros_driver2/config/mid360_L1.json`
```json
{
    "ip": "192.168.1.10",
    "extrinsic_parameter": {
        "roll": 90.0,    // Rx(90°): Rotate sensor upright
        "pitch": 0.0,
        "yaw": 180.0,    // Rz(180°): Point backward
        "x": 0,          // No X offset
        "y": 110,        // 110mm forward of base_link
        "z": 0           // Same height
    }
}
```

**LiDAR 2:** `/home/jetson/ros2_ws/src/livox_ros_driver2/config/mid360_L2.json`
```json
{
    "ip": "192.168.1.18",
    "extrinsic_parameter": {
        "roll": 90.0,    // Rx(90°): Rotate sensor upright
        "pitch": 0.0,
        "yaw": 0.0,      // No yaw rotation (forward facing)
        "x": 0,          // No X offset
        "y": -110,       // 110mm behind base_link
        "z": 0           // Same height
    }
}
```

### Transformation Matrices

**LiDAR 1 Rotation:**
```
R_L1 = Rz(180°) × Rx(90°) = [-1,  0,  0]
                             [ 0,  0,  1]
                             [ 0,  1,  0]
```

**LiDAR 2 Rotation:**
```
R_L2 = Rx(90°) = [1,  0,  0]
                 [0,  0, -1]
                 [0,  1,  0]
```

### Point Cloud Transformation

The Livox driver transforms each point:
```
P_base_link = R_Li × P_lidar_native + T_Li
```

Where:
- **LiDAR 1**: `T_L1 = [0, 0.110, 0]` meters
- **LiDAR 2**: `T_L2 = [0, -0.110, 0]` meters

**Result**: Point clouds on `/livox/lidar_192_168_1_10` and `/livox/lidar_192_168_1_18` are in **base_link frame**.

---

## Stage 2: slam_tools IMU Transformer

### Purpose

The `imu_transformer` node aligns IMU orientation with the rotated LiDAR 1 frame, creating the **L1 IMU body frame** that FAST-LIO uses as its reference.

**Critical**: Applies **ONLY rotation**, NO translation.

### Configuration

```bash
ros2 run slam_tools imu_transformer \
    --ros-args \
    -p roll_deg:=90.0 \
    -p pitch_deg:=0.0 \
    -p yaw_deg:=180.0 \
    -p input_topic:=/livox/imu_192_168_1_10 \
    -p output_topic:=/livox/imu_transformed
```

Applies: `R_imu_body = (Rz(180°) × Rx(90°))^T` to IMU measurements.

**Result**: IMU on `/livox/imu_transformed` is rotationally aligned, defining the **L1 IMU body coordinate frame** used by FAST-LIO.

---

## Stage 3: FAST-LIO Extrinsic Configuration

### Understanding FAST-LIO Reference Frame

FAST-LIO operates in **L1 IMU body frame**:

1. **L1 IMU body** is at the origin (0,0,0)
2. Point clouds in **base_link** must be transformed TO this frame
3. Extrinsic parameters define: **base_link → L1 IMU body**

```
┌──────────────┐      Extrinsics       ┌──────────────┐
│  base_link   │  ─────────────────→   │ L1 IMU body  │
│ (PCL frame)  │   T, R from config    │ (FAST-LIO)   │
└──────────────┘                        └──────────────┘
```

### Calculating L1 IMU Position in base_link

#### Step 1: Rotate Native IMU Offset

Apply Livox driver rotation to native IMU position:

```
P_imu_native = [0.011, 0.02329, -0.04412]  (meters)

R_L1 = [-1,  0,  0]
       [ 0,  0,  1]
       [ 0,  1,  0]

P_imu_rotated = R_L1 × P_imu_native
              = [-1,  0,  0]   [0.011  ]     [-0.011  ]
                [ 0,  0,  1] × [0.02329]  =  [-0.04412]
                [ 0,  1,  0]   [-0.04412]    [0.02329 ]
```

#### Step 2: Add L1 Translation in base_link

```
T_L1_in_base = [0, 0.110, 0]  (L1 position in base_link)

P_L1_IMU_in_base = P_imu_rotated + T_L1_in_base
                 = [-0.011, -0.04412, 0.02329] + [0, 0.110, 0]
                 = [-0.011, 0.06588, 0.02329]
```

This is where the L1 IMU physically sits in base_link coordinates.

### FAST-LIO Extrinsic Transformation

To transform points FROM base_link TO L1 IMU body:

```
extrinsic_T_1 = -P_L1_IMU_in_base
              = -[-0.011, 0.06588, 0.02329]
              = [0.011, -0.06588, -0.02329]
```

**Rotation**: Identity (slam_tools already aligned)
```
extrinsic_R_1 = [1, 0, 0]
                [0, 1, 0]
                [0, 0, 1]
```

### LiDAR 2 Extrinsics

Since L2 points are ALSO in base_link, the transformation to L1 IMU body is:

```
extrinsic_T_2 = [0.011, -0.06588, -0.02329]  (SAME as L1!)
extrinsic_R_2 = Identity
```

Both LiDARs' points are in the same base_link frame, so they need the same transformation to reach L1 IMU body.

### Relative Transformation L2 ↔ L1

In base_link coordinates:
```
extrinsic_T_L2_wrt_L1 = T_L2_in_base - T_L1_in_base
                       = [0, -0.110, 0] - [0, 0.110, 0]
                       = [0, -0.220, 0]  (L2 is 220mm behind L1)

extrinsic_R_L2_wrt_L1 = Identity  (both aligned in base_link)
```

---

## Final FAST-LIO Configuration

**File**: `/home/jetson/ros2_ws/src/fast_lio_ros2/config/dual_mid360.yaml`

```yaml
mapping:
  # ================================================================
  # EXTRINSICS: Transform from base_link → L1 IMU body
  # ================================================================

  # LiDAR 1 Extrinsics
  extrinsic_T_1: [0.011, -0.06588, -0.02329]  # [x, y, z] meters

  # Calculation:
  #   1. Native IMU offset: [0.011, 0.02329, -0.04412]
  #   2. After L1 rotation:  [-0.011, -0.04412, 0.02329]
  #   3. Add L1 translation: [-0.011, 0.06588, 0.02329] (in base_link)
  #   4. Invert for transform: [0.011, -0.06588, -0.02329]

  extrinsic_R_1: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]  # Identity (pre-aligned)

  # LiDAR 2 Extrinsics (SAME as L1, both in base_link)
  extrinsic_T_2: [0.011, -0.06588, -0.02329]  # [x, y, z] meters

  extrinsic_R_2: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]  # Identity

  # Relative Transformation (L2 relative to L1 in base_link)
  extrinsic_T_L2_wrt_L1: [0.0, -0.220, 0.0]  # [x, y, z] meters

  extrinsic_R_L2_wrt_L1: [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
```

---

## Verification

### Reference Frame Diagram

```
     L1 IMU Body (FAST-LIO Reference)
            ┌───┐
            │ ● │  (0, 0, 0)
            └───┘
              ↑
              │ [0.011, -0.06588, -0.02329]
              │
     ╔════════════════════╗
     ║    base_link       ║
     ║                    ║
     ║  L1: Y=+110mm      ║
     ║  ┌────┐            ║
     ║  │    │            ║
     ║  └────┘            ║
     ║          ┌────┐    ║
     ║          │    │    ║
     ║          └────┘    ║
     ║  L2: Y=-110mm      ║
     ╚════════════════════╝
```

### Expected Behavior

When running with these extrinsics:

1. **Point clouds** arrive pre-transformed to base_link
2. **IMU data** is rotationally aligned with L1 frame
3. **FAST-LIO** transforms everything to L1 IMU body frame
4. **Odometry** and **mapping** are smooth and consistent

### Visual Check in RViz

1. Place payload in front of known geometry (wall, corner)
2. Enable visualization:
   - `/cloud_registered` (merged map)
   - `/lidar1_colored` (green)
   - `/lidar2_colored` (red)
3. Verify perfect alignment on static features

---

## Configuration Files Summary

| File | Purpose | Key Values |
|------|---------|------------|
| **mid360_L1.json** | Livox driver L1 | Roll=90°, Yaw=180°, Y=+110mm |
| **mid360_L2.json** | Livox driver L2 | Roll=90°, Yaw=0°, Y=-110mm |
| **slam_tools** | IMU rotation | Roll=90°, Yaw=180° |
| **dual_mid360.yaml** | FAST-LIO extrinsics | T=[0.011, -0.06588, -0.02329] |

---

## Summary Table

| Parameter | LiDAR 1 | LiDAR 2 | Notes |
|-----------|---------|---------|-------|
| **Livox Driver** |
| Rotation (R,P,Y) | 90°, 0°, 180° | 90°, 0°, 0° | Both upright, L1 backward |
| Translation (mm) | [0, 110, 0] | [0, -110, 0] | Y-axis separation |
| Output Frame | base_link | base_link | Same frame |
| **slam_tools IMU** |
| Rotation Applied | 90°, 0°, 180° | N/A | Only L1 IMU used |
| Translation | None | N/A | Rotation only |
| Output Frame | L1 IMU body | N/A | FAST-LIO reference |
| **FAST-LIO Extrinsics** |
| extrinsic_T | [0.011, -0.06588, -0.02329] | [0.011, -0.06588, -0.02329] | base_link → L1 IMU |
| extrinsic_R | Identity | Identity | Pre-aligned |
| Reference Frame | L1 IMU body | L1 IMU body | Same origin |

---

## Troubleshooting

### Issue: Map not aligning properly

**Check**:
1. Verify Livox driver configs match this document
2. Confirm slam_tools is running with correct parameters
3. Review FAST-LIO extrinsics are `[0.011, -0.06588, -0.02329]`

### Issue: Odometry drift

**Possible causes**:
1. Incorrect extrinsic_T values
2. slam_tools not running → IMU not aligned
3. Time synchronization issues between sensors

**Debug**:
```bash
# Verify slam_tools is running
ros2 node list | grep imu_transformer

# Check transformed IMU
ros2 topic echo /livox/imu_transformed --no-arr

# Monitor FAST-LIO logs
ros2 launch fast_lio_ros2 dual_mapping_core.launch.py
```

### Issue: One LiDAR not contributing

**Check**:
1. Verify both point cloud topics publishing:
   ```bash
   ros2 topic hz /livox/lidar_192_168_1_10
   ros2 topic hz /livox/lidar_192_168_1_18
   ```
2. Check FAST-LIO is subscribing to both topics
3. Verify `multi_lidar: true` in config

---

## References

- **Livox MID-360 Datasheet**: IMU offset specifications
- **FAST-LIO Paper**: "FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package"
- **ROS REP-105**: Coordinate Frames for Mobile Platforms
- **livox_ros_driver2**: Livox SDK 2.0 ROS2 Driver Documentation

---

**Document Version**: 2.0
**Last Updated**: 2025-12-02
**Status**: Verified with working dual MID-360 setup
**Key Insight**: FAST-LIO uses L1 IMU body as reference frame, not base_link
