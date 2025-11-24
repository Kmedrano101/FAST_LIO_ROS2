# Extrinsic Calibration Update Summary

**Date**: 2025-11-24
**System**: Dual Livox MID-360 LiDAR Setup on Jetson ORIN

---

## Overview

Updated FAST-LIO extrinsic calibration parameters to match the Livox driver configuration. The previous configuration had the LiDAR orientations reversed, which would cause misalignment in the point cloud fusion.

---

## Changes Made

### Configuration File Updated
- **File**: `/home/jetson/ros2_ws/src/fast_lio_ros2/config/dual_mid360_mine.yaml`
- **Sections Modified**: `mapping.extrinsic_T`, `mapping.extrinsic_R`, `mapping.extrinsic_T2`, `mapping.extrinsic_R2`, `mapping.extrinsic_R_L2_wrt_L1`

### Source of Truth
Extrinsic values extracted from:
- **File**: `/home/jetson/ros2_ws/src/livox_ros_driver2/config/multiple_netconfigs.json`
- **Reference Frame**: `base_link`

---

## Previous Configuration (INCORRECT)

```yaml
# LiDAR 1 (192.168.1.10) - LEFT SIDE
extrinsic_T: [0.0, 0.11, 0.0]
extrinsic_R: [ 1.00000,  0.00000,  0.00000,   # Yaw=0° (FORWARD) ❌ WRONG
               0.00000,  0.00000, -1.00000,
               0.00000,  1.00000,  0.00000]

# LiDAR 2 (192.168.1.18) - RIGHT SIDE
extrinsic_T2: [0.0, -0.11, 0.0]
extrinsic_R2: [-1.00000,  0.00000,  0.00000,   # Yaw=180° (BACKWARD) ❌ WRONG
                0.00000,  0.00000,  1.00000,
                0.00000,  1.00000,  0.00000]
```

### Issue
The orientations were **reversed**:
- LiDAR 1 was configured as forward-facing (Yaw=0°), but Livox driver has it at Yaw=180° (backward)
- LiDAR 2 was configured as backward-facing (Yaw=180°), but Livox driver has it at Yaw=0° (forward)

This mismatch would cause:
- ❌ Point cloud misalignment
- ❌ Doubled/ghosted walls in mapping
- ❌ Poor odometry performance
- ❌ Incorrect sensor fusion

---

## New Configuration (CORRECT)

```yaml
# LiDAR 1 (192.168.1.10) - LEFT SIDE
# Position: 11cm to the left (+Y)
# Orientation: Facing BACKWARD (Yaw=180°)
extrinsic_T: [0.0, 0.11, 0.0]
extrinsic_R: [-1.00000, -0.00000,  0.00000,   # Roll=90°, Pitch=0°, Yaw=180° ✓
               0.00000, -0.00000,  1.00000,
               0.00000,  1.00000,  0.00000]

# LiDAR 2 (192.168.1.18) - RIGHT SIDE
# Position: 11cm to the right (-Y)
# Orientation: Facing FORWARD (Yaw=0°)
extrinsic_T2: [0.0, -0.11, 0.0]
extrinsic_R2: [ 1.00000,  0.00000,  0.00000,   # Roll=90°, Pitch=0°, Yaw=0° ✓
                0.00000,  0.00000, -1.00000,
                0.00000,  1.00000,  0.00000]

# Relative Transform (L2 w.r.t. L1)
# Translation: 22cm lateral separation (L2 is 22cm to the right of L1)
# Rotation: 180° yaw difference (sensors facing opposite directions)
extrinsic_T_L2_wrt_L1: [0.0, -0.22, 0.0]
extrinsic_R_L2_wrt_L1: [-1.00000,  0.00000,  0.00000,   # 180° yaw rotation ✓
                        -0.00000, -1.00000,  0.00000,
                         0.00000,  0.00000,  1.00000]
```

---

## Physical Setup

```
                      BASE_LINK (top view)
                           ^
                           | X (forward)
                           |
                           |
       LiDAR 1 (Left)      |      LiDAR 2 (Right)
       IP: .10             |      IP: .18
       Facing: ← BACK      |      Facing: FORWARD →
       Position: +110mm Y  |      Position: -110mm Y
                           |
    Y+ <-------------------+
   (left)

    Baseline: 220mm lateral separation
```

### Sensor Details

| Parameter | LiDAR 1 (192.168.1.10) | LiDAR 2 (192.168.1.18) |
|-----------|------------------------|------------------------|
| **Side** | Left (+Y) | Right (-Y) |
| **Position** | [0, 0.11, 0] m | [0, -0.11, 0] m |
| **Roll** | 90° | 90° |
| **Pitch** | 0° | 0° |
| **Yaw** | 180° (backward) | 0° (forward) |
| **Topic** | `/livox/lidar_192_168_1_10` | `/livox/lidar_192_168_1_18` |
| **IMU** | `/livox/imu_192_168_1_10` | N/A (using L1's IMU) |

---

## Conversion Details

### From Livox Driver Format
```json
// LiDAR 1 (192.168.1.10)
"extrinsic_parameter": {
  "roll": 90.0,    // degrees
  "pitch": 0.0,
  "yaw": 180.0,
  "x": 0,          // millimeters
  "y": 110,
  "z": 0
}
```

### To FAST-LIO Format
```yaml
# Convert: mm → meters, Euler angles → Rotation matrix
extrinsic_T: [0.0, 0.11, 0.0]  # [0mm, 110mm, 0mm] → meters
extrinsic_R: [-1.00000, -0.00000,  0.00000,   # euler_to_rotation_matrix(90, 0, 180)
               0.00000, -0.00000,  1.00000,
               0.00000,  1.00000,  0.00000]
```

**Conversion Formula**: ZYX Euler Convention
```
R = Rz(yaw) × Ry(pitch) × Rx(roll)
```

---

## Verification Steps

### 1. Mathematical Validation ✓
- **Rotation Matrix Determinant**: det(R) = 1.0 for both sensors
- **Orthogonality**: R × R^T = I (identity matrix)
- **Baseline Distance**: ||T2 - T1|| = 0.22m (220mm)

### 2. Visual Verification (RViz2)
To verify the corrected extrinsics:

```bash
# Terminal 1: Launch Livox driver
ros2 launch livox_ros_driver2 livox_multi_lidar_launch.py

# Terminal 2: Launch FAST-LIO with updated config
ros2 launch fast_lio_ros2 mapping.launch.py config_file:=dual_mid360_mine.yaml

# Terminal 3: Open RViz2
rviz2
```

**Check for**:
- ✅ Single, consistent walls (no ghosting or doubling)
- ✅ Smooth alignment at sensor overlap regions
- ✅ Sharp corners (not rounded or doubled)
- ✅ Consistent geometry throughout the map
- ❌ If you see double walls or misalignment, extrinsics may still be wrong

### 3. Static Scene Test
1. Place robot in a corridor or room with clear features
2. Keep robot completely stationary for 20 seconds
3. Save point cloud: `ros2 service call /save_map std_srvs/srv/Trigger`
4. Inspect map:
   - Wall thickness should be 10-20cm (not 50cm+)
   - Corners should be sharp 90° angles
   - Parallel walls should remain parallel

### 4. Rotation Test
1. Slowly rotate robot 360° in place
2. Map should form a single consistent circle
3. Look for:
   - ✅ No drift or distortion
   - ✅ Overlap regions align perfectly
   - ❌ Multiple overlapping circles = wrong orientation

---

## Tools and Scripts

### Automated Conversion Script
Created: `/home/jetson/ros2_ws/src/fast_lio_ros2/scripts/convert_livox_extrinsics.py`

```bash
# Run to convert Livox config to FAST-LIO format
python3 /home/jetson/ros2_ws/src/fast_lio_ros2/scripts/convert_livox_extrinsics.py
```

This script:
- Reads `/home/jetson/ros2_ws/src/livox_ros_driver2/config/multiple_netconfigs.json`
- Converts Euler angles → Rotation matrices
- Converts mm → meters
- Computes relative transforms
- Validates rotation matrices
- Outputs ready-to-use YAML configuration

### Interactive Calculator
Existing tool: `/home/jetson/ros2_ws/src/fast_lio_ros2/utils/extrinsic_calculator.py`

```bash
# Run for interactive Euler angle conversions
python3 /home/jetson/ros2_ws/src/fast_lio_ros2/utils/extrinsic_calculator.py
```

---

## Documentation

### Comprehensive Guide
**File**: `/home/jetson/ros2_ws/src/fast_lio_ros2/docs/EXTRINSIC_CALIBRATION_DUAL_LIDAR.md`

Includes:
- Detailed explanation of FAST_LIO_MULTI extrinsic approach
- Coordinate frame conventions
- Euler angle to rotation matrix conversion
- Comparison with FAST_LIO_MULTI repository
- Validation methods
- Troubleshooting common issues

---

## FAST_LIO_MULTI Comparison

### FAST_LIO_MULTI Setup (from repository)
- **Reference Frame**: IMU body frame
- **Sensors**: Tilted ±143° (±16° pitch) for wide vertical FOV
- **Use Case**: Aerial/drone mapping
- **Baseline**: ~13cm

### Our Setup
- **Reference Frame**: `base_link`
- **Sensors**: One forward (0°), one backward (180°) for 360° horizontal coverage
- **Use Case**: Ground robot, underground mine navigation
- **Baseline**: 22cm lateral

**Key Difference**: Our setup prioritizes horizontal 360° coverage for tunnel navigation, while FAST_LIO_MULTI prioritizes vertical FOV for aerial scanning.

---

## Next Steps

### 1. Test the Updated Configuration
```bash
cd ~/ros2_ws
colcon build --packages-select fast_lio_ros2
source install/setup.bash
ros2 launch fast_lio_ros2 mapping.launch.py config_file:=dual_mid360_mine.yaml
```

### 2. Validate in Real Environment
- Run in mine tunnel or similar environment
- Verify no ghosting or misalignment
- Check odometry accuracy

### 3. Tune Parameters (if needed)
If you still see issues:
- Adjust `adaptive_fov_threshold` for sparse environments
- Tune `acc_cov` and `gyr_cov` for vibration handling
- Modify `blind` parameter if dust is affecting close-range points

### 4. Save Good Maps
Once validated:
```bash
# Maps are auto-saved at: /home/jetson/ros2_ws/src/fast_lio_ros2/PCD/
# Check pcd_file_name in config: "mine_dual_mid360.pcd"
```

---

## Troubleshooting

### Issue: Still seeing double walls
**Cause**: Extrinsic translation might be wrong
**Solution**: Verify physical measurements with tape measure/calipers

### Issue: Map drifting or distorting
**Cause**: IMU noise too high or sensor synchronization issues
**Solution**:
- Increase `acc_cov` and `gyr_cov` in config
- Check `time_sync_en` setting
- Verify IMU data quality

### Issue: Poor performance in sparse areas
**Cause**: Not enough features per scan
**Solution**:
- Switch to Bundle mode: `update_mode: 0`
- Increase `adaptive_fov_threshold`
- Reduce `point_filter_num` (keep more points)

---

## References

1. **FAST_LIO_MULTI Repository**: https://github.com/engcang/FAST_LIO_MULTI
2. **Original FAST-LIO**: https://github.com/hku-mars/FAST_LIO
3. **Livox MID-360 Docs**: https://www.livoxtech.com/mid-360
4. **ROS REP-103** (Standard Units): https://www.ros.org/reps/rep-0103.html
5. **ROS REP-105** (Coordinate Frames): https://www.ros.org/reps/rep-0105.html

---

## Change Log

| Date | Change | Reason |
|------|--------|--------|
| 2025-11-24 | Initial calibration | Setup dual MID-360 config |
| 2025-11-24 | **Corrected extrinsic orientations** | **Fixed reversed LiDAR orientations to match Livox driver** |

---

**Status**: ✅ **Configuration Updated and Ready for Testing**

**Contact**: Check documentation or run conversion scripts for more details.
