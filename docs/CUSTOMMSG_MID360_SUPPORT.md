# CustomMsg Support for Dual Livox MID-360

## Overview

This document describes the modifications made to `fast_lio_ros2` to support Livox CustomMsg format for MID-360 LiDAR sensors, enabling precise per-point timestamps for improved motion compensation.

**Date:** January 2025
**Branch:** `jetson-dev-pcl`
**Target Hardware:** Dual Livox MID-360 + NVIDIA Jetson

---

## Motivation

### PointCloud2 vs CustomMsg

| Feature | PointCloud2 | CustomMsg |
|---------|-------------|-----------|
| Per-point timestamp | No (calculated from yaw angle) | Yes (`offset_time` in ns) |
| Point quality tag | Limited | Full (`tag` field) |
| Scan line info | Derived | Direct (`line` field) |
| Motion compensation | Approximate | Precise |

The `offset_time` field in CustomMsg provides exact timestamps for each point, which is critical for accurate motion undistortion in FAST-LIO's IMU-LiDAR fusion.

---

## Files Modified

### 1. `src/preprocess.h`

**Change:** Added declaration for `mid360_custom_handler`

```cpp
#ifdef USE_LIVOX_DRIVER2
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, const int &lidar_num);
  void mid360_custom_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, const int &lidar_num);  // NEW
#endif
```

**Location:** Line 184-186

---

### 2. `src/preprocess.cpp`

#### 2.1 Modified `process()` for CustomMsg

**Change:** Added switch case to route MID360 to the new handler

```cpp
void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg,
                         PointCloudXYZI::Ptr& pcl_out, const int &lidar_num)
{
  switch (lidar_type[lidar_num])
  {
    case AVIA:
      avia_handler(msg, lidar_num);
      break;
    case MID360:
      mid360_custom_handler(msg, lidar_num);  // NEW
      break;
    default:
      avia_handler(msg, lidar_num);
      break;
  }
  *pcl_out = pl_surf;
}
```

**Location:** Lines 189-205

#### 2.2 New `mid360_custom_handler()` Implementation

**Purpose:** Process CustomMsg for MID-360 with proper handling of:
- Per-point `offset_time` (nanoseconds -> milliseconds)
- 4 scan lines (`line` field: 0-3)
- Blind zone filtering
- Duplicate point removal

**Key differences from AVIA handler:**
1. **Tag filtering removed** - MID360 uses different tag scheme
2. **Fixed operator precedence bug** in blind zone filter

**Original bug (inherited from avia_handler):**
```cpp
// WRONG: && has higher precedence than ||
if ((abs(x) > 1e-7) || (abs(y) > 1e-7) || (abs(z) > 1e-7) && (range > blind))
// Evaluates as: A || B || (C && D)  -- blind filter only applies when Z changes!
```

**Fixed implementation:**
```cpp
// CORRECT: explicit precedence
bool is_not_duplicate = (abs(x) > 1e-7) || (abs(y) > 1e-7) || (abs(z) > 1e-7);
if (is_not_duplicate && (range_sq > blind_sq))
// Evaluates as: (A || B || C) && D  -- blind filter applies to ALL points
```

**Full handler code:**
```cpp
void Preprocess::mid360_custom_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg,
                                        const int &lidar_num)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  int plsize = msg->point_num;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS[lidar_num]; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  if (feature_enabled)
  {
    // Feature extraction mode (same as AVIA)
    // ... [see source code]
  }
  else
  {
    // Non-feature mode: process all valid points with offset_time
    for (uint i = 1; i < plsize; i++)
    {
      // Accept points with valid line number (0-3 for MID360)
      if (msg->points[i].line < N_SCANS[lidar_num])
      {
        valid_num++;
        if (valid_num % point_filter_num[lidar_num] == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000);  // ns -> ms

          // Check if point is outside blind zone
          double range_sq = pl_full[i].x * pl_full[i].x +
                           pl_full[i].y * pl_full[i].y +
                           pl_full[i].z * pl_full[i].z;
          double blind_sq = blind[lidar_num] * blind[lidar_num];

          // Filter: point must be outside blind zone AND not a duplicate
          bool is_not_duplicate = (abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
                                  (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
                                  (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7);

          if (is_not_duplicate && (range_sq > blind_sq))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}
```

**Location:** Lines 207-297

---

### 3. `src/laserMapping.cpp`

**Change:** Modified subscription logic to use CustomMsg for both AVIA and MID360

**Before:**
```cpp
// Only AVIA used CustomMsg
if (p_pre->lidar_type[LIDAR1] == AVIA)
{
    sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(...);
}
else
{
    // MID360 used PointCloud2
    sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(...);
}
```

**After:**
```cpp
// Helper lambda to check if lidar type supports CustomMsg
auto is_livox_custom_msg_type = [](int type) {
    return (type == AVIA || type == MID360);
};

if (is_livox_custom_msg_type(p_pre->lidar_type[LIDAR1]))
{
    RCLCPP_INFO(this->get_logger(), "Using Livox CustomMsg format (AVIA/MID360)");
    sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(...);
}
else
{
    // Velodyne, Ouster, etc. use PointCloud2
    sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(...);
}
```

**Location:** Lines 1626-1700

---

## CustomMsg Structure

The `livox_ros_driver2::msg::CustomMsg` contains:

```
header:
  stamp: {sec, nanosec}
  frame_id: "livox_frame"
timebase: uint64 (nanoseconds since epoch)
point_num: uint32 (number of points in frame)
lidar_id: uint8 (from IP: 192 for 192.168.1.x)
points[]:
  offset_time: uint32 (nanoseconds from timebase)
  x, y, z: float32 (meters)
  reflectivity: uint8 (0-255)
  tag: uint8 (point quality/type)
  line: uint8 (scan line 0-3 for MID360)
```

### Tag Field Interpretation (MID360)

| Tag Value | Binary | Meaning |
|-----------|--------|---------|
| 0 | 00000000 | Valid point, normal return |
| 128 | 10000000 | Valid point, bit 7 set |
| 192 | 11000000 | Blind zone / invalid (coords near origin) |

**Note:** Unlike AVIA, MID360 points with tag=192 typically have coordinates (0, 0.11, 0) representing the sensor offset position. These are filtered by the blind zone check, not by tag.

---

## Configuration

### YAML Parameters (`config/dual_mid360.yaml`)

```yaml
preprocess:
  lidar_type: 4                    # MID360
  lidar_type2: 4                   # MID360 (second sensor)
  scan_line: 4                     # MID360 has 4 scan lines
  scan_line2: 4
  blind: 1.0                       # Blind zone in meters
  blind2: 0.3
  timestamp_unit: 3                # 3 = nanoseconds (for PointCloud2 fallback)
```

### Required livox_ros_driver2 Configuration

```yaml
# config/livox_params.yaml
livox_lidar_publisher:
  ros__parameters:
    xfer_format: 1      # CRITICAL: 1 = CustomMsg (not 0 = PointCloud2)
    multi_topic: 1      # Separate topics per LiDAR
    publish_freq: 10.0  # Hz (must be float, not int)
```

---

## Data Flow

```
Livox MID-360 Hardware
        |
        v
livox_ros_driver2 (xfer_format: 1)
        |
        v
/livox/lidar_192_168_1_10  [CustomMsg]
/livox/lidar_192_168_1_18  [CustomMsg]
        |
        v
fast_lio_ros2::livox_pcl_cbk() / livox_pcl_cbk2()
        |
        v
Preprocess::process(CustomMsg)
        |
        v
mid360_custom_handler()
  - Extract offset_time -> curvature (ms)
  - Filter blind zone
  - Remove duplicates
        |
        v
PointCloudXYZI (pl_surf)
        |
        v
IMU_Processing::UndistortPcl()
  - Uses curvature (offset_time) for precise motion compensation
        |
        v
FAST-LIO EKF Update
```

---

## Verification

### Check Topic Type
```bash
ros2 topic info /livox/lidar_192_168_1_10 -v
# Expected: Type: livox_ros_driver2/msg/CustomMsg
```

### Verify CustomMsg Data
```bash
ros2 topic echo /livox/lidar_192_168_1_10 --once | head -30
# Should show: offset_time, tag, line fields
```

### Check Valid Points (outside blind zone)
```bash
ros2 topic echo /livox/lidar_192_168_1_10 --once | grep -A7 "x: [1-9]" | head -20
# Should show points with x > 1m, tag: 0
```

---

## Troubleshooting

### Issue: Still publishing PointCloud2
**Cause:** `xfer_format` not set correctly or namespace mismatch

**Solution:**
1. Verify YAML namespace matches node name: `livox_lidar_publisher:`
2. Check launch file override: `'xfer_format': 1`
3. Rebuild: `colcon build --packages-select livox_ros_driver2`

### Issue: Parameter type error (publish_freq)
**Cause:** YAML has integer `10` instead of float `10.0`

**Solution:** Use `publish_freq: 10.0` (with decimal point)

### Issue: Points not being processed
**Cause:** Operator precedence bug in blind zone filter

**Solution:** Use the corrected `mid360_custom_handler` with explicit precedence

---

## References

- [Livox SDK2 Documentation](https://github.com/Livox-SDK/Livox-SDK2)
- [FAST-LIO Paper](https://github.com/hku-mars/FAST_LIO)
- [livox_ros_driver2 CustomMsg Definition](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/msg/CustomMsg.msg)
