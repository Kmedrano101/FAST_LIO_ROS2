# FAST-LIO Gazebo Drift Analysis: MID360 vs Gazebo Simulation

## Problem Statement

FAST-LIO works perfectly with real MID360 hardware but experiences significant drift (10+ meters) in Gazebo simulation even after flying for short periods.

## Root Cause: Missing Point Timestamps

### Analysis of preprocess.cpp

#### MID360 Handler (Real Hardware) - Lines 504-584

```cpp
void Preprocess::mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, const int &lidar_num)
{
  pcl::PointCloud<livox_ros::LivoxPointXyzitl> pl_orig;  // Has 'line' field
  pcl::fromROSMsg(*msg, pl_orig);

  // Calculate scan angular velocity
  double omega_l = 0.361 * SCAN_RATE[lidar_num];  // deg/s

  for (uint i = 0; i < plsize; ++i)
  {
    PointType added_pt;
    // ... copy position and intensity ...

    int layer = pl_orig.points[i].line;
    double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

    // ✅ CALCULATE TIMESTAMP FOR EACH POINT based on rotation angle
    if (yaw_angle <= yaw_fp[layer])
    {
      added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;  // Time offset
    }
    else
    {
      added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
    }

    // Handle wraparound
    if (added_pt.curvature < time_last[layer])
      added_pt.curvature += 360.0 / omega_l;

    pl_surf.push_back(std::move(added_pt));
  }
}
```

**Key Features**:
- ✅ Calculates per-point timestamp based on yaw angle
- ✅ Accounts for scanner rotation using `omega_l = 0.361 * SCAN_RATE`
- ✅ Handles 360° wraparound
- ✅ Stores timestamp in `added_pt.curvature` field

---

#### Gazebo Handler (Simulation) - Lines 619-713

```cpp
void Preprocess::gazebo_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, const int &lidar_num)
{
  pcl::PointCloud<gazebo_ros::GazeboPointXyzir> pl_orig;  // Has 'ring' field only
  pcl::fromROSMsg(*msg, pl_orig);

  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    if (i % point_filter_num[lidar_num] != 0)
      continue;

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;

    // ❌ NO TIMESTAMP CALCULATION - Always set to 0!
    added_pt.curvature = 0.;

    pl_surf.points.push_back(added_pt);
  }
}
```

**Key Problems**:
- ❌ **All points have timestamp = 0.0**
- ❌ No per-point time calculation
- ❌ Assumes all points in scan captured simultaneously

---

### Gazebo Point Cloud Structure

```bash
$ ros2 topic echo /sim_lidar/lidar --once --field fields

fields:
  - name: 'x'         (offset=0,  datatype=7, count=1)
  - name: 'y'         (offset=4,  datatype=7, count=1)
  - name: 'z'         (offset=8,  datatype=7, count=1)
  - name: 'intensity' (offset=16, datatype=7, count=1)
  - name: 'ring'      (offset=24, datatype=4, count=1)
```

**Missing**: No `time` or `timestamp` field in Gazebo point cloud!

---

## Impact on FAST-LIO

### 1. Motion Compensation Failure

When a LiDAR rotates, the robot is moving. Each point is captured at a different time and position.

**With Timestamps (MID360)**:
```
Point 1: t=0.000s, robot at (0.0, 0.0, 0.0)
Point 2: t=0.005s, robot at (0.01, 0.0, 0.0)
Point 3: t=0.010s, robot at (0.02, 0.0, 0.0)
→ FAST-LIO corrects each point to common reference frame
```

**Without Timestamps (Gazebo)**:
```
Point 1: t=0.0s, robot at ???
Point 2: t=0.0s, robot at ???
Point 3: t=0.0s, robot at ???
→ FAST-LIO assumes robot didn't move → DISTORTED SCAN
```

### 2. IMU-LiDAR Fusion Breakdown

FAST-LIO uses timestamps to interpolate IMU measurements for each LiDAR point.

**With Timestamps**:
```
LiDAR point at t=0.005s → Interpolate IMU between t=0.004s and t=0.006s
→ Accurate motion estimate
```

**Without Timestamps**:
```
All LiDAR points at t=0.0s → Use only IMU at t=0.0s
→ No motion compensation during scan
→ IMU integration errors accumulate
```

### 3. Scan Deskewing Disabled

Motion deskewing requires knowing when each point was captured relative to robot motion.

- **MID360**: Points are deskewed based on calculated timestamps
- **Gazebo**: Deskewing impossible → all points treated as simultaneous

---

## Why This Causes Drift

1. **Distorted Geometry**: Without motion compensation, scans are geometrically distorted
2. **Map Inconsistency**: Distorted scans create inconsistent maps
3. **Feature Matching Errors**: ICP/point-to-plane matching fails with distorted geometry
4. **IMU Drift Accumulation**: Without LiDAR corrections at correct times, IMU drift accumulates
5. **Loop Closure Failure**: Distorted maps prevent recognizing previously visited locations

**Result**: Position error compounds over time → large drift

---

## Solution: Add Per-Point Timestamps to Gazebo Handler

### ✅ Option 1: Calculate Timestamps in gazebo_handler() [IMPLEMENTED]

**Status**: Implemented on 2025-10-29
**File**: `src/preprocess.cpp` (lines 619-792)
**Build**: Successful

Modified `preprocess.cpp` gazebo_handler to calculate timestamps similar to MID360:

```cpp
void Preprocess::gazebo_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, const int &lidar_num)
{
  pl_surf.clear();
  pcl::PointCloud<gazebo_ros::GazeboPointXyzir> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();

  // Calculate scan angular velocity
  double omega_l = 0.361 * SCAN_RATE[lidar_num];  // same as MID360

  // Track first/last yaw per ring
  std::vector<bool> is_first(N_SCANS[lidar_num], true);
  std::vector<double> yaw_fp(N_SCANS[lidar_num], 0.0);
  std::vector<float> yaw_last(N_SCANS[lidar_num], 0.0);
  std::vector<float> time_last(N_SCANS[lidar_num], 0.0);

  for (int i = 0; i < plsize; i++)
  {
    if (i % point_filter_num[lidar_num] != 0)
      continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x +
                   pl_orig.points[i].y * pl_orig.points[i].y +
                   pl_orig.points[i].z * pl_orig.points[i].z;
    if (range < (blind[lidar_num] * blind[lidar_num]))
      continue;

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;

    int ring = pl_orig.points[i].ring;
    double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

    if (is_first[ring])
    {
      yaw_fp[ring] = yaw_angle;
      is_first[ring] = false;
      added_pt.curvature = 0.0;
      yaw_last[ring] = yaw_angle;
      time_last[ring] = 0.0;
    }
    else
    {
      // Calculate timestamp based on yaw rotation (same as MID360)
      if (yaw_angle <= yaw_fp[ring])
      {
        added_pt.curvature = (yaw_fp[ring] - yaw_angle) / omega_l;
      }
      else
      {
        added_pt.curvature = (yaw_fp[ring] - yaw_angle + 360.0) / omega_l;
      }

      if (added_pt.curvature < time_last[ring])
        added_pt.curvature += 360.0 / omega_l;

      yaw_last[ring] = yaw_angle;
      time_last[ring] = added_pt.curvature;
    }

    pl_surf.points.push_back(added_pt);
  }
}
```

---

### Option 2: Add Timestamps to Gazebo LiDAR Plugin

Modify Gazebo's GPU LiDAR plugin to publish per-point timestamps based on ray index:

```cpp
// In Gazebo plugin
float time_offset = (float)ray_index / (float)total_rays * scan_period;
point.time = time_offset;
```

Then update `GazeboPointXyzir` struct to include time:
```cpp
namespace gazebo_ros
{
typedef struct {
  float x;
  float y;
  float z;
  float intensity;
  uint16_t ring;
  float time;  // ← Add this field
} GazeboPointXyzir;
}
```

---

## Configuration Check

**Current config** (`gazebosim.yaml`):
```yaml
preprocess:
  lidar_type: 5       # Gazebo
  scan_line: 16       # Changed from 128
  blind: 0.01
  scan_rate: 10
```

**Scan parameters**:
- Gazebo publishes: 2048 samples × 128 rings = 262,144 points/scan
- Scan rate: 10 Hz → 100ms per full rotation
- Per-point time span: 0 to 100ms

---

## Testing Plan

### 1. Implement Option 1 (Quick Fix)

Modify `gazebo_handler()` to calculate timestamps

### 2. Test Drift Reduction

Compare before/after:
- Static hover: < 0.1m drift in 60s
- Flight path: < 0.5m drift after 3-minute flight
- Map quality: Sharp edges, straight walls

### 3. Monitor FAST-LIO Output

```bash
# Check if motion compensation is working
ros2 topic echo /cloud_registered --once

# Verify timestamp range in processed points
# Should see curvature values from 0.0 to ~0.1 (100ms)
```

---

## Expected Results

**Before Fix (Current)**:
- ✗ Drift: ~10+ meters after 1-2 minutes
- ✗ All timestamps = 0.0
- ✗ No motion compensation
- ✗ Distorted scans

**After Fix**:
- ✓ Drift: < 0.5m after 3 minutes
- ✓ Timestamps: 0.0 to 0.1s (per scan period)
- ✓ Motion compensation enabled
- ✓ Clean, undistorted scans

---

## Comparison Summary

| Feature | MID360 (Real) | Gazebo (Current) | Solution |
|---------|---------------|------------------|----------|
| **Point Timestamp** | ✅ Calculated | ❌ Always 0.0 | Calculate from yaw |
| **Motion Compensation** | ✅ Enabled | ❌ Disabled | Enable with timestamps |
| **Scan Deskewing** | ✅ Yes | ❌ No | Add via timestamps |
| **IMU-LiDAR Sync** | ✅ Interpolated | ❌ Single time | Fix with per-point time |
| **Drift Performance** | ✅ <0.5m/3min | ❌ >10m/2min | Match real sensor |

---

## References

- FAST-LIO Source: `/home/kmedrano/ros2_ws/src/fast_lio_ros2/src/preprocess.cpp`
  - MID360 handler: Lines 504-584
  - Gazebo handler: Lines 619-713
- Gazebo Model: `/home/kmedrano/PX4-Autopilot/Tools/simulation/gz/models/x500_lidars/model.sdf`
- Config: `/home/kmedrano/ros2_ws/src/fast_lio_ros2/config/gazebosim.yaml`

---

## Implementation Status

### ✅ Completed (2025-10-29)

**Changes Made**:
1. ✅ Added timestamp calculation variables to `gazebo_handler()`
2. ✅ Implemented per-point timestamp calculation based on yaw angle
3. ✅ Applied to both feature-enabled and simple processing modes
4. ✅ Code compiled successfully

**Modified Files**:
- `src/preprocess.cpp` (lines 619-792): Complete gazebo_handler() rewrite

**Key Implementation Details**:
```cpp
// Added timestamp tracking per ring
double omega_l = 0.361 * SCAN_RATE[lidar_num];
std::vector<bool> is_first(N_SCANS[lidar_num], true);
std::vector<double> yaw_fp(N_SCANS[lidar_num], 0.0);
std::vector<float> time_last(N_SCANS[lidar_num], 0.0);

// Calculate timestamp for each point
double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
if (yaw_angle <= yaw_fp[ring])
{
  added_pt.curvature = (yaw_fp[ring] - yaw_angle) / omega_l;
}
else
{
  added_pt.curvature = (yaw_fp[ring] - yaw_angle + 360.0) / omega_l;
}
```

**Build Status**: ✅ Success
```bash
colcon build --packages-select fast_lio_ros2
# Finished in 5.81s
```

### Next Steps

1. **Restart FAST-LIO** with updated code:
   ```bash
   killall fastlio_mapping
   ros2 launch fast_lio_ros2 simulation_mapping.launch.py config_file:=gazebosim.yaml
   ```

2. **Verify Timestamps**: Check that points have varying timestamps (0.0 to ~0.1s)

3. **Test Drift Performance**: Compare with ground truth publisher

4. **Expected Improvement**: Drift reduction from >10m to <0.5m in 60s

---

**Document Version**: 1.1
**Date**: 2025-10-29
**Issue**: Gazebo simulation drift vs real MID360 performance
**Root Cause**: Missing per-point timestamps in Gazebo handler
**Solution**: ✅ Implemented timestamp calculation in gazebo_handler()
**Status**: Ready for testing
