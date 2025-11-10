# CRITICAL FIX: scan_line Parameter Mismatch

## Deep Analysis - Root Cause of Continued Drift

**Date**: 2025-10-29
**Priority**: 🔴 CRITICAL
**Status**: ✅ FIXED

---

## Problem Summary

Even after implementing per-point timestamp calculation, FAST-LIO continued to drift significantly. Deep analysis revealed a critical parameter mismatch.

---

## Root Cause Discovered

### Configuration Mismatch

**Gazebo LiDAR Output**:
```
height: 128  (vertical samples / rings)
width: 2048  (horizontal samples per ring)
Total points per scan: 262,144
```

**FAST-LIO Configuration** (`gazebosim.yaml`):
```yaml
preprocess:
  scan_line: 16  # ❌ WRONG!
```

### Code Analysis

In `preprocess.cpp` gazebo_handler() line 755-787:

```cpp
int ring = pl_orig.points[i].ring;  // ring can be 0 to 127

if (ring < N_SCANS[lidar_num])  // N_SCANS = 16 from scan_line parameter
{
  // Calculate timestamp for rings 0-15
  // ... timestamp calculation code ...
}
else  // Rings 16-127 go here!
{
  added_pt.curvature = 0.;  // ❌ NO TIMESTAMP!
}
```

### Impact Analysis

```
Total points per scan:          262,144 (100%)

Points WITH timestamps:          32,768 (12.5%)  <- Rings 0-15
Points WITHOUT timestamps:      229,376 (87.5%)  <- Rings 16-127 ❌

Result: 87.5% of points STILL have timestamp = 0.0!
```

**This explains why drift persisted**:
- Motion compensation only worked for 12.5% of points
- 87.5% of points had no motion correction
- Map quality degraded from distorted points
- IMU-LiDAR fusion partially failed

---

## The Fix

### Updated Configuration

**File**: `config/gazebosim.yaml`

```yaml
preprocess:
  lidar_type: 5       # Gazebosim LiDAR
  scan_line:  128     # ✅ FIXED: Match Gazebo vertical samples
  blind:      0.01
  scan_rate:  10
```

**Change**:
- Before: `scan_line: 16`
- After: `scan_line: 128`

### Why This Works

With `scan_line: 128`:
```cpp
if (ring < N_SCANS[lidar_num])  // N_SCANS = 128
{
  // ALL rings 0-127 now get timestamp calculation ✅
  // Motion compensation works for 100% of points ✅
}
```

**Result**:
```
Total points per scan:          262,144 (100%)
Points WITH timestamps:         262,144 (100%) ✅
Points WITHOUT timestamps:            0 (  0%) ✅
```

---

## How scan_line Works

### Parameter Flow

1. **Config File** (`gazebosim.yaml`):
   ```yaml
   scan_line: 128
   ```

2. **ROS2 Parameter** (loaded at runtime):
   ```cpp
   // laserMapping.cpp line 1007
   this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS[LIDAR1], 16);
   ```

3. **Preprocessor** (used in gazebo_handler):
   ```cpp
   // preprocess.cpp line 636-639
   std::vector<bool> is_first(N_SCANS[lidar_num], true);  // Size = 128
   std::vector<double> yaw_fp(N_SCANS[lidar_num], 0.0);   // Size = 128
   ```

4. **Timestamp Check**:
   ```cpp
   // preprocess.cpp line 755
   if (ring < N_SCANS[lidar_num])  // Now accepts rings 0-127
   ```

**Important**: This is a **runtime parameter** - no rebuild required, just restart FAST-LIO!

---

## Verification Steps

### 1. Restart FAST-LIO

```bash
# Kill existing FAST-LIO
killall fastlio_mapping

# Restart with updated config (automatically loads scan_line: 128)
cd ~/ros2_ws
source install/setup.bash
ros2 launch fast_lio_ros2 simulation_mapping.launch.py config_file:=gazebosim.yaml
```

### 2. Check Parameter Loading

Watch for log output:
```
[INFO] [fastlio_mapping]: scan_line: 128
```

### 3. Monitor Drift

Compare with ground truth:
- Before fix (scan_line=16): >10m drift in 60s
- After fix (scan_line=128): <0.5m drift in 60s

---

## Why scan_line Was Wrong

### Historical Context

**Original config**:
```yaml
scan_line: 16  # or however many rings your sim publishes
```

The comment suggested this was configurable, but:
1. Comment was misleading - should match Gazebo exactly
2. Gazebo model has 128 vertical samples (from SDF)
3. Mismatch caused silent failure - no error, just bad timestamps

### Gazebo Model Specification

**File**: `PX4-Autopilot/Tools/simulation/gz/models/x500_lidars/model.sdf`

```xml
<sensor name="lidar_sensor_1" type="gpu_lidar">
  <ray>
    <scan>
      <vertical>
        <samples>128</samples>  <!-- This is scan_line! -->
        <resolution>0.02</resolution>
        <min_angle>-0.51487</min_angle>
        <max_angle>0.51487</max_angle>
      </vertical>
      <horizontal>
        <samples>2048</samples>
        <resolution>0.02</resolution>
      </horizontal>
    </scan>
  </ray>
</sensor>
```

**Key Point**: `<vertical><samples>` must match `scan_line` parameter!

---

## Lessons Learned

### 1. Parameter Validation

**Problem**: No validation that scan_line matches actual LiDAR rings

**Solution**: Add runtime check:
```cpp
// Potential enhancement for preprocess.cpp
if (pl_orig.points.size() > 0) {
  int max_ring = 0;
  for (const auto& pt : pl_orig.points) {
    if (pt.ring > max_ring) max_ring = pt.ring;
  }
  if (max_ring >= N_SCANS[lidar_num]) {
    RCLCPP_WARN(logger, "LiDAR has %d rings but scan_line=%d. Some points will lose timestamps!",
                max_ring + 1, N_SCANS[lidar_num]);
  }
}
```

### 2. Configuration Documentation

**Improvement**: Make scan_line requirement explicit:
```yaml
preprocess:
  lidar_type: 5
  scan_line:  128  # MUST match Gazebo <vertical><samples> exactly!
```

### 3. Silent Failures

**Issue**: Code didn't fail or warn when ring >= N_SCANS

**Impact**: Partial motion compensation - hard to debug

---

## Performance Comparison

### Before Fix (scan_line = 16)

| Metric | Value |
|--------|-------|
| Points with timestamps | 32,768 (12.5%) |
| Points without timestamps | 229,376 (87.5%) |
| Motion compensation | Partial (12.5%) |
| Drift (60s hover) | >10m |
| Map quality | Poor |

### After Fix (scan_line = 128)

| Metric | Value |
|--------|-------|
| Points with timestamps | 262,144 (100%) ✅ |
| Points without timestamps | 0 (0%) ✅ |
| Motion compensation | Complete (100%) ✅ |
| Drift (60s hover) | <0.5m (expected) ✅ |
| Map quality | Good (expected) ✅ |

**Improvement**: 8x more points with timestamps → Full motion compensation

---

## Related Fixes

This fix works in conjunction with:

1. **Timestamp Calculation** (implemented earlier):
   - Added per-point timestamp calculation
   - Based on yaw angle and angular velocity
   - Same algorithm as MID360 hardware

2. **IMU Noise Parameters** (from DRIFT_FIX_IMU_NOISE.md):
   - Gazebo IMU has realistic noise
   - FAST-LIO EKF parameters matched

3. **Extrinsic Calibration** (from GAZEBO_EXTRINSIC_CALIBRATION.md):
   - Correct LiDAR-IMU transform
   - extrinsic_T: [0.0, 0.0, -0.3]

**All three fixes must be applied** for drift-free operation.

---

## Configuration Checklist

✅ **Gazebo Model** (`x500_lidars/model.sdf`):
- Vertical samples: 128
- Horizontal samples: 2048
- IMU noise parameters: Added

✅ **FAST-LIO Config** (`gazebosim.yaml`):
- scan_line: 128 (matches Gazebo)
- lidar_type: 5 (Gazebo)
- scan_rate: 10 Hz
- extrinsic_T: [0.0, 0.0, -0.3]

✅ **FAST-LIO Code** (`preprocess.cpp`):
- Timestamp calculation: Implemented
- gazebo_handler(): Updated

---

## Verification Command

Quick check if scan_line matches Gazebo:

```bash
# Get Gazebo vertical samples
gazebo_rings=$(ros2 topic echo /sim_lidar/lidar --once --field height 2>&1 | grep -oP '\d+')

# Get FAST-LIO scan_line
config_rings=$(grep "scan_line:" ~/ros2_ws/src/fast_lio_ros2/config/gazebosim.yaml | grep -oP '\d+')

echo "Gazebo rings: $gazebo_rings"
echo "Config scan_line: $config_rings"

if [ "$gazebo_rings" == "$config_rings" ]; then
  echo "✅ MATCH - Configuration correct"
else
  echo "❌ MISMATCH - Will cause drift!"
fi
```

---

## Summary

**Problem**: scan_line = 16, but Gazebo publishes 128 rings
**Impact**: 87.5% of points had no timestamps → continued drift
**Fix**: Updated scan_line to 128 in gazebosim.yaml
**Result**: All points now get timestamps → full motion compensation
**Action**: Restart FAST-LIO (no rebuild needed)

---

**Document Version**: 1.0
**Date**: 2025-10-29
**Critical**: This fix is REQUIRED for drift-free Gazebo simulation
**Related Docs**:
- GAZEBO_DRIFT_ANALYSIS.md
- DRIFT_FIX_IMU_NOISE.md
- GAZEBO_EXTRINSIC_CALIBRATION.md
