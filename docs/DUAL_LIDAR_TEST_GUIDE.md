# Dual LiDAR Test and Alignment Verification Guide

## Overview
This guide explains how to test the dual Livox MID-360 FAST-LIO implementation and verify extrinsic calibration alignment.

---

## Quick Start

### On Jetson (Robot/Payload)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch fast_lio_ros2 dual_mapping_core.launch.py
```

### On Desktop (Visualization)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch fast_lio_ros2 dual_mapping_desktop.launch.py
```

---

## Test Scenarios

### Test 1: ASYNC Mode (Default)
**Purpose**: Verify both LiDARs are receiving data and publishing correctly

**Config**: Set in `config/dual_mid360.yaml`
```yaml
update_method: 1  # ASYNC mode
```

**Expected Behavior**:
- You should see alternating scans from LiDAR 1 (green) and LiDAR 2 (red)
- Each LiDAR updates independently at ~10Hz
- Total update rate: ~20Hz combined

**Console Logs to Expect**:
```
[INFO] [laser_mapping]: Update method: 1 (ASYNC)
[INFO] [laser_mapping]: [LiDAR 1] First scan received!
[INFO] [laser_mapping]: [LiDAR 2] First scan received!
[ASYNC] Using LiDAR 1 scan
[ASYNC] Using LiDAR 2 scan
```

**RViz Verification**:
1. Enable `/lidar1_colored` display (should be GREEN)
2. Enable `/lidar2_colored` display (should be RED)
3. Disable `/cloud_registered` temporarily
4. Observe alternating point clouds
5. Both sensors should cover different FOV regions

---

### Test 2: BUNDLE Mode
**Purpose**: Verify synchronized fusion of both LiDAR scans

**Config**: Set in `config/dual_mid360.yaml`
```yaml
update_method: 0  # BUNDLE mode
```

**Expected Behavior**:
- System waits for synchronized scans from both LiDARs
- Merges them into single dense scan
- Update rate: ~10Hz (half of ASYNC)
- Higher point density per scan

**Console Logs to Expect**:
```
[INFO] [laser_mapping]: Update method: 0 (BUNDLE)
[BUNDLE] Merged scans: L1=8234 pts, L2=7891 pts, Total=16125 pts
```

**Warning Logs (Normal)**:
```
[WARN] [BUNDLE] Time desync: 12.3 ms, discarding older scan
```
*This is normal - indicates temporal synchronization working*

**RViz Verification**:
1. `/lidar1_colored` and `/lidar2_colored` will be empty (merged mode)
2. Enable `/cloud_registered` - should show denser combined scan
3. Verify 360° coverage with both sensors

---

### Test 3: ADAPTIVE Mode
**Purpose**: Test intelligent switching between ASYNC and BUNDLE based on point quality

**Config**: Set in `config/dual_mid360.yaml`
```yaml
update_method: 2  # ADAPTIVE mode
voxelized_pt_num_thres: 100
effect_pt_num_ratio_thres: 0.5
```

**Expected Behavior**:
- Starts in BUNDLE mode
- Switches to ASYNC when sufficient feature points detected
- Switches back to BUNDLE when features lacking (e.g., featureless corridor)
- Includes 10-scan hysteresis to prevent oscillation

**Console Logs to Expect**:
```
[INFO] [laser_mapping]: Update method: 2 (ADAPTIVE)
[INFO] [laser_mapping]: Adaptive thresholds: voxelized_pts >= 100, effect_ratio >= 0.50
[ADAPTIVE] Switching to ASYNC mode (voxelized=250, effect=150, ratio=60.00%)
[ADAPTIVE] Switching to BUNDLE mode (voxelized=80, effect=30, ratio=37.50%, tic=10)
```

---

## Extrinsic Calibration Verification

### Visual Alignment Check

#### Step 1: Static Scene Test
1. Launch in ASYNC mode
2. Place robot in environment with clear geometric features:
   - Walls
   - Corners
   - Doorways
   - Pipes/beams
3. Let robot remain stationary
4. In RViz:
   - Enable `/lidar1_colored` (GREEN)
   - Enable `/lidar2_colored` (RED)
   - Set decay time to 10 seconds

#### Step 2: Overlap Analysis
**What to look for**:
- In overlapping FOV regions, GREEN and RED points should coincide
- Walls should appear as single line, not doubled
- Corners should be sharp, not blurred
- Parallel lines should remain parallel

**Good Alignment**:
```
Wall view (top-down):
GREEN:  ═══════════
RED:    ═══════════
Result: Perfect overlap
```

**Bad Alignment (Translation Error)**:
```
Wall view (top-down):
GREEN:  ═══════════
RED:       ═══════════
Result: Offset wall (adjust extrinsic_T_2)
```

**Bad Alignment (Rotation Error)**:
```
Wall view (top-down):
GREEN:  ═══════════
RED:    ═══════╱
Result: Angled wall (adjust extrinsic_R_2)
```

#### Step 3: Dynamic Test
1. Move robot slowly through environment
2. Watch for:
   - Consistent alignment during motion
   - No "swimming" or relative motion between clouds
   - Features remain aligned across different distances

---

### Quantitative Alignment Metrics

#### Metric 1: Point Cloud Distance
Use PCL tools to measure distance between corresponding points:
```bash
# Save both clouds
ros2 topic echo /lidar1_colored --once > l1.pcd
ros2 topic echo /lidar2_colored --once > l2.pcd

# Calculate ICP alignment error (should be < 2cm for good calibration)
```

#### Metric 2: Feature Correspondence
- Identify 10-20 clear corner points in both clouds
- Measure 3D Euclidean distance
- **Threshold**: Average error < 2cm, Max error < 5cm

---

## Extrinsic Calibration Adjustment

### Current Values
**File**: `config/dual_mid360.yaml`

**LiDAR 1 → IMU** (Front sensor, IP: 192.168.1.10):
```yaml
extrinsic_T_1: [-0.011, 0.02329, -0.15412]  # [x, y, z] meters
extrinsic_R_1: [-1.00000,  0.00000,  0.00000,
                 0.00000, -1.00000,  0.00000,
                 0.00000,  0.00000,  1.00000]
```

**LiDAR 2 → IMU** (Rear sensor, IP: 192.168.1.18):
```yaml
extrinsic_T_2: [-0.011, -0.02329, -0.15412]  # [x, y, z] meters
extrinsic_R_2: [-1.00000,  0.00000,  0.00000,
                 0.00000, -1.00000,  0.00000,
                 0.00000,  0.00000,  1.00000]
```

### Adjustment Procedure

#### Translation Misalignment
**Symptom**: Walls/features appear shifted

**Solution**:
1. Measure offset in RViz (use measurement tool)
2. Adjust `extrinsic_T_2`:
   - X offset: Forward/backward
   - Y offset: Left/right
   - Z offset: Up/down
3. Restart `dual_mapping_core.launch.py`
4. Verify improvement

#### Rotation Misalignment
**Symptom**: Walls appear angled, corners not sharp

**Solution**:
1. Use rotation matrix calculator
2. Small rotations (< 5°) can be approximated:
   ```
   Rotation around Z (yaw):
   cos(θ)  -sin(θ)  0
   sin(θ)   cos(θ)  0
   0        0       1
   ```
3. Update `extrinsic_R_2`
4. Restart and verify

---

## Performance Monitoring

### Topic Hz Check
```bash
# Should be ~10Hz each in ASYNC mode
ros2 topic hz /lidar1_colored
ros2 topic hz /lidar2_colored

# Should be ~10Hz
ros2 topic hz /Odometry

# Should match LiDAR rate
ros2 topic hz /cloud_registered
```

### CPU Usage (on Jetson)
```bash
top -p $(pgrep fastlio_mapping)
```
**Expected**: 200-300% CPU (using 2-3 cores efficiently)

### Memory Usage
```bash
ros2 run fast_lio_ros2 fastlio_mapping --ros-args --param runtime_pos_log_enable:=true
```
**Expected**: < 2GB RAM

---

## Troubleshooting

### Problem: Only one LiDAR shows data

**Check**:
```bash
# Verify topics exist
ros2 topic list | grep lidar

# Check data rate
ros2 topic hz /livox/lidar_192_168_1_10
ros2 topic hz /livox/lidar_192_168_1_18
```

**Solution**:
- Verify `multi_lidar: true` in config
- Check Livox driver configuration
- Ensure both sensors powered and connected

### Problem: Severe misalignment

**Check**:
1. Verify physical sensor mounting
2. Confirm sensor IPs match config
3. Check if sensors were swapped during installation

**Solution**:
- Re-measure sensor positions relative to IMU
- Use CAD model if available
- Perform manual calibration procedure

### Problem: Bundle mode shows desync warnings

**This is normal!** The 50ms tolerance ensures:
- Temporal alignment
- Only synchronized scans are merged

**Excessive warnings (>50% of scans)**:
- Check sensor synchronization
- Verify both sensors at same scan rate (10Hz)
- Check for network latency issues

---

## Data Collection for Calibration

### Calibration Dataset Capture
```bash
# Record topics for offline calibration
ros2 bag record /livox/lidar_192_168_1_10 /livox/lidar_192_168_1_18 /livox/imu_transformed

# Move robot to capture:
# - Walls at multiple angles
# - Corners and edges
# - Objects at various distances (5-30m)
# - 360° rotation in place
```

### Recommended Calibration Tools
- **Livox Calibration Tool**: Official tool for multi-LiDAR calibration
- **MCalib**: Multi-LiDAR automatic calibration
- **Manual correspondence**: For fine-tuning

---

## Success Criteria

### ✅ System is working correctly when:
1. Both LiDAR topics publishing at 10Hz
2. Console shows alternating L1/L2 scans (ASYNC) or merged scans (BUNDLE)
3. Visual alignment error < 2cm in overlapping regions
4. No persistent drift during static tests
5. Trajectory smooth and continuous
6. Map quality matches single-LiDAR performance

### ❌ Issues requiring attention:
1. Only one LiDAR active
2. Alignment error > 5cm
3. Clouds "swimming" relative to each other
4. Frequent temporal desync warnings (>50%)
5. Crashes or seg faults
6. Trajectory jumps or discontinuities

---

## Advanced Testing

### Stress Test: Fast Motion
1. Move robot at maximum speed
2. Verify no scan drops
3. Check trajectory continuity

### Stress Test: Feature-poor Environment
1. Long corridor with uniform walls
2. Adaptive mode should switch to BUNDLE
3. Verify localization doesn't drift

### Stress Test: Dense Features
1. Cluttered environment
2. Adaptive mode should stay in ASYNC
3. Verify high update rate maintained

---

## Contact and Support

For issues specific to dual LiDAR implementation:
1. Check GitHub issues: https://github.com/anthropics/fast_lio_ros2
2. Review configuration in `config/dual_mid360.yaml`
3. Enable debug logging: `async_debug: true`

---

## Version Info
- FAST-LIO: ROS2 Dual LiDAR Implementation
- Tested with: Livox MID-360 (Firmware >= 1.0)
- ROS2: Humble/Iron
- Platform: NVIDIA Jetson Orin (tested), x86_64 (should work)
