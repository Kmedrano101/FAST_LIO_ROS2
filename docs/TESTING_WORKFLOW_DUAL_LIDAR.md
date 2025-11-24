# Testing Workflow: Dual MID-360 LiDAR with FAST-LIO

## Overview

This document provides a **step-by-step testing workflow** to validate the dual LiDAR configuration with IMU body frame reference. Each step progressively tests more functionality, allowing you to identify and fix issues early.

---

## Table of Contents

1. [Test 0: Hardware Connection Verification](#test-0-hardware-connection-verification)
2. [Test 1: Raw LiDAR Data Visualization](#test-1-raw-lidar-data-visualization)
3. [Test 2: Extrinsic Calibration Validation](#test-2-extrinsic-calibration-validation)
4. [Test 3: Single LiDAR FAST-LIO (Baseline)](#test-3-single-lidar-fast-lio-baseline)
5. [Test 4: Dual LiDAR Fusion - Static Test](#test-4-dual-lidar-fusion-static-test)
6. [Test 5: Dual LiDAR Fusion - Motion Test](#test-5-dual-lidar-fusion-motion-test)
7. [Test 6: Full System Integration](#test-6-full-system-integration)

---

## Prerequisites

### Required Packages
```bash
# Ensure all packages are built
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2 fast_lio_ros2
source install/setup.bash
```

### Hardware Setup Checklist
- [ ] Both MID-360 sensors connected via Ethernet
- [ ] LiDAR 1 (192.168.1.10) powered and accessible
- [ ] LiDAR 2 (192.168.1.18) powered and accessible
- [ ] Host IP configured: 192.168.1.124
- [ ] Network switch/router if using multiple sensors

---

## Test 0: Hardware Connection Verification

**Goal**: Verify both LiDARs are reachable and responding.

### Steps

1. **Ping both LiDARs**:
   ```bash
   ping -c 3 192.168.1.10  # LiDAR 1
   ping -c 3 192.168.1.18  # LiDAR 2
   ```

2. **Check network interface**:
   ```bash
   ip addr show  # Verify 192.168.1.124 is assigned
   ```

### Expected Results
- ✅ Both pings succeed with < 5ms latency
- ✅ Host IP 192.168.1.124 is active

### Troubleshooting
- ❌ Ping fails: Check power, Ethernet cables, network configuration
- ❌ High latency: Check for network congestion or switch issues

---

## Test 1: Raw LiDAR Data Visualization

**Goal**: Verify both LiDARs are publishing point cloud data independently.

### Steps

1. **Launch Livox driver only**:
   ```bash
   ros2 launch livox_ros_driver2 multiple_lidars.launch.py
   ```

2. **In separate terminal, check topics**:
   ```bash
   ros2 topic list | grep livox
   ```

3. **Check data rates**:
   ```bash
   ros2 topic hz /livox/lidar_192_168_1_10
   ros2 topic hz /livox/lidar_192_168_1_18
   ros2 topic hz /livox/imu_192_168_1_10
   ```

4. **Visualize in RViz2**:
   ```bash
   rviz2
   ```
   - Add → PointCloud2 → Topic: `/livox/lidar_192_168_1_10`
   - Add → PointCloud2 → Topic: `/livox/lidar_192_168_1_18`
   - Set Fixed Frame: `livox_frame`
   - Change colors to distinguish sensors (L1=Red, L2=Blue)

### Expected Results
- ✅ Both topics publishing at ~10 Hz
- ✅ IMU topic publishing at ~200 Hz
- ✅ Point clouds visible in RViz (may be misaligned at this stage)
- ✅ No error messages in driver output

### Troubleshooting
- ❌ Topic missing: Check IP configuration in `multiple_netconfigs.json`
- ❌ Low/zero Hz: Sensor may be initializing or network issue
- ❌ No IMU data: Check IMU port configuration

### Screenshot
Take a screenshot showing both point clouds (unaligned is OK).

---

## Test 2: Extrinsic Calibration Validation

**Goal**: Verify extrinsic transforms align the sensors correctly.

### Steps

1. **Use calibration visualization launch file**:
   ```bash
   ros2 launch livox_ros_driver2 lidar_calibration_viz.launch.py
   ```

2. **In RViz2**:
   - Fixed Frame: `base_link`
   - Enable TF display (check frames: `base_link`, `lidar_L1`, `lidar_L2`)
   - Add both LiDAR point cloud topics
   - Color: L1=Red, L2=Blue

3. **Static scene test**:
   - Point both sensors at a **flat wall** or **corridor**
   - Keep robot **completely stationary**
   - Observe for 10-20 seconds

### Expected Results
- ✅ TF tree shows: `base_link` → `lidar_L1` and `base_link` → `lidar_L2`
- ✅ Wall appears as **single thin plane** (10-20cm thick)
- ✅ No obvious ghosting or double features
- ✅ Corners align sharply (90° angles)

### Pass/Fail Criteria

| Observation | Status | Action |
|-------------|--------|--------|
| Wall is single thin plane (10-20cm) | ✅ PASS | Continue to Test 3 |
| Wall is thick (>40cm) or doubled | ❌ FAIL | Check extrinsics, re-measure |
| Corners are misaligned | ❌ FAIL | Rotation matrix may be wrong |
| Features shifted laterally | ❌ FAIL | Translation vector incorrect |

### Troubleshooting
- ❌ Thick walls: Extrinsic translation may be wrong
- ❌ Rotational misalignment: Check rotation matrices
- ❌ TF errors: Run `ros2 run tf2_tools view_frames` to debug

### Screenshot
Take a screenshot showing aligned wall/corner in RViz.

---

## Test 3: Single LiDAR FAST-LIO (Baseline)

**Goal**: Establish baseline performance with single LiDAR before testing fusion.

### Configuration Changes

**Create test config**: `dual_mid360_mine_test_single.yaml`

```bash
cd ~/ros2_ws/src/fast_lio_ros2/config
cp dual_mid360_mine.yaml dual_mid360_mine_test_single.yaml
```

**Edit `dual_mid360_mine_test_single.yaml`**:

```yaml
/**:
  ros__parameters:
    # DISABLE multi-LiDAR for baseline test
    multi_lidar: false                  # ← Change from true to false

    common:
      # Use ONLY LiDAR 1
      lid_topic: "/livox/lidar_192_168_1_10"
      # lid_topic2: "/livox/lidar_192_168_1_18"  # ← Comment out L2
      imu_topic: "/livox/imu_192_168_1_10"
```

### Steps

1. **Launch FAST-LIO with single LiDAR**:
   ```bash
   ros2 launch fast_lio_ros2 mapping.launch.py config_file:=dual_mid360_mine_test_single.yaml
   ```

2. **Open RViz2 with FAST-LIO config**:
   ```bash
   rviz2 -d ~/ros2_ws/src/fast_lio_ros2/rviz/fastlio.rviz
   ```

3. **Check topics**:
   ```bash
   ros2 topic hz /cloud_registered
   ros2 topic hz /Odometry
   ros2 topic hz /path
   ```

4. **Static test (30 seconds stationary)**:
   - Keep robot still
   - Observe odometry drift (should be minimal)
   - Check map quality

5. **Motion test**:
   - Slowly move robot forward/backward (1 m)
   - Rotate in place (90°)
   - Return to start position

### Expected Results
- ✅ FAST-LIO initializes without errors
- ✅ `/cloud_registered` publishing at ~10 Hz
- ✅ Odometry drift < 5cm when stationary
- ✅ Map is consistent during motion
- ✅ Robot returns close to start position (< 10cm error)

### Performance Baselines
Record these values for comparison:

| Metric | Value | Notes |
|--------|-------|-------|
| Update rate (Hz) | _____ | From `/cloud_registered` |
| Static drift (cm/min) | _____ | Position drift when still |
| Loop closure error (cm) | _____ | Error returning to start |
| Feature density (pts/scan) | _____ | From terminal output |

### Troubleshooting
- ❌ Initialization fails: Check IMU calibration, ensure robot is level
- ❌ High drift: IMU noise parameters may need tuning (`acc_cov`, `gyr_cov`)
- ❌ Degenerate cases: Increase `point_filter_num` if too many points

---

## Test 4: Dual LiDAR Fusion - Static Test

**Goal**: Test dual LiDAR fusion without motion (easiest case).

### Configuration Changes

**Edit `dual_mid360_mine.yaml`** (or create test variant):

```yaml
/**:
  ros__parameters:
    # ENABLE multi-LiDAR
    multi_lidar: true

    # Start with BUNDLE mode (most robust)
    update_mode: 0  # 0=Bundle, 1=Async, 2=Adaptive

    common:
      lid_topic: "/livox/lidar_192_168_1_10"
      lid_topic2: "/livox/lidar_192_168_1_18"  # ← Ensure uncommented
      imu_topic: "/livox/imu_192_168_1_10"
```

### Steps

1. **Launch dual LiDAR FAST-LIO**:
   ```bash
   ros2 launch fast_lio_ros2 mapping.launch.py config_file:=dual_mid360_mine.yaml
   ```

2. **Static environment test**:
   - Place robot in corridor or room with clear geometry
   - **Keep completely stationary** for 60 seconds
   - Observe map building

3. **In RViz2, check**:
   - `/cloud_registered` (global map)
   - `/cloud_registered_body` (current scan in body frame)
   - TF frames

4. **Save point cloud**:
   ```bash
   ros2 service call /save_pcd std_srvs/srv/Trigger
   ```

5. **Inspect saved PCD**:
   - Check file: `~/ros2_ws/src/fast_lio_ros2/PCD/mine_dual_mid360.pcd`
   - Open in CloudCompare or similar tool
   - Measure wall thickness

### Expected Results
- ✅ Both LiDAR scans fused without errors
- ✅ Wall thickness: 10-20cm (single clean surface)
- ✅ No ghosting or double features
- ✅ Feature overlap regions show smooth transition
- ✅ Update rate ~10 Hz (may be slightly lower than single LiDAR)

### Pass/Fail Criteria

| Test | Pass ✅ | Fail ❌ |
|------|---------|---------|
| Wall thickness | < 25cm | > 40cm |
| Ghosting | None visible | Double walls/objects |
| Corners | Sharp 90° | Rounded or misaligned |
| Console errors | None | TF errors, segfaults |

### Troubleshooting
- ❌ Thick walls: Re-check extrinsics (go back to Test 2)
- ❌ Segfault/crash: Check LiDAR topic names match config
- ❌ Only one sensor working: Check `multi_lidar: true` and `lid_topic2`

---

## Test 5: Dual LiDAR Fusion - Motion Test

**Goal**: Validate fusion during robot motion.

### Test Scenarios

#### Scenario A: Straight Line Motion
1. Start in corridor
2. Move slowly forward 5 meters
3. Stop
4. Move backward to start position
5. Check loop closure error

**Expected**: < 20cm position error

#### Scenario B: Rotation Test
1. Start in open area
2. Rotate 360° in place (slow, ~30 sec/rotation)
3. Check for circular map consistency

**Expected**: Complete 360° map with no drift/gaps

#### Scenario C: Combined Motion
1. Drive in rectangular path (2m × 3m)
2. Return to start position
3. Measure loop closure error

**Expected**: < 30cm position error

### Configuration Variants to Test

Test each update mode:

```yaml
update_mode: 0  # Bundle - most robust, lower rate
update_mode: 1  # Async - higher rate, needs good FOV
update_mode: 2  # Adaptive - best of both
```

### Performance Comparison

| Update Mode | Scenario A Error (cm) | Scenario B Quality | Scenario C Error (cm) | Notes |
|-------------|----------------------|-------------------|----------------------|-------|
| Bundle (0) | _____ | _____ | _____ | |
| Async (1) | _____ | _____ | _____ | |
| Adaptive (2) | _____ | _____ | _____ | |

### Troubleshooting
- ❌ High drift: Tune `acc_cov`, `gyr_cov` for vibration
- ❌ Async mode fails: Increase `adaptive_fov_threshold`
- ❌ Degenerate cases: Sparse features, need bundle mode

---

## Test 6: Full System Integration

**Goal**: Long-duration test with all features enabled.

### Configuration

Use final configuration with optimal settings from previous tests:

```yaml
/**:
  ros__parameters:
    multi_lidar: true
    update_mode: 2  # Adaptive (or best from Test 5)
    adaptive_debug_output: true  # Enable for monitoring
```

### Long-Duration Test Protocol

1. **30-minute continuous mapping**:
   - Navigate full environment
   - Include various speeds (slow, medium, fast)
   - Test different lighting (dusty areas)
   - Record rosbag for analysis

2. **Monitor during test**:
   ```bash
   # Terminal 1: Watch update rate
   watch -n 1 'ros2 topic hz /cloud_registered'

   # Terminal 2: Watch mode switches (if adaptive)
   ros2 topic echo /fast_lio/mode  # If published

   # Terminal 3: Monitor CPU/memory
   htop
   ```

3. **Record metrics**:
   - Average update rate
   - Max/min update rate
   - CPU usage (%)
   - Memory usage (GB)
   - Mode switch count (if adaptive)

### Performance Targets

| Metric | Target | Measured | Pass/Fail |
|--------|--------|----------|-----------|
| Avg update rate | > 8 Hz | _____ | _____ |
| CPU usage | < 80% | _____ | _____ |
| Memory usage | < 6 GB | _____ | _____ |
| Map consistency | No drift | _____ | _____ |
| Feature density | > 3000 pts/scan | _____ | _____ |

### Save Final Map

```bash
# PCD map automatically saved on shutdown
# Or trigger manually:
ros2 service call /save_pcd std_srvs/srv/Trigger

# Map saved to:
~/ros2_ws/src/fast_lio_ros2/PCD/mine_dual_mid360.pcd
```

### Post-Test Analysis

1. **Inspect PCD in CloudCompare**:
   - Check overall quality
   - Measure feature dimensions
   - Look for artifacts or misalignment

2. **Review rosbag**:
   ```bash
   ros2 bag info test_run_<timestamp>
   ```

3. **Document issues**:
   - Any crashes or errors
   - Performance bottlenecks
   - Areas for tuning

---

## Quick Reference: Common Issues

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Thick walls (>40cm) | Wrong extrinsics | Re-measure, check rotation matrices |
| Ghosting/double features | Misaligned rotation | Verify Euler → rotation matrix conversion |
| Single LiDAR only working | `multi_lidar: false` | Set to `true`, check `lid_topic2` |
| Low update rate (<5 Hz) | Too many points | Increase `point_filter_num` |
| High CPU usage (>90%) | Too dense scans | Increase `filter_size_surf`, `filter_size_map` |
| Drift during static test | IMU noise | Increase `acc_cov`, `gyr_cov` |
| Degenerate/failed tracking | Insufficient features | Use bundle mode, decrease `blind` distance |
| Segfault on launch | Topic mismatch | Verify topic names in config vs. driver |

---

## Validation Checklist

Before moving to production use:

- [ ] Test 0: Hardware connectivity verified
- [ ] Test 1: Both LiDARs publishing data
- [ ] Test 2: Extrinsics properly aligned (thin walls)
- [ ] Test 3: Single LiDAR baseline established
- [ ] Test 4: Static dual fusion working
- [ ] Test 5: Motion fusion tested in all modes
- [ ] Test 6: 30-min integration test passed
- [ ] Final PCD map inspected and approved
- [ ] Performance metrics meet targets
- [ ] Configuration documented and backed up

---

## Next Steps After Validation

1. **Fine-tune parameters** based on test results
2. **Create production launch files** with optimal settings
3. **Document your specific environment** (mine layout, typical features)
4. **Set up monitoring/logging** for long-term operation
5. **Train operators** on system usage and troubleshooting

---

## Appendix A: RViz Configuration

### Recommended Display Setup

**For Calibration Validation:**
- Fixed Frame: `base_link`
- TF: Enabled, show axes
- PointCloud2 (L1): Topic `/livox/lidar_192_168_1_10`, Color=Red
- PointCloud2 (L2): Topic `/livox/lidar_192_168_1_18`, Color=Blue
- Grid: 1m spacing

**For FAST-LIO Mapping:**
- Fixed Frame: `camera_init` (or `map`)
- PointCloud2 (Map): Topic `/cloud_registered`, Color=Intensity
- PointCloud2 (Scan): Topic `/cloud_registered_body`, Color=Z-axis
- Path: Topic `/path`, Color=Green
- Odometry: Topic `/Odometry`, Covariance enabled
- TF: Enabled

---

## Appendix B: Useful Commands

```bash
# Check all FAST-LIO topics
ros2 topic list | grep -E "(cloud|Odometry|path)"

# Monitor all rates
ros2 topic hz /cloud_registered & ros2 topic hz /Odometry & ros2 topic hz /path

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Echo extrinsic parameters (if using dynamic_reconfigure)
ros2 param list /fast_lio_ros2_node

# Check for errors
ros2 topic echo /diagnostics

# Save current RViz config
# In RViz: File → Save Config As
```

---

**Document Version**: 1.0
**Last Updated**: 2025-11-24
**Author**: Testing workflow for dual MID-360 FAST-LIO integration
