# Quick Test Guide: Dual MID-360 LiDAR System

## Overview
Fast reference for testing your dual LiDAR FAST-LIO setup with progressive feature validation.

---

## Prerequisites

```bash
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2 fast_lio_ros2
source install/setup.bash
```

**Hardware checklist:**
- [ ] Both LiDARs powered and connected (192.168.1.10, 192.168.1.18)
- [ ] Host IP: 192.168.1.124
- [ ] Ping test passed for both sensors

---

## Test Sequence

### 🔍 Test 1: Raw LiDAR Visualization

**Purpose:** Verify both LiDARs publish data

```bash
# Terminal 1
ros2 launch fast_lio_ros2 test_1_raw_lidar.launch.py

# Terminal 2 (optional - check rates)
ros2 topic hz /livox/lidar_192_168_1_10
ros2 topic hz /livox/lidar_192_168_1_18
ros2 topic hz /livox/imu_192_168_1_10
```

**Expected:**
- ✅ Both point cloud topics at ~10 Hz
- ✅ IMU at ~200 Hz
- ✅ Point clouds visible in RViz (may be misaligned)

**RViz Colors:**
- LiDAR 1 (192.168.1.10) = RED
- LiDAR 2 (192.168.1.18) = BLUE

---

### 📐 Test 2: Extrinsic Calibration Validation

**Purpose:** Verify extrinsic transforms align sensors

```bash
ros2 launch fast_lio_ros2 test_2_extrinsic_validation.launch.py
```

**What to do:**
1. Point robot at flat wall or corridor
2. Keep completely stationary for 10-20 seconds
3. Observe wall/corner alignment in RViz

**Pass Criteria:**
- ✅ Wall is single thin plane (10-20cm thick)
- ✅ No ghosting or double features
- ✅ Corners are sharp 90° angles
- ✅ TF tree shows: `base_link` → `lidar_1_frame` and `lidar_2_frame`

**Fail → Re-check extrinsics:**
- ❌ Thick walls (>40cm): Translation vector wrong
- ❌ Rotational misalignment: Rotation matrix wrong
- ❌ Go back and verify `/home/kmedrano/ros2_ws/src/fast_lio_ros2/config/dual_mid360_mine.yaml`

---

### 🎯 Test 3: Single LiDAR Baseline

**Purpose:** Establish baseline performance before dual fusion

```bash
ros2 launch fast_lio_ros2 test_3_single_lidar.launch.py
```

**Static Test (30 seconds):**
- Keep robot still
- Record odometry drift

**Motion Test:**
- Move forward/backward 1m
- Rotate 90° in place
- Return to start position

**Record Baseline Metrics:**

| Metric | Value |
|--------|-------|
| Update rate (Hz) | _____ |
| Static drift (cm/min) | _____ |
| Loop closure error (cm) | _____ |
| Feature density (pts/scan) | _____ |

**Expected:**
- ✅ Update rate: ~10 Hz
- ✅ Static drift: < 5cm
- ✅ Loop closure: < 10cm

---

### 🔄 Test 4: Dual LiDAR Static Fusion

**Purpose:** Test dual fusion without motion (easiest case)

```bash
ros2 launch fast_lio_ros2 test_4_dual_static.launch.py
```

**What to do:**
1. Position in corridor/room with clear geometry
2. **Keep completely stationary for 60 seconds**
3. Observe map building in RViz

**Save PCD for inspection:**
```bash
ros2 service call /save_pcd std_srvs/srv/Trigger
```

**Check saved map:**
```bash
# Map saved to:
~/ros2_ws/src/fast_lio_ros2/PCD/test_4_dual_static.pcd

# Open in CloudCompare (if installed)
cloudcompare ~/ros2_ws/src/fast_lio_ros2/PCD/test_4_dual_static.pcd
```

**Pass Criteria:**
- ✅ Wall thickness: 10-20cm (single surface)
- ✅ No ghosting or double features
- ✅ Smooth transitions in overlap regions
- ✅ Update rate: ~10 Hz
- ✅ No console errors

**If PASS:** Continue to Test 5
**If FAIL:** Go back to Test 2 (extrinsics)

---

### 🚗 Test 5: Dual LiDAR Motion Tests

**Purpose:** Validate fusion during motion

#### Test 5A: Bundle Mode
```bash
# Edit config/dual_mid360_mine.yaml
update_mode: 0  # Bundle mode
```

```bash
ros2 launch fast_lio_ros2 mapping.launch.py config_file:=dual_mid360_mine.yaml
```

**Scenarios:**
1. **Straight line:** Drive 5m forward, return to start (< 20cm error)
2. **Rotation:** 360° in place, check circular map consistency
3. **Rectangle:** 2m × 3m path, return to start (< 30cm error)

#### Test 5B: Async Mode
```bash
# Edit config/dual_mid360_mine.yaml
update_mode: 1  # Async mode
```

**Repeat scenarios above**

#### Test 5C: Adaptive Mode
```bash
# Edit config/dual_mid360_mine.yaml
update_mode: 2  # Adaptive mode
```

**Repeat scenarios above**

**Compare Performance:**

| Update Mode | Straight Error (cm) | Rotation Quality | Rectangle Error (cm) |
|-------------|---------------------|------------------|----------------------|
| Bundle (0)  | _____ | _____ | _____ |
| Async (1)   | _____ | _____ | _____ |
| Adaptive (2)| _____ | _____ | _____ |

**Choose best mode for your environment**

---

### 🏁 Test 6: Full Integration (30-min test)

**Purpose:** Long-duration validation

```bash
# Use optimal settings from Test 5
ros2 launch fast_lio_ros2 mapping.launch.py config_file:=dual_mid360_mine.yaml
```

**Monitor in separate terminals:**
```bash
# Terminal 2: Update rate
watch -n 1 'ros2 topic hz /cloud_registered'

# Terminal 3: System resources
htop
```

**Performance Targets:**

| Metric | Target | Measured | Pass/Fail |
|--------|--------|----------|-----------|
| Avg update rate | > 8 Hz | _____ | _____ |
| CPU usage | < 80% | _____ | _____ |
| Memory usage | < 6 GB | _____ | _____ |

**Final map saved automatically on shutdown**

---

## Troubleshooting Quick Reference

| Symptom | Fix |
|---------|-----|
| **Thick walls (>40cm)** | Check extrinsics, verify translation vectors |
| **Ghosting/doubles** | Check rotation matrices in config |
| **Only one LiDAR works** | Set `multi_lidar: true`, verify `lid_topic2` |
| **Low rate (<5 Hz)** | Increase `point_filter_num: 2 → 3` |
| **High CPU (>90%)** | Increase `filter_size_surf: 0.3 → 0.5` |
| **Static drift** | Increase `acc_cov: 0.5 → 1.0`, `gyr_cov: 0.5 → 1.0` |
| **Tracking fails** | Use bundle mode, check feature density |
| **Segfault** | Verify topic names match driver config |

---

## Files Reference

### Configuration Files
- **Production:** `config/dual_mid360_mine.yaml`
- **Test 3:** `config/test_3_single_lidar.yaml`
- **Test 4:** `config/test_4_dual_static.yaml`

### Launch Files
- **Test 1:** `launch/test_1_raw_lidar.launch.py`
- **Test 2:** `launch/test_2_extrinsic_validation.launch.py`
- **Test 3:** `launch/test_3_single_lidar.launch.py`
- **Test 4:** `launch/test_4_dual_static.launch.py`
- **Production:** `launch/mapping.launch.py`

### RViz Configs
- **Calibration:** `rviz/dual_lidar_calibration.rviz`
- **Mapping:** `rviz/fastlio.rviz`

### Documentation
- **Full Workflow:** `docs/TESTING_WORKFLOW_DUAL_LIDAR.md`
- **Extrinsic Guide:** `docs/EXTRINSIC_CALIBRATION_DUAL_LIDAR.md`
- **This Guide:** `docs/QUICK_TEST_GUIDE.md`

---

## Validation Checklist

Before production use:

- [ ] Test 1: Both LiDARs publishing
- [ ] Test 2: Extrinsics aligned (thin walls)
- [ ] Test 3: Single LiDAR baseline established
- [ ] Test 4: Static dual fusion working
- [ ] Test 5: Motion fusion tested (all modes)
- [ ] Test 6: 30-min integration passed
- [ ] Final map inspected and approved
- [ ] Configuration documented

---

## Quick Commands

```bash
# Check topics
ros2 topic list | grep livox
ros2 topic list | grep cloud

# Check rates
ros2 topic hz /livox/lidar_192_168_1_10
ros2 topic hz /cloud_registered

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Save current map
ros2 service call /save_pcd std_srvs/srv/Trigger

# Check diagnostics
ros2 topic echo /diagnostics

# Monitor system
htop
```

---

**Quick Test Guide v1.0** | Last Updated: 2025-11-24
