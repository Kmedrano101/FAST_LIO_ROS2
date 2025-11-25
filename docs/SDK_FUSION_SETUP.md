# SDK Fusion Setup Guide for Dual MID-360

This guide explains how to configure FAST-LIO to work with Livox SDK fusion mode for dual MID-360 LiDARs.

## Overview

**SDK Fusion Mode** means that the Livox ROS2 driver handles the fusion of point clouds from both LiDARs at the driver level, publishing a single fused point cloud. FAST-LIO then processes this pre-fused cloud as if it were from a single sensor.

### Advantages
- Lower computational load on FAST-LIO
- Simpler FAST-LIO configuration (no multi-sensor logic needed)
- Extrinsic calibration handled once at driver level
- Better for resource-constrained devices (e.g., Jetson ORIN Nano)

### Disadvantages
- No adaptive fusion strategies (Bundle/Async/Adaptive modes)
- Less flexibility in handling sensor asynchrony
- Cannot tune fusion parameters within FAST-LIO

---

## Configuration Steps

### 1. Configure Livox ROS2 Driver for SDK Fusion

**File:** `livox_ros_driver2/config/livox_params.yaml`

```yaml
livox_ros_driver2_node:
  ros__parameters:
    xfer_format: 0
    multi_topic: 0  # ← CRITICAL: Set to 0 for SDK fusion
    data_src: 0
    publish_freq: 10
    output_data_type: 0
    frame_id: "livox_frame"
    user_config_path: "<package_share>/config/multiple_netconfigs.json"
```

**Key Setting:** `multi_topic: 0` enables SDK fusion mode.

---

### 2. Verify Extrinsic Calibration in Livox Config

**File:** `livox_ros_driver2/config/multiple_netconfigs.json`

The extrinsic parameters define how each LiDAR is positioned and oriented relative to `base_link`:

```json
"lidar_configs": [
  {
    "ip": "192.168.1.10",
    "extrinsic_parameter": {
      "roll": 90.0,
      "pitch": 0.0,
      "yaw": 180.0,
      "x": 0,
      "y": 110,     // 110mm lateral offset (left sensor)
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
      "y": -110,    // 110mm lateral offset (right sensor)
      "z": 0
    }
  }
]
```

**These values are critical!** The Livox SDK uses them to transform both point clouds to `base_link` before fusing them.

**Current Configuration:**
- **L1 (192.168.1.10):** Left sensor, 110mm offset, facing backward (Yaw=180°)
- **L2 (192.168.1.18):** Right sensor, 110mm offset, facing forward (Yaw=0°)
- Both sensors rotated 90° roll to account for mounting orientation

---

### 3. Use FAST-LIO SDK Fusion Configuration

**File:** `fast_lio_ros2/config/dual_mid360_sdk_fusion.yaml`

This configuration has been created specifically for SDK fusion mode. Key differences from multi-sensor mode:

```yaml
/**:
  ros__parameters:
    common:
      lid_topic: "/livox/lidar"   # Single fused topic (not separate topics)
      imu_topic: "/livox/imu"     # IMU from primary sensor

    mapping:
      # Extrinsics from base_link (fused cloud frame) to IMU frame
      extrinsic_T: [-0.011, 0.02329, -0.15412]
      extrinsic_R: [-1.0, 0.0, 0.0,
                     0.0, -1.0, 0.0,
                     0.0, 0.0, 1.0]
```

---

## Extrinsic Transformation Explained

### Understanding the Coordinate Frames

1. **base_link:** Reference frame for the fused point cloud (SDK applies extrinsics here)
2. **L1 optical center:** LiDAR 1's scanning origin
3. **L1 IMU:** IMU inside LiDAR 1 (primary reference for FAST-LIO)

### Transformation Chain

```
base_link → L1 optical center → L1 IMU
```

**From `multiple_netconfigs.json`:**
- L1 position: [0, 110, 0] mm = [0, 0.11, 0] m
- L1 rotation: Roll=90°, Yaw=180°

**MID-360 Internal Offset (LiDAR optical → IMU):**
- From MID-360 datasheet: [-0.011, -0.02329, 0.04412] m

**Final Transformation (base_link → IMU):**

1. Transform base_link → L1 optical center:
   - Translation: [0, 0.11, 0]
   - Rotation: R_L1 (from Roll=90°, Yaw=180°)

2. Transform L1 optical → L1 IMU:
   - Apply internal offset in L1's rotated frame
   - L1_IMU_pos = [0, 0.11, 0] + R_L1 × [-0.011, -0.02329, 0.04412]
   - Result: [0.011, 0.15412, -0.02329]

3. Inverse transformation (for FAST-LIO config):
   - `extrinsic_T = [-0.011, 0.02329, -0.15412]`
   - `extrinsic_R = R_L1^T` (inverse rotation)

---

## Launch Sequence

### Option A: Manual Sequential Launch

1. **Start Livox Driver:**
   ```bash
   ros2 launch livox_ros_driver2 multiple_lidars.launch.py
   ```

2. **Verify Topics:**
   ```bash
   ros2 topic list
   # Should see:
   # /livox/lidar  (fused point cloud)
   # /livox/imu    (IMU data)
   ```

3. **Check Topic Data:**
   ```bash
   ros2 topic hz /livox/lidar
   ros2 topic hz /livox/imu

   # Visualize in RViz2
   rviz2
   # Add PointCloud2 display, topic: /livox/lidar, frame: livox_frame
   ```

4. **Start FAST-LIO:**
   ```bash
   ros2 launch fast_lio_ros2 sdk_fusion_mapping.launch.py
   ```

### Option B: Combined Launch (Recommended)

Create a combined launch file to start both nodes:

```bash
# TODO: Create a combined launch file for convenience
ros2 launch fast_lio_ros2 full_sdk_fusion_system.launch.py
```

---

## Verification and Debugging

### Check Point Cloud Fusion

```bash
# Monitor point cloud rate
ros2 topic hz /livox/lidar

# Expected: ~10 Hz (matching scan_rate)

# Check point count
ros2 topic echo /livox/lidar --field height,width
# Should see combined points from both sensors
```

### Check IMU Data

```bash
ros2 topic hz /livox/imu
# Expected: Higher rate than LiDAR (typically 100-200 Hz)

ros2 topic echo /livox/imu
# Verify accelerometer and gyroscope data
```

### Verify FAST-LIO Output

```bash
# Check odometry
ros2 topic hz /Odometry

# Check map
ros2 topic hz /cloud_registered

# Monitor FAST-LIO logs
# Look for successful initialization and stable tracking
```

### Visualize in RViz2

```bash
rviz2
```

**Add these displays:**
1. **PointCloud2** → `/livox/lidar` (raw fused input)
2. **PointCloud2** → `/cloud_registered` (FAST-LIO map output)
3. **Odometry** → `/Odometry` (trajectory)
4. **Path** → `/path` (historical path)
5. **TF** → Enable to see frame transformations

**Fixed Frame:** Set to `camera_init` (FAST-LIO's global frame)

---

## Troubleshooting

### No Point Cloud Data

**Problem:** `/livox/lidar` topic not publishing

**Solutions:**
1. Check `multi_topic` is set to 0 in `livox_params.yaml`
2. Verify both LiDARs are connected and powered
3. Check network connectivity:
   ```bash
   ping 192.168.1.10
   ping 192.168.1.18
   ```
4. Review driver logs for connection errors

### Point Cloud Not Aligned

**Problem:** Fused cloud shows misalignment or double walls

**Solutions:**
1. Verify extrinsic parameters in `multiple_netconfigs.json`
2. Check sensor mounting matches configured angles
3. Recalibrate extrinsics using calibration tools
4. Ensure sensors are rigidly mounted (no vibration-induced shifts)

### FAST-LIO Odometry Drift

**Problem:** FAST-LIO tracking loses accuracy over time

**Solutions:**
1. Check IMU noise parameters in `dual_mid360_sdk_fusion.yaml`:
   - Increase `acc_cov` and `gyr_cov` if high vibration
2. Verify time synchronization between LiDAR and IMU
3. Increase `max_iteration` for better convergence (at cost of speed)
4. Check for sensor calibration issues

### Performance Issues on Jetson

**Problem:** High CPU usage or dropped frames

**Solutions:**
1. Increase downsampling: `point_filter_num: 2 → 3`
2. Increase voxel sizes:
   - `filter_size_surf: 0.3 → 0.5`
   - `filter_size_map: 0.4 → 0.5`
3. Reduce map size: `cube_side_length: 200 → 100`
4. Disable unnecessary publishing: `dense_publish_en: false`
5. Monitor with:
   ```bash
   top  # Check CPU usage
   ros2 topic hz /livox/lidar  # Check input rate
   ros2 topic hz /Odometry     # Check output rate
   ```

---

## Switching Between SDK Fusion and FAST-LIO Multi-Sensor Fusion

### To Switch to SDK Fusion (Current Guide):
1. Set `multi_topic: 0` in `livox_params.yaml`
2. Use `dual_mid360_sdk_fusion.yaml` config
3. Launch with `sdk_fusion_mapping.launch.py`

### To Switch to FAST-LIO Multi-Sensor Fusion:
1. Set `multi_topic: 1` in `livox_params.yaml`
2. Use `dual_mid360_mine.yaml` config (from feature/multi-sensors branch)
3. Launch with dual-sensor launch file
4. Benefit: Adaptive fusion modes (Bundle/Async/Adaptive)

**Recommendation:**
- Use **SDK fusion** for Jetson or when computational resources are limited
- Use **FAST-LIO multi-sensor fusion** when you need advanced fusion strategies and have more computational power

---

## Files Summary

### Created/Modified Files for SDK Fusion:

| File | Purpose |
|------|---------|
| `config/dual_mid360_sdk_fusion.yaml` | FAST-LIO configuration for SDK fusion mode |
| `launch/sdk_fusion_mapping.launch.py` | Launch file for SDK fusion mode |
| `SDK_FUSION_SETUP.md` | This documentation file |

### Required Livox Driver Files:

| File | Setting | Purpose |
|------|---------|---------|
| `livox_params.yaml` | `multi_topic: 0` | Enable SDK fusion |
| `multiple_netconfigs.json` | Extrinsic parameters | Define sensor positions/orientations |

---

## Additional Resources

- [FAST-LIO GitHub](https://github.com/hku-mars/FAST_LIO)
- [FAST_LIO_MULTI GitHub](https://github.com/APRIL-ZJU/FAST_LIO_MULTI)
- [Livox SDK2 Documentation](https://github.com/Livox-SDK/Livox-SDK2)
- [MID-360 User Manual](https://www.livoxtech.com/mid-360)

---

## Contact

For issues specific to this configuration:
- Branch: `jetson-dev`
- Repository: fast_lio_ros2 (your fork)

For general FAST-LIO or Livox driver issues, refer to the official repositories.