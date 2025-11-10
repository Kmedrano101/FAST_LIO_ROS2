# PX4 Gazebo + FAST-LIO Integration Guide

Complete guide for running FAST-LIO SLAM with PX4 Gazebo simulation using the lidar_imu_sync_node.

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    PX4 Gazebo Simulation                         │
│  ┌────────────┐              ┌────────────┐                     │
│  │   LiDAR    │              │    IMU     │                     │
│  │ 128 rings  │              │  100+ Hz   │                     │
│  └─────┬──────┘              └──────┬─────┘                     │
└────────┼────────────────────────────┼───────────────────────────┘
         │                            │
         │ /sim_lidar/lidar           │ /sim_imu/imu
         │ (PointCloud2)              │ (Imu)
         │                            │
┌────────▼────────────────────────────▼───────────────────────────┐
│              lidar_imu_sync_node                                 │
│  (px4_offboard_sim package)                                      │
│                                                                   │
│  • Adds 'time' field to each point                              │
│  • Synchronizes LiDAR & IMU                                     │
│  • Sets IMU covariances                                         │
│  • Transforms to Velodyne format                                │
└────────┬────────────────────────────┬───────────────────────────┘
         │                            │
         │ /velodyne_points           │ /imu/data
         │ (Velodyne format)          │ (Imu)
         │                            │
┌────────▼────────────────────────────▼───────────────────────────┐
│                      FAST-LIO                                    │
│  (fast_lio_ros2 package)                                         │
│                                                                   │
│  Config: velodyne.yaml                                           │
│  • lidar_type: 2 (Velodyne)                                     │
│  • scan_line: 128                                               │
│  • Topics: /velodyne_points, /imu/data                          │
└────────┬────────────────────────────────────────────────────────┘
         │
         │ /Odometry, /map_cloud, /path
         │
         ▼
    SLAM Output

```

---

## Prerequisites

- ROS 2 Humble
- PX4-Autopilot v1.14
- Gazebo Garden (gz sim 7.x)
- ros_gz_bridge for Gazebo Garden
- fast_lio_ros2
- px4_offboard_sim

---

## Installation

### 1. Install ros_gz for Gazebo Garden

```bash
# Option 1: Binary packages (recommended)
sudo apt update
sudo apt install ros-humble-ros-gzgarden

# Option 2: Build from source
cd ~/ros2_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b garden
cd ~/ros2_ws
rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build --packages-select ros_gz_bridge ros_gz_sim ros_gz_image
```

### 2. Build workspace

```bash
cd ~/ros2_ws
colcon build --packages-select px4_offboard_sim fast_lio_ros2
source install/setup.bash
```

---

## Configuration Files

### 1. Sync Node Config
**File**: `~/ros2_ws/src/px4_offboard_sim/config/lidar_imu_sync.yaml`

```yaml
/**:
  ros__parameters:
    # Input topics from Gazebo
    lidar_topic_in: "/sim_lidar/lidar"
    imu_topic_in: "/sim_imu/imu"

    # Output topics (Velodyne format)
    lidar_topic_out: "/velodyne_points"
    imu_topic_out: "/imu/data"

    # Parameters
    lidar_frame_id: "lidar_link"
    imu_frame_id: "imu_link"
    time_offset: 0.0      # LiDAR to IMU time offset
    scan_rate: 10         # LiDAR scan rate (Hz)
    use_sync: true        # Enable message_filters sync
```

### 2. FAST-LIO Config (Updated for Gazebo)
**File**: `~/ros2_ws/src/fast_lio_ros2/config/velodyne.yaml`

Key parameters for Gazebo:
```yaml
preprocess:
  lidar_type: 2        # Velodyne (converted by sync node)
  scan_line: 128       # ← UPDATED: Gazebo 128-ring LiDAR
  scan_rate: 10        # ← MUST match sync node
  blind: 0.01          # ← UPDATED: Indoor simulation

mapping:
  det_range: 60.0      # ← UPDATED: Gazebo environment
  extrinsic_T: [0.0, 0.0, -0.3]  # ← UPDATED: Gazebo mounting
  extrinsic_est_en: true

publish:
  map_frame_id: "world"   # ← UPDATED: PX4 frame
  body_frame_id: "drone"  # ← UPDATED: PX4 frame
  path_en: true
```

---

## Usage

### Method 1: Complete Launch (Recommended)

```bash
# Terminal 1: Start PX4 Gazebo simulation
ros2 launch px4_offboard_sim slam_simulation.launch.py

# Terminal 2: Launch sync node
ros2 launch px4_offboard_sim lidar_imu_sync.launch.py

# Terminal 3: Launch FAST-LIO (uses velodyne.yaml by default)
ros2 launch fast_lio_ros2 simulation_mapping.launch.py
```

### Method 2: Step-by-Step

```bash
# Terminal 1: Gazebo with PX4
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lidars

# Terminal 2: MicroXRCE Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Sync node
ros2 run px4_offboard_sim lidar_imu_sync_node \
    --ros-args --params-file ~/ros2_ws/src/px4_offboard_sim/config/lidar_imu_sync.yaml

# Terminal 4: FAST-LIO
ros2 launch fast_lio_ros2 simulation_mapping.launch.py
```

### Method 3: Using Different Config

```bash
# Use gazebosim.yaml instead (direct connection, no sync node)
ros2 launch fast_lio_ros2 simulation_mapping.launch.py \
    config_file:=gazebosim.yaml
```

---

## Launch File Arguments

### px4_offboard_sim/lidar_imu_sync.launch.py

```bash
ros2 launch px4_offboard_sim lidar_imu_sync.launch.py \
    config_file:=/path/to/custom_config.yaml \
    use_sync:=false
```

### fast_lio_ros2/simulation_mapping.launch.py

```bash
ros2 launch fast_lio_ros2 simulation_mapping.launch.py \
    config_file:=velodyne.yaml \      # Config file (default: velodyne.yaml)
    use_sim_time:=true \              # Use Gazebo clock (default: true)
    rviz:=true                        # Launch RViz (default: true)
```

---

## Verification

### 1. Check Topics

```bash
# Should see:
# /sim_lidar/lidar (input)
# /sim_imu/imu (input)
# /velodyne_points (output)
# /imu/data (output)
ros2 topic list | grep -E "(lidar|imu|velodyne)"
```

### 2. Check Topic Rates

```bash
ros2 topic hz /velodyne_points    # Should be ~5-10 Hz
ros2 topic hz /imu/data           # Should be 100+ Hz
```

### 3. Check Message Structure

```bash
# Verify 'time' field is present
ros2 topic echo /velodyne_points --once | grep -A 10 "fields:"

# Should show:
# - x, y, z, intensity, time, ring
```

### 4. Check FAST-LIO Output

```bash
ros2 topic list | grep -E "(Odometry|cloud|path)"

# Should see:
# /Odometry
# /cloud_registered
# /cloud_registered_body
# /path
```

---

## Monitoring & Debugging

### View in RViz

FAST-LIO launches RViz by default. Add these displays:

1. **PointCloud2**: `/cloud_registered` (map)
2. **Path**: `/path` (trajectory)
3. **Odometry**: `/Odometry`
4. **PointCloud2**: `/velodyne_points` (raw scans)
5. **TF**: Show transforms

**Fixed Frame**: `world` or `camera_init`

### Check TF Tree

```bash
ros2 run tf2_tools view_frames
# Open frames_*.pdf to verify transforms
```

### Monitor Performance

```bash
# CPU usage
top -p $(pgrep -f fastlio_mapping)
top -p $(pgrep -f lidar_imu_sync_node)

# Memory
ros2 topic bw /velodyne_points
ros2 topic delay /velodyne_points
```

### Enable Debug Logging

```bash
ros2 run px4_offboard_sim lidar_imu_sync_node \
    --ros-args --log-level debug
```

---

## Troubleshooting

### Issue: "No topics published"
**Check**:
1. Gazebo simulation is running
2. Sensors are enabled in model
3. ros_gz_bridge is installed

```bash
# Verify Gazebo topics
gz topic -l | grep lidar
```

### Issue: FAST-LIO not receiving data
**Check**:
1. Sync node is running
2. Topic names match
3. Config file loaded correctly

```bash
ros2 node info /lidar_imu_sync_node
ros2 param list /laserMapping
```

### Issue: Poor mapping quality
**Solutions**:
1. Ensure sync node is running (timestamps critical)
2. Check IMU rate is sufficient (>100 Hz)
3. Verify extrinsic calibration
4. Adjust filter sizes in config

### Issue: "time field missing" in FAST-LIO
**Solution**: Ensure sync node is running. The time field is added by lidar_imu_sync_node.

```bash
# Verify sync node is active
ros2 node list | grep lidar_imu_sync
```

### Issue: Frame ID mismatch
**Check** frame IDs in configs:
- Sync node: `lidar_frame_id`, `imu_frame_id`
- FAST-LIO: `map_frame_id`, `body_frame_id`
- TF tree matches

---

## Performance Tuning

### For Dense Point Clouds (128 rings)

```yaml
# velodyne.yaml
filter_size_surf: 0.09   # Smaller = preserve detail
filter_size_map: 0.09
point_filter_num: 3      # Skip fewer points
```

### For Real-Time Performance

```yaml
# velodyne.yaml
filter_size_surf: 0.15   # Larger = faster
filter_size_map: 0.15
dense_publish_en: false  # Reduce output
point_filter_num: 4      # Skip more points
```

### For Large Environments

```yaml
# velodyne.yaml
cube_side_length: 2000.0  # Larger map
det_range: 100.0          # Longer range
```

---

## File Reference

### Config Files
- Sync node: `px4_offboard_sim/config/lidar_imu_sync.yaml`
- FAST-LIO: `fast_lio_ros2/config/velodyne.yaml`
- Alternative: `fast_lio_ros2/config/gazebosim.yaml` (direct, no sync)

### Launch Files
- Sync node: `px4_offboard_sim/launch/lidar_imu_sync.launch.py`
- FAST-LIO: `fast_lio_ros2/launch/simulation_mapping.launch.py`
- Full sim: `px4_offboard_sim/launch/slam_simulation.launch.py`

### Source Code
- Sync node: `px4_offboard_sim/src/lidar_imu_sync_node.cpp`
- FAST-LIO: External package

### Documentation
- Sync node: `px4_offboard_sim/README_LIDAR_IMU_SYNC.md`
- Test results: `px4_offboard_sim/TEST_RESULTS.md`
- This guide: `ros2_ws/GAZEBO_FASTLIO_INTEGRATION_GUIDE.md`

---

## Key Changes Summary

### ✅ What Was Updated

1. **FAST-LIO velodyne.yaml**:
   - `scan_line: 32` → `128`
   - `blind: 2.0` → `0.01`
   - `filter_size: 0.5` → `0.09`
   - Added PX4 frame IDs

2. **FAST-LIO simulation_mapping.launch.py**:
   - `default_value='mobile_robot.yaml'` → `'velodyne.yaml'`

3. **Created lidar_imu_sync_node**:
   - Adds time field to PointCloud2
   - Synchronizes LiDAR + IMU
   - Transforms to Velodyne format

### 🎯 Why These Changes

- **128 scan lines**: Gazebo uses 128-ring LiDAR, not 32
- **Smaller filter sizes**: Dense point clouds need finer filtering
- **Velodyne format**: Better FAST-LIO compatibility than custom Gazebo type
- **Sync node**: Ensures temporal alignment and proper timestamps

---

## Quick Reference

### Start Everything (One-liner per terminal)
```bash
# T1: Simulation
ros2 launch px4_offboard_sim slam_simulation.launch.py

# T2: Sync
ros2 launch px4_offboard_sim lidar_imu_sync.launch.py

# T3: FAST-LIO
ros2 launch fast_lio_ros2 simulation_mapping.launch.py
```

### Check Status
```bash
ros2 node list                    # See running nodes
ros2 topic hz /velodyne_points    # Check LiDAR rate
ros2 topic hz /Odometry           # Check SLAM rate
```

### Stop Everything
```bash
# Ctrl+C in each terminal, or:
killall -9 gz ruby MicroXRCEAgent fastlio_mapping lidar_imu_sync_node
```

---

## Advanced Usage

### Multiple LiDARs

The sync node can handle multiple LiDARs. Update config:

```yaml
lidar_topic_in: "/sim_lidar2/lidar"  # Use second LiDAR
lidar_topic_out: "/velodyne_points2"
```

### Time Offset Calibration

If sensors are misaligned in time:

1. Record rosbag
2. Use Kalibr or similar tool
3. Update `time_offset` parameter
4. Restart sync node

### Custom Extrinsics

Update FAST-LIO config with your calibrated values:

```yaml
mapping:
  extrinsic_T: [dx, dy, dz]     # LiDAR to IMU translation
  extrinsic_R: [r11, r12, r13,  # LiDAR to IMU rotation (row-major)
                r21, r22, r23,
                r31, r32, r33]
```

---

## FAQ

**Q: Do I always need the sync node?**
A: Yes, for Velodyne format. Alternatively, use gazebosim.yaml with lidar_type: 5.

**Q: Can I use other LiDAR types?**
A: Yes, but you need to create similar sync nodes or modify configs.

**Q: Why not use gazebosim.yaml directly?**
A: You can! Sync node provides better compatibility and flexibility.

**Q: How to save the map?**
A: Enable in velodyne.yaml: `pcd_save_en: true`

**Q: FAST-LIO crashes with segfault?**
A: Check scan_line matches actual LiDAR (must be 128 for Gazebo).

---

## Next Steps

1. ✅ System configured and tested
2. 🎯 Test SLAM with drone flight
3. 🎯 Tune parameters for your environment
4. 🎯 Integrate with path planning
5. 🎯 Add obstacle detection

---

## Support

- FAST-LIO Issues: https://github.com/hku-mars/FAST_LIO/issues
- PX4 Issues: https://github.com/PX4/PX4-Autopilot/issues
- Sync Node: Check px4_offboard_sim package documentation

---

**Last Updated**: 2025-11-05
**Tested With**: ROS 2 Humble, PX4 v1.14, Gazebo Garden 7.9
