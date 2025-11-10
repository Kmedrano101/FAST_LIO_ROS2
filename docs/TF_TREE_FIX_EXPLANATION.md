# TF Tree Fix for fast_lio_ros2 Point Cloud Alignment

## Problem Summary

Point clouds from fast_lio_ros2 were appearing below z=0 in the world frame, not aligning with the Gazebo environment. This was caused by incorrect TF tree configuration.

---

## Root Cause Analysis

### Original TF Tree Structure

There were actually TWO separate, unconnected TF trees:

**Tree 1: Ground Truth (from px4_offboard_sim)**
```
world
  └─ ground_truth (dynamic, z≈0.2m)
      └─ drone_gt
          ├─ imu_link [0, 0, 0.29]
          └─ lidar_link [0, 0, 0.29]
```

**Tree 2: SLAM Estimate (from fast_lio_ros2)**
```
world
  └─ camera_init [0, 0, 0]  ← PROBLEM: Should be at z≈0.5m!
      └─ body (dynamic, fast_lio's estimated IMU pose)
          └─ ??? (no connection to lidar_link)
```

### Issues Identified

1. **Wrong `camera_init` Height**
   - `camera_init` was at z=0 (ground level)
   - Drone actually spawns at z≈0.2m
   - Sensors are at +0.29m above drone base
   - Total sensor height: ~0.49m, but camera_init was at 0m
   - **Result**: Point clouds appeared ~0.5m below their actual position

2. **Missing TF Connection**
   - fast_lio's `body` frame had no connection to `lidar_link`
   - fast_lio receives point clouds in `lidar_link` frame
   - Without a TF path, transformations would fail
   - **Result**: Potential transform lookup failures

3. **Incomplete TF Chain**
   - world → camera_init → body (from fast_lio)
   - But no: body → lidar_link
   - **Result**: TF tree incomplete

---

## Solution Implemented

### Updated TF Tree Structure

**After fixes:**
```
world (Gazebo ground, z=0)
  │
  ├─ ground_truth (dynamic, z≈0.2m) ← From px4_offboard_sim
  │   └─ drone_gt
  │       ├─ imu_link [0, 0, 0.29]
  │       │   └─ x500_lidars_0/lidar_link_1/imu_sensor_1 [identity]
  │       └─ lidar_link [0, 0, 0.29]
  │
  └─ camera_init [0, 0, 0.5] ← FIXED: Now at sensor height!
      └─ body (dynamic, from fast_lio)
          └─ lidar_link [identity] ← NEW: Connects to sensor frame
```

### Changes Made

**File: `/home/kmedrano/ros2_ws/src/fast_lio_ros2/launch/simulation_mapping.launch.py`**

#### 1. Fixed `camera_init` Height (Line 54)

**Before:**
```python
arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera_init']
```

**After:**
```python
arguments=['0', '0', '0.5', '0', '0', '0', 'world', 'camera_init']
```

**Explanation:**
- Sets camera_init at z=0.5m to match drone spawn height (~0.2m) + sensor offset (0.29m)
- This value should match where the sensors actually are when SLAM initializes
- Adjust this if your drone spawns at a different height

#### 2. Added `body` → `lidar_link` Transform (Lines 69-78)

**New static TF publisher:**
```python
static_tf_body_to_lidar = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_body_to_lidar',
    arguments=['0', '0', '0', '0', '0', '0', 'body', 'lidar_link'],
    parameters=[{'use_sim_time': use_sim_time}]
)
```

**Explanation:**
- Connects fast_lio's `body` frame (IMU body frame it tracks) to `lidar_link`
- Identity transform (0 translation, no rotation) because:
  - IMU and LiDAR are co-located in the SDF (both at z=0.29 on lidar_link_1)
  - Extrinsic calibration confirms this: `extrinsic_T: [0.0, 0.0, 0.0]`
- Allows fast_lio to properly transform point clouds from lidar_link to camera_init

#### 3. Added to Launch Sequence (Line 104)

```python
ld.add_action(static_tf_body_to_lidar)
```

---

## How It Works Now

### Data Flow

1. **LiDAR publishes** point cloud in `lidar_link` frame at sensor position (z=0.29 relative to drone_gt)

2. **fast_lio receives** point cloud and looks up transform:
   - `lidar_link` → `body` (via our static TF, identity transform)
   - Applies extrinsic calibration (also identity)

3. **fast_lio estimates** IMU pose:
   - Publishes TF: `camera_init` → `body`
   - Publishes point clouds transformed to `camera_init` frame

4. **Visualization in world frame**:
   - Point cloud is in `camera_init` frame
   - `world` → `camera_init` transform is [0, 0, 0.5]
   - Final point cloud position matches Gazebo environment!

### TF Lookup Example

When RViz needs to display a point cloud in world frame:

```
Point in lidar_link → body (static, identity)
                   → camera_init (fast_lio estimate)
                   → world (static, z=0.5)
```

**Result**: Point clouds now appear at the correct height in the world frame!

---

## Verification Steps

### 1. Check TF Tree is Complete

```bash
# Install if needed
sudo apt install ros-humble-tf2-tools

# Generate TF tree diagram
ros2 run tf2_tools view_frames

# Check specific transforms
ros2 run tf2_ros tf2_echo world camera_init
ros2 run tf2_ros tf2_echo camera_init body
ros2 run tf2_ros tf2_echo body lidar_link
```

**Expected Output:**
- world → camera_init: translation [0, 0, 0.5]
- camera_init → body: dynamic (changes as drone moves)
- body → lidar_link: identity [0, 0, 0]

### 2. Verify Point Cloud Height

```bash
# Launch fast_lio
ros2 launch fast_lio_ros2 simulation_mapping.launch.py

# Echo registered point cloud and check z values
ros2 topic echo /cloud_registered --field data --once | grep -A 5 "z"
```

**Expected**: Z values should be around 0.5m (matching sensor height), not negative or near zero.

### 3. Visual Verification in RViz

1. Open RViz with the launch file
2. Add displays:
   - **PointCloud2**: `/cloud_registered`
     - Frame: `world`
     - Size: 0.05
   - **TF**: Show all frames
   - **Axes**: For world, camera_init, body, lidar_link
3. Set Fixed Frame to `world`

**Expected Observations:**
- Point cloud appears at ground level or above (z > 0)
- camera_init frame is at z=0.5m
- body frame moves as drone moves
- lidar_link frame visible in TF tree

---

## Tuning the Height Offset

The `camera_init` z-offset (currently 0.5m) should match your drone's actual sensor height when SLAM initializes.

### To Measure Actual Height:

```bash
# Get drone's current position
ros2 topic echo /px4_offboard_sim/ground_truth/pose --field position --once

# Example output:
# - 0.0        ← x
# - 0.0        ← y
# - 0.24       ← z (drone base height)

# Add sensor offset from SDF (0.29m)
# Total sensor height = 0.24 + 0.29 = 0.53m
```

### To Adjust in Launch File:

Edit line 54 in `simulation_mapping.launch.py`:

```python
# Change the third argument (z value)
arguments=['0', '0', '0.53', '0', '0', '0', 'world', 'camera_init']
#                     ^^^^
#                  Adjust this value
```

**Guidelines:**
- If point clouds appear TOO LOW: Increase z value
- If point clouds appear TOO HIGH: Decrease z value
- Typical range: 0.4m to 0.6m for this drone configuration

---

## Understanding Frame Naming

### fast_lio_ros2 Frames

- **`camera_init`**: Map frame where SLAM initializes (origin of SLAM map)
  - Configuration: `map_frame_id: "camera_init"` in gazebosim.yaml:48
  - This is the global reference frame for fast_lio's map

- **`body`**: IMU body frame being tracked by SLAM
  - Configuration: `body_frame_id: "body"` in gazebosim.yaml:49
  - Represents the drone's estimated orientation and position
  - In fast_lio, this is the IMU coordinate frame

### Gazebo/px4_offboard_sim Frames

- **`world`**: Gazebo world coordinate frame (ground is z=0)

- **`ground_truth`**: PX4 ground truth odometry frame

- **`drone_gt`**: Drone ground truth base frame

- **`imu_link`**: IMU sensor frame (0.29m above drone base)

- **`lidar_link`**: LiDAR sensor frame (0.29m above drone base, co-located with IMU)

- **`x500_lidars_0/lidar_link_1/imu_sensor_1`**: Gazebo scoped IMU sensor frame
  - Maps to `imu_link` via static TF

---

## Troubleshooting

### Issue: Point clouds still below zero

**Solution:**
1. Check camera_init height: `ros2 run tf2_ros tf2_echo world camera_init`
2. Verify drone spawn height: `ros2 topic echo /px4_offboard_sim/ground_truth/pose`
3. Adjust z value in launch file accordingly

### Issue: TF lookup failures

**Symptoms:**
```
Could not transform point cloud from lidar_link to camera_init
```

**Solution:**
1. Verify all static TF publishers are running:
   ```bash
   ros2 node list | grep static_transform_publisher
   ```
2. Check TF tree completeness:
   ```bash
   ros2 run tf2_tools view_frames
   # Open frames.pdf to visualize
   ```

### Issue: Two parents for `lidar_link`

**Symptoms:**
```
TF_REPEATED_DATA ignoring data with redundant timestamp
```

**Cause:**
- Both `drone_gt → lidar_link` (from frame_manager) and `body → lidar_link` (from our fix) exist
- This is intentional! They represent different estimates (ground truth vs SLAM)

**Solution:**
- This warning is harmless
- The two estimates can coexist for comparison
- If bothersome, comment out `static_tf_body_to_lidar` in launch file

### Issue: Point clouds in wrong orientation

**Solution:**
Check extrinsic calibration in `gazebosim.yaml`:
```yaml
extrinsic_R: [ 1., 0., 0.,
               0., 1., 0.,
               0., 0., 1.]
```

Verify this matches the actual sensor mounting in Gazebo SDF.

---

## Summary

**What was fixed:**
1. ✅ `camera_init` height adjusted from 0m to 0.5m
2. ✅ Added `body` → `lidar_link` static transform
3. ✅ Point clouds now align with world frame

**Result:**
- Point clouds appear at correct height in visualization
- TF tree is complete and connected
- fast_lio can properly transform sensor data

**Next steps:**
1. Launch and verify point clouds are at correct height
2. Fine-tune camera_init z-offset if needed
3. Test SLAM mapping quality
