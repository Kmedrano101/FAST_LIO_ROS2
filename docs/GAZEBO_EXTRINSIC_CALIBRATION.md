# Gazebo Extrinsic Calibration Guide

**How to Extract IMU-LiDAR Extrinsic Parameters from Gazebo SDF Models**

This guide explains how to find and calculate the correct `extrinsic_T` and `extrinsic_R` values for FAST-LIO from your Gazebo simulation model.

---

## Table of Contents

1. [What are Extrinsic Parameters?](#what-are-extrinsic-parameters)
2. [Finding Your Gazebo SDF Model](#finding-your-gazebo-sdf-model)
3. [Reading Sensor Poses from SDF](#reading-sensor-poses-from-sdf)
4. [Calculating Extrinsic Transform](#calculating-extrinsic-transform)
5. [Example: x500_lidars Model](#example-x500_lidars-model)
6. [Updating FAST-LIO Configuration](#updating-fast-lio-configuration)
7. [Verification](#verification)

---

## What are Extrinsic Parameters?

**Extrinsic calibration** defines the spatial relationship between two sensors:

- **`extrinsic_T`**: Translation vector [x, y, z] from **LiDAR frame** to **IMU frame** (in meters)
- **`extrinsic_R`**: 3x3 rotation matrix representing orientation difference between sensors

In FAST-LIO:
- The IMU is the **reference frame** (body frame)
- LiDAR measurements are transformed to IMU frame using extrinsics
- Accurate extrinsics are **critical** for drift-free odometry

---

## Finding Your Gazebo SDF Model

### Step 1: Identify Your Gazebo Model Name

When you launch your simulation, look for the model name in the terminal output or Gazebo GUI.

```bash
# Example: If using PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lidars  # Model name: x500_lidars
```

### Step 2: Locate the SDF File

Gazebo models are stored in:
```
PX4-Autopilot/Tools/simulation/gz/models/<model_name>/model.sdf
```

**For x500_lidars example:**
```bash
/home/kmedrano/PX4-Autopilot/Tools/simulation/gz/models/x500_lidars/model.sdf
```

### Step 3: Check for Base Model Includes

Open the model SDF and look for `<include>` tags:

```xml
<model name='x500_lidars'>
  <include merge='true'>
    <uri>x500</uri>  <!-- Base model -->
  </include>
  <!-- Additional sensors defined here -->
</model>
```

You may need to check the base model too:
```bash
/home/kmedrano/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf
```

---

## Reading Sensor Poses from SDF

### Understanding SDF Pose Format

```xml
<pose>x y z roll pitch yaw</pose>
```

- **Position**: `[x, y, z]` in meters
- **Orientation**: `[roll, pitch, yaw]` in radians (Euler angles)
- Pose is relative to the **parent link** or **model origin**

### Step 1: Find the IMU Sensor

Search for `<sensor name="imu_sensor"` or similar:

```xml
<!-- Example from x500_lidars/model.sdf -->
<link name="lidar_link_1">
  <pose>0 0 .29 0 0 0</pose>  <!-- Link position relative to base_link -->

  <sensor name="imu_sensor_1" type="imu">
    <always_on>1</always_on>
    <update_rate>200</update_rate>
    <topic>gazebo_imu1/imu</topic>
    <!-- No explicit pose = at link origin -->
  </sensor>
</link>
```

**IMU Position:**
- If no `<pose>` tag inside sensor: IMU is at the **link origin**
- Link origin: `[0, 0, 0.29]` relative to base_link

### Step 2: Find the LiDAR Sensor

Search for `<sensor name="lidar_sensor"` or `type="gpu_lidar"`:

```xml
<sensor name="lidar_sensor_1" type="gpu_lidar">
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <pose>0 0 .3 0 0 0</pose>  <!-- LiDAR pose within lidar_link_1 -->
  <topic>gazebo_lidar1</topic>
  <!-- ... -->
</sensor>
```

**LiDAR Position:**
- Within link: `[0, 0, 0.3]`
- Relative to base_link: `[0, 0, 0.29]` (link) + `[0, 0, 0.3]` (sensor) = `[0, 0, 0.59]`

### Step 3: Check Topic Remapping

Verify which Gazebo topics are bridged to ROS2:

**File**: `px4_offboard_sim/config/gz_bridge.yaml`

```yaml
- gz_topic_name: "gazebo_imu1/imu"
  ros_topic_name: "sim_imu/imu"      # ← FAST-LIO subscribes here

- gz_topic_name: "gazebo_lidar1/points"
  ros_topic_name: "sim_lidar/lidar"  # ← FAST-LIO subscribes here
```

Make sure you're analyzing the sensors that match your ROS2 topics!

---

## Calculating Extrinsic Transform

### Coordinate Frame Diagram

```
          Z
          ↑
          |
    Y ←---●  (base_link origin)
         /
        X

base_link (0, 0, 0)
    │
    ├─> lidar_link_1 (0, 0, 0.29)
           │
           ├─> IMU sensor (0, 0, 0) [in lidar_link_1 frame]
           │     = (0, 0, 0.29) [in base_link frame]
           │
           └─> LiDAR sensor (0, 0, 0.3) [in lidar_link_1 frame]
                 = (0, 0, 0.59) [in base_link frame]
```

### Calculation Steps

**1. Find sensor positions in a common reference frame (base_link or world):**

- IMU position: `P_imu = [0, 0, 0.29]`
- LiDAR position: `P_lidar = [0, 0, 0.59]`

**2. Calculate translation from LiDAR to IMU:**

```
extrinsic_T = P_imu - P_lidar
            = [0, 0, 0.29] - [0, 0, 0.59]
            = [0, 0, -0.3]
```

**Interpretation**: IMU is **0.3 meters below** the LiDAR sensor center.

**3. Calculate rotation matrix:**

If both sensors have the same orientation (same roll, pitch, yaw):
```
extrinsic_R = Identity matrix = [1, 0, 0]
                                [0, 1, 0]
                                [0, 0, 1]
```

If sensors have different orientations, use rotation matrix conversion:
```python
import numpy as np
from scipy.spatial.transform import Rotation

# Example: LiDAR rotated 180° around Z-axis
roll, pitch, yaw = 0, 0, np.pi  # radians
R = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
```

---

## Example: x500_lidars Model

### Step-by-Step Walkthrough

**File**: `/home/kmedrano/PX4-Autopilot/Tools/simulation/gz/models/x500_lidars/model.sdf`

#### 1. Find the Sensor Link

```xml
<link name="lidar_link_1">
  <pose>.0 .0 .29 0 0 0</pose>
```
- Link position: `(0, 0, 0.29)` meters above base_link
- Link orientation: `(0, 0, 0)` radians (no rotation)

#### 2. Find IMU Sensor

```xml
<sensor name="imu_sensor_1" type="imu">
  <always_on>1</always_on>
  <update_rate>200</update_rate>
  <topic>gazebo_imu1/imu</topic>
</sensor>
```
- No `<pose>` tag → at link origin
- **IMU position in lidar_link_1**: `(0, 0, 0)`
- **IMU position in base_link**: `(0, 0, 0.29)`

#### 3. Find LiDAR Sensor

```xml
<sensor name="lidar_sensor_1" type="gpu_lidar">
  <pose>0 0 .3 0 0 0</pose>
  <topic>gazebo_lidar1</topic>
```
- **LiDAR position in lidar_link_1**: `(0, 0, 0.3)`
- **LiDAR position in base_link**: `(0, 0, 0.29 + 0.3) = (0, 0, 0.59)`

#### 4. Calculate Extrinsics

```
Translation (LiDAR → IMU):
  T = IMU_pos - LiDAR_pos
    = [0, 0, 0.29] - [0, 0, 0.59]
    = [0, 0, -0.3]

Rotation:
  Both aligned with lidar_link_1
  → Identity matrix
```

#### 5. Result

```yaml
extrinsic_T: [0.0, 0.0, -0.3]
extrinsic_R: [ 1., 0., 0.,
               0., 1., 0.,
               0., 0., 1.]
```

---

## Updating FAST-LIO Configuration

### File: `fast_lio_ros2/config/gazebosim.yaml`

```yaml
/**:
    ros__parameters:
        common:
            lid_topic:  "/sim_lidar/lidar"   # Match gz_bridge.yaml
            imu_topic:  "/sim_imu/imu"       # Match gz_bridge.yaml

        mapping:
          extrinsic_est_en: true             # Enable online refinement
          extrinsic_T: [0.0, 0.0, -0.3]      # From calculation above
          extrinsic_R: [ 1., 0., 0.,
                         0., 1., 0.,
                         0., 0., 1.]         # Identity
```

### Important Notes

- **`extrinsic_est_en: true`**: Allows FAST-LIO to refine extrinsics during initialization
- Initial values should be **close to truth** (within ~0.1m, ~10°)
- Bad initial values → poor convergence or drift

---

## Verification

### 1. Check Topic Connections

```bash
# Start your simulation
ros2 launch px4_offboard_sim slam_simulation.launch.py

# Verify topics exist
ros2 topic list | grep -E "(sim_imu|sim_lidar)"
# Should show:
#   /sim_imu/imu
#   /sim_lidar/lidar

# Check message rates
ros2 topic hz /sim_imu/imu       # Should be ~200 Hz
ros2 topic hz /sim_lidar/lidar   # Should be ~10 Hz
```

### 2. Launch FAST-LIO with New Extrinsics

```bash
cd ~/ros2_ws
colcon build --packages-select fast_lio_ros2
source install/setup.bash
ros2 launch fast_lio_ros2 simulation_mapping.launch.py config_file:=gazebosim.yaml
```

### 3. Check for Convergence

Watch FAST-LIO terminal output for:
```
[ INFO] Extrinsic calibration:
        Translation: [0.000, 0.000, -0.300]
        Rotation matrix:
            1.000  0.000  0.000
            0.000  1.000  0.000
            0.000  0.000  1.000
```

### 4. Verify No Drift

Compare FAST-LIO odometry with ground truth:

```bash
# Terminal 1: FAST-LIO position
ros2 topic echo /Odometry --field pose.pose.position

# Terminal 2: Ground truth (if available)
ros2 topic echo /ground_truth/pose --field pose.position
```

**Expected**: Position should closely track ground truth with minimal drift.

---

## Common Issues and Solutions

### Issue 1: Large Position Drift

**Symptom**: Robot position drifts far from ground truth over time

**Causes:**
- Incorrect `extrinsic_T` (wrong translation)
- Sensors not time-synchronized

**Solution:**
1. Re-check SDF pose calculations
2. Verify timestamps: `ros2 topic echo /sim_imu/imu --field header.stamp`
3. Enable `use_sim_time: true` in all nodes

### Issue 2: Map Rotation/Flip

**Symptom**: Map appears rotated or mirrored

**Causes:**
- Incorrect `extrinsic_R` (wrong rotation)
- Frame convention mismatch (FRD vs FLU)

**Solution:**
1. Verify sensor orientations in SDF
2. Check if rotation is needed for frame alignment
3. Test with identity matrix first

### Issue 3: Oscillating Odometry

**Symptom**: Position/orientation oscillates or jumps

**Causes:**
- Bad initial extrinsics (too far from truth)
- `extrinsic_est_en: true` with poor initial guess

**Solution:**
1. Set `extrinsic_est_en: false` temporarily
2. Use static TF publisher to verify frame alignment
3. Get accurate measurement of physical sensor positions

### Issue 4: X-Axis Points Backward in RViz

**Symptom**: Red X-axis arrow points opposite to drone's front

**Cause:**
- IMU frame convention (NED/FRD) vs ROS convention (ENU/FLU)
- Cosmetic visualization issue only

**Solution:**
- Option A: Keep as-is (doesn't affect accuracy)
- Option B: Add 180° Z-rotation to `extrinsic_R`
- Option C: Modify Gazebo SDF sensor mounting

---

## Reference: Coordinate Systems

### Gazebo/PX4 Convention (Typical)
- **X**: Forward
- **Y**: Right
- **Z**: Down
- **Body frame**: FRD (Forward-Right-Down)

### ROS2/RViz Convention (REP 103/147)
- **X**: Forward or East
- **Y**: Left or North
- **Z**: Up
- **Body frame**: FLU (Forward-Left-Up) or ENU (East-North-Up)

**Note**: FAST-LIO is frame-agnostic - as long as extrinsics are correct and all sensors use the same convention, it will work.

---

## Advanced: Multi-Sensor Setups

If your robot has **multiple LiDARs and IMUs**:

### 1. Choose Primary IMU

Usually the IMU closest to robot's center of mass or control system.

### 2. Calculate Extrinsics for Each LiDAR

```yaml
# If using lidar_sensor_1 with imu_sensor_1:
extrinsic_T: [0.0, 0.0, -0.3]

# If using lidar_sensor_2 (different mounting):
extrinsic_T: [0.1, 0.0, -0.15]  # Recalculate from SDF
```

### 3. Configure Multi-LiDAR (if supported)

FAST-LIO can fuse multiple LiDARs - check documentation for multi-LiDAR configuration.

---

## Tools and Utilities

### 1. SDF Parser (Python)

```python
import xml.etree.ElementTree as ET

def parse_sensor_pose(sdf_file, sensor_name):
    tree = ET.parse(sdf_file)
    root = tree.getroot()

    # Find sensor
    for sensor in root.iter('sensor'):
        if sensor.get('name') == sensor_name:
            pose = sensor.find('pose')
            if pose is not None:
                values = [float(x) for x in pose.text.split()]
                return {
                    'position': values[0:3],
                    'orientation': values[3:6]
                }
    return None

# Usage
pose = parse_sensor_pose('model.sdf', 'lidar_sensor_1')
print(f"Position: {pose['position']}")
```

### 2. TF Tree Visualization

```bash
# Install if not already
sudo apt install ros-humble-tf2-tools

# View TF tree
ros2 run tf2_tools view_frames

# Opens frames_<timestamp>.pdf with transform tree
```

### 3. Static TF Publisher (for testing)

```bash
# Publish test transform
ros2 run tf2_ros static_transform_publisher \
  0 0 -0.3 0 0 0 1 \
  imu_frame lidar_frame
```

---

## Checklist

Before running FAST-LIO with new extrinsics:

- [ ] Located correct Gazebo SDF model file
- [ ] Identified IMU sensor and its position/orientation
- [ ] Identified LiDAR sensor and its position/orientation
- [ ] Calculated translation vector (LiDAR → IMU)
- [ ] Calculated rotation matrix (if sensors not aligned)
- [ ] Updated `gazebosim.yaml` with new extrinsics
- [ ] Verified topic names match ROS2 bridge configuration
- [ ] Rebuilt and sourced workspace
- [ ] Tested with simulation running
- [ ] Checked for drift and map quality

---

## Further Reading

- [FAST-LIO Paper](https://github.com/hku-mars/FAST_LIO)
- [ROS REP 103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [ROS REP 147: Aerial Vehicles](https://www.ros.org/reps/rep-0147.html)
- [Gazebo SDF Specification](http://sdformat.org/spec)
- [PX4 Coordinate Systems](https://docs.px4.io/main/en/ros/external_position_estimation.html)

---

**Document Version**: 1.0
**Last Updated**: 2025-01-28
**Tested With**:
- PX4-Autopilot: v1.14+
- Gazebo Harmonic (gz-sim8)
- fast_lio_ros2: ROS2 Humble
- Model: x500_lidars

---

**Questions or Issues?** Check the troubleshooting section or open an issue in the repository.
