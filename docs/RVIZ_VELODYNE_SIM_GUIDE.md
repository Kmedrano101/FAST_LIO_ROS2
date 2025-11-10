# RViz Configuration Guide - Velodyne Simulation

## Overview

The `velodyne_sim.rviz` configuration is optimized for visualizing FAST-LIO SLAM with PX4 Gazebo simulation using the Velodyne-format LiDAR data.

**File**: `fast_lio_ros2/rviz/velodyne_sim.rviz`

---

## Automatic Launch

The `simulation_mapping.launch.py` now uses `velodyne_sim.rviz` by default:

```bash
ros2 launch fast_lio_ros2 simulation_mapping.launch.py
```

RViz will launch automatically with the optimized configuration.

---

## Display Panels

### 1. **TF (Transform Frames)** ✅ Enabled
- **Purpose**: Shows coordinate frame hierarchy
- **Frames Displayed**:
  - `world` - Global reference frame (PX4)
  - `drone` - Drone body frame
  - `lidar_link` - LiDAR sensor frame
  - `imu_link` - IMU sensor frame
  - `camera_init` - FAST-LIO initial frame
- **Settings**:
  - Marker Scale: 0.5
  - Show Names: Yes
  - Show Axes: Yes

**Usage**: Monitor frame transformations and verify TF tree structure

---

### 2. **Odometry** ✅ Enabled
- **Topic**: `/Odometry`
- **Purpose**: Real-time drone pose from FAST-LIO
- **Visualization**: Red arrow showing position and orientation
- **Settings**:
  - Arrow Length: 1.5m
  - History: 50 poses
  - Covariance: Disabled (cleaner view)

**Usage**: Track drone movement and orientation in real-time

---

### 3. **Path** ✅ Enabled
- **Topic**: `/path`
- **Purpose**: Drone trajectory over time
- **Visualization**: Green line trail
- **Settings**:
  - Color: Green (0, 255, 0)
  - Line Width: 0.05m
  - Buffer: All poses

**Usage**: Visualize the complete flight path and SLAM trajectory

---

### 4. **VelodyneRaw** ⚠️ Disabled by default
- **Topic**: `/velodyne_points`
- **Purpose**: Raw LiDAR scans from sync node
- **Visualization**: White point cloud (intensity-based)
- **Settings**:
  - Point Size: 2 pixels
  - Decay Time: 0 (current scan only)
  - Color: Intensity-based

**Enable**: To see raw sensor input before FAST-LIO processing

**Usage**: Debug LiDAR data issues or verify sync node output

---

### 5. **CloudRegistered** ✅ Enabled
- **Topic**: `/cloud_registered`
- **Purpose**: FAST-LIO processed and registered point cloud map
- **Visualization**: Rainbow-colored by height (Z-axis)
- **Settings**:
  - Point Size: 2-3 pixels
  - Decay Time: 100s (accumulated map)
  - Color: AxisColor (Z-axis rainbow)

**Usage**: Primary map visualization - shows accumulated SLAM map

---

### 6. **CloudBodyFrame** ⚠️ Disabled by default
- **Topic**: `/cloud_registered_body`
- **Purpose**: Current scan in drone body frame
- **Visualization**: Colored point cloud
- **Settings**:
  - Point Size: 3-4 pixels
  - Decay Time: 0 (current only)
  - Style: Flat Squares

**Enable**: To see current scan relative to drone

**Usage**: Debug registration issues or visualize local observations

---

### 7. **MapCloud** ⚠️ Disabled by default
- **Topic**: `/Laser_map`
- **Purpose**: Global accumulated map (if published)
- **Visualization**: Height-colored point cloud
- **Settings**:
  - Point Size: 2 pixels
  - Decay Time: 0
  - Color: AxisColor (Z-axis)

**Enable**: For dense global map visualization

**Usage**: View complete accumulated map (may be resource-intensive)

---

### 8. **Grid** ✅ Enabled
- **Reference Frame**: `world`
- **Purpose**: Ground plane reference
- **Settings**:
  - Cell Size: 5m
  - Plane: XY (horizontal)
  - Color: Gray (160, 160, 160)
  - Alpha: 0.3 (semi-transparent)
  - Cells: 20x20 (100m x 100m)

**Usage**: Spatial reference for understanding scale and position

---

### 9. **WorldOrigin** ✅ Enabled
- **Reference Frame**: `world`
- **Purpose**: Shows world coordinate system origin
- **Settings**:
  - Axes Length: 2m
  - Axes Radius: 0.1m

**Usage**: Reference point for global coordinates

---

## Global Settings

### Fixed Frame
**Value**: `world`

This is the PX4 global reference frame. All visualizations are displayed relative to this frame.

**Why `world`?**
- Matches PX4 convention
- FAST-LIO velodyne config uses `map_frame_id: "world"`
- Enables proper integration with PX4 navigation stack

### Background Color
**Value**: Dark Gray (48, 48, 48)

Provides good contrast for colored point clouds without eye strain.

### Frame Rate
**Value**: 30 FPS

Balances smooth visualization with CPU usage.

---

## Camera View

### Default View Settings
- **Type**: Orbit Camera
- **Distance**: 25m from focal point
- **Target**: `drone` frame (follows drone)
- **Pitch**: 0.785 rad (~45°)
- **Yaw**: 0.785 rad (~45°)

This provides a good overview perspective that follows the drone.

### Adjusting the View

**To follow drone closely:**
1. Change "Target Frame" to `drone`
2. Reduce "Distance" to 5-10m

**To view from above (bird's eye):**
1. Set "Pitch" to 1.57 rad (90°)
2. Set "Target Frame" to `world`

**To reset view:**
- Click "Reset" in Views panel
- Or press `R` key

---

## Performance Tips

### For Better Performance

1. **Disable unnecessary displays**:
   ```
   VelodyneRaw: OFF (unless debugging)
   CloudBodyFrame: OFF (unless debugging)
   MapCloud: OFF (unless needed)
   ```

2. **Reduce point cloud size**:
   - Adjust "Size (m)" to smaller values
   - Use "Points" style instead of "Flat Squares"

3. **Reduce decay time**:
   - Lower "Decay Time" on CloudRegistered (e.g., 30s instead of 100s)

### For Better Visualization

1. **Enable raw data view**:
   ```
   VelodyneRaw: ON
   ```

2. **Increase point sizes**:
   - CloudRegistered: 3-5 pixels
   - Set "Size (m)" to 0.05-0.1

3. **Show covariances**:
   - Odometry → Covariance → Value: ON

---

## Common Adjustments

### Change Map Color Scheme

**CloudRegistered Panel**:
- Color Transformer: `AxisColor` (height-based)
- Or: `Intensity` (sensor reflectivity)
- Or: `FlatColor` (single color)

### Show/Hide Frames

**TF Panel → Frames**:
- Toggle individual frames on/off
- Useful for decluttering view

### Adjust Point Cloud Density

**Any PointCloud2 Display**:
- Increase "Size (Pixels)" for denser appearance
- Or increase "Size (m)" for physical point size

---

## Topics Reference

| Display Name | Topic | Type | Purpose |
|--------------|-------|------|---------|
| Odometry | `/Odometry` | nav_msgs/Odometry | Drone pose |
| Path | `/path` | nav_msgs/Path | Trajectory |
| VelodyneRaw | `/velodyne_points` | PointCloud2 | Raw scans |
| CloudRegistered | `/cloud_registered` | PointCloud2 | SLAM map |
| CloudBodyFrame | `/cloud_registered_body` | PointCloud2 | Local scan |
| MapCloud | `/Laser_map` | PointCloud2 | Global map |

---

## Troubleshooting

### Issue: "No TF data" warning
**Solution**:
1. Ensure FAST-LIO is running
2. Check that sync node is publishing to `/velodyne_points`
3. Verify TF tree: `ros2 run tf2_tools view_frames`

### Issue: Point clouds not visible
**Solutions**:
1. Check Fixed Frame is `world`
2. Ensure topics are being published: `ros2 topic list`
3. Check topic rates: `ros2 topic hz /cloud_registered`
4. Verify QoS settings match publishers

### Issue: RViz is slow/laggy
**Solutions**:
1. Disable VelodyneRaw, CloudBodyFrame, MapCloud
2. Reduce CloudRegistered decay time to 30s
3. Reduce point sizes
4. Lower frame rate to 15 FPS

### Issue: Map appears upside down
**Solution**: Fixed frame should be `world`, not `camera_init`

### Issue: Camera doesn't follow drone
**Solution**:
1. Views → Current View → Target Frame → `drone`
2. Or keep as `world` for global view

---

## Saving Custom Configurations

1. Make your adjustments in RViz
2. File → Save Config As
3. Save to: `~/.ros/my_custom.rviz`
4. Load with:
   ```bash
   ros2 launch fast_lio_ros2 simulation_mapping.launch.py \
       rviz_cfg:=~/.ros/my_custom.rviz
   ```

---

## Comparison with Other Configs

| Feature | mobile_robot.rviz | velodyne_sim.rviz |
|---------|-------------------|-------------------|
| Fixed Frame | `map` | `world` (PX4) |
| TF Frames | `map`, `base_link` | `world`, `drone`, `lidar_link`, `imu_link` |
| Raw Points | Not included | `/velodyne_points` |
| Grid Reference | No | Yes (100m x 100m) |
| Target Frame | `map` | `drone` (follows) |
| Optimized For | Mobile robot | PX4 drone simulation |

---

## Quick Reference

### Enable/Disable Displays
Click checkbox next to display name in Displays panel

### Change Colors
Display → Color Transformer → Choose option

### Adjust View
- Scroll: Zoom in/out
- Right-drag: Rotate
- Middle-drag: Pan
- `R`: Reset view

### Show More Data
TF → Frames → All Enabled: Toggle ON

---

## Integration with Launch File

The `simulation_mapping.launch.py` automatically uses this config:

```python
default_rviz_config_path = os.path.join(
    package_path, 'rviz', 'velodyne_sim.rviz')
```

To use a different config:
```bash
ros2 launch fast_lio_ros2 simulation_mapping.launch.py \
    rviz_cfg:=/path/to/other.rviz
```

To disable RViz:
```bash
ros2 launch fast_lio_ros2 simulation_mapping.launch.py \
    rviz:=false
```

---

## Advanced Features

### Multiple Viewports

You can create split views:
1. Views → New View
2. Configure second camera
3. Right-click viewport → Split

### Recording Views

Create saved viewpoints:
1. Position camera as desired
2. Views → Saved → Add Current View
3. Switch between saved views

### Time Synchronization

Time panel shows:
- Simulation time (if use_sim_time:=true)
- Real-time
- Sync source: CloudRegistered topic

---

## Summary

The `velodyne_sim.rviz` configuration provides:

✅ **Optimized for PX4 Gazebo** - Proper frames and topics
✅ **Performance-focused** - Only essential displays enabled by default
✅ **Follow-drone camera** - Tracks drone movement
✅ **Height-based coloring** - Easy map interpretation
✅ **Grid reference** - Spatial awareness
✅ **Debugging options** - Raw data available when needed

**Default Launch**: `ros2 launch fast_lio_ros2 simulation_mapping.launch.py`

---

## References

- RViz2 Documentation: http://wiki.ros.org/rviz2
- FAST-LIO Topics: Check package documentation
- PX4 Frames: https://docs.px4.io/main/en/ros/

---

**Created**: 2025-11-05
**For**: PX4 Gazebo + FAST-LIO + Velodyne Simulation
