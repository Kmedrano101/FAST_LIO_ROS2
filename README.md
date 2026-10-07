ROS2 Fork repo maintainer: [Kmedrano101](https://github.com/Kmedrano101)
# FAST-LIO2 ROS2 — 3D Reconstruction Branch

## About the `jetson-dev-rec` Branch

This branch is purpose-built for **offline 3D reconstruction** using dual Livox MID-360 LiDARs on NVIDIA Jetson ORIN. It processes rosbag recordings to generate dense PCD point cloud maps.

**Key features:**
- Dual MID-360 LiDAR support (ASYNC mode)
- Jetson ORIN optimized — 12cm voxels, all points from both LiDARs, non-essential publishing disabled
- 500-scan buffer to absorb processing spikes during rosbag playback
- `use_sim_time` auto-enabled for reconstruction mode
- PCD output with timestamped filenames on clean shutdown (`Ctrl+C`)

### Quick Start

```bash
# 1. Build (replace ~/ros2_ws with your workspace, e.g. ~/tidop_ws)
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2 fast_lio_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 2. Terminal 1 — launch FAST-LIO (use_sim_time defaults to true for reconstruction)
ros2 launch fast_lio_ros2 dual_mapping_core.launch.py mode:=reconstruction

# 3. Terminal 2 — play only the LiDAR + IMU topics of the rosbag
ros2 bag play <path_to_bag>/ --clock --rate 0.5 --topics \
    /livox/lidar_192_168_1_10 /livox/lidar_192_168_1_18 \
    /livox/imu_192_168_1_10 /livox/imu_192_168_1_18

# 4. When the bag finishes, wait ~10-20 s and save the map (Terminal 2)
ros2 param set /fastlio_mapping map_file_path <path_to_bag>/<bag_name>_reconstruction.pcd
ros2 service call /map_save std_srvs/srv/Trigger

# 5. Ctrl+C in Terminal 1 to stop FAST-LIO
```

> **The launch argument is `mode`, not `modo`.** An unknown argument such as
> `modo:=reconstruction` is silently ignored and FAST-LIO runs with
> `navigation.yaml` (BUNDLE, 20 cm voxels, 1-in-3 points). Check that the log
> prints `Update method: 1 (ASYNC)`.

> **Why `--topics`:** XTRACT bags also record FAST-LIO's own `/Odometry` (and GoPro
> images). Replaying the recorded `/Odometry` would clash with the one FAST-LIO publishes.

> **`/map_save` vs `Ctrl+C`:** without calling `/map_save`, `Ctrl+C` saves the map to
> `PCD/reconstruction_map_<timestamp>.pcd`. `/map_save` clears the accumulator after
> saving (`pcd_save.reset_after_save: true`), so a later `Ctrl+C` saves nothing.

See [docs/QUICK_START.md](docs/QUICK_START.md#reconstruction-workflow) for the full step-by-step guide (verification, buffer behaviour, troubleshooting).

### Configuration

The reconstruction config is at `config/reconstruction.yaml`. Key parameters tuned for Jetson:

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `filter_size_surf` | 0.12m | 12cm voxels — dense but processable on Jetson |
| `filter_size_map` | 0.12m | Match surf resolution |
| `point_filter_num` / `point_filter_num2` | 1 | All points from each LiDAR (maximum density) |
| `max_iteration` | 3 | Fast EKF convergence |
| `det_range` | 30m | Bounded map management |
| `map_en` | false | Saves CPU — PCD save is independent of topic publishing |
| `cube_side_length` | 2000m | No point trimming during reconstruction |

### Documentation

| Document | Purpose |
|----------|---------|
| [QUICK_START.md](docs/QUICK_START.md) | Step-by-step reconstruction workflow |
| [EXTRINSIC_CALIBRATION_GUIDE.md](docs/EXTRINSIC_CALIBRATION_GUIDE.md) | Dual MID-360 calibration chain |
| [DUAL_LIDAR_TEST_GUIDE.md](docs/DUAL_LIDAR_TEST_GUIDE.md) | Verifying reconstruction output |
| [POINT_CLOUD_DELETION_ANALYSIS.md](docs/POINT_CLOUD_DELETION_ANALYSIS.md) | Map density analysis and fixes |
| [EXECUTIVE_REPORT_MULTI_LIDAR.md](docs/EXECUTIVE_REPORT_MULTI_LIDAR.md) | System architecture overview |
| [BUNDLE_VS_ASYNC_COMPARISON.md](docs/BUNDLE_VS_ASYNC_COMPARISON.md) | Why ASYNC mode is used |
| [CUSTOMMSG_MID360_SUPPORT.md](docs/CUSTOMMSG_MID360_SUPPORT.md) | CustomMsg format implementation |
| [DEEP_ANALYSIS_REPORT.md](docs/DEEP_ANALYSIS_REPORT.md) | Code analysis and known issues |
| [SDK_FUSION_SETUP.md](docs/SDK_FUSION_SETUP.md) | Alternative SDK fusion approach |

### Building

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fast_lio_ros2
source install/setup.bash
```

> This package targets NVIDIA Jetson ORIN with ROS2 Humble. Ensure `livox_ros_driver2` is installed and configured for dual MID-360.
