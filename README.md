ROS2 Fork repo maintainer: [Kmedrano101](https://github.com/Kmedrano101)
# FAST-LIO2 ROS2 — 3D Reconstruction Branch

## About the `jetson-dev-rec` Branch

This branch is purpose-built for **offline 3D reconstruction** using dual Livox MID-360 LiDARs on NVIDIA Jetson ORIN. It processes rosbag recordings to generate dense PCD point cloud maps.

**Key features:**
- Dual MID-360 LiDAR support (ASYNC mode)
- Jetson ORIN optimized — 12cm voxels, every-3rd-point decimation, non-essential publishing disabled
- 500-scan buffer to absorb processing spikes during rosbag playback
- `use_sim_time` auto-enabled for reconstruction mode
- PCD output with timestamped filenames on clean shutdown (`Ctrl+C`)

### Quick Start

```bash
# 1. Build
cd ~/ros2_ws
colcon build --packages-select fast_lio_ros2
source install/setup.bash

# 2. Launch (use_sim_time defaults to true for reconstruction)
ros2 launch fast_lio_ros2 dual_mapping_core.launch.py modo:=reconstruction

# 3. Play rosbag (separate terminal)
ros2 bag play <path_to_bag>/ --clock --rate 0.5

# 4. Wait for bag to finish, then Ctrl+C to save PCD
# Output: PCD/reconstruction_map_<timestamp>.pcd
```

See [docs/QUICK_START.md](docs/QUICK_START.md) for the full guide.

### Configuration

The reconstruction config is at `config/reconstruction.yaml`. Key parameters tuned for Jetson:

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `filter_size_surf` | 0.12m | 12cm voxels — dense but processable on Jetson |
| `filter_size_map` | 0.12m | Match surf resolution |
| `point_filter_num` | 3 | Every 3rd point from each LiDAR |
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
