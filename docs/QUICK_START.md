# FAST-LIO ROS2 — 3D Reconstruction Quick Start

Dual Livox MID-360 reconstruction on NVIDIA Jetson ORIN.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Reconstruction Workflow](#reconstruction-workflow)
3. [Rosbag Playback](#rosbag-playback)
4. [Config Reference](#config-reference)
5. [Important Notes](#important-notes)
6. [Troubleshooting](#troubleshooting)

---

## Prerequisites

```bash
# 1. Build the workspace
cd ~/ros2_ws
colcon build --packages-select fast_lio_ros2 --symlink-install

# 2. Source the workspace (add to ~/.bashrc for convenience)
source ~/ros2_ws/install/setup.bash

# 3. Verify livox_ros_driver2 is publishing topics (live sensors only)
ros2 topic list | grep livox
# Expected:
#   /livox/lidar_192_168_1_10
#   /livox/lidar_192_168_1_18
#   /livox/imu_192_168_1_10
```

---

## Reconstruction Workflow

Optimized for generating dense 3D maps on Jetson ORIN. Uses 12cm voxels and every-3rd-point
decimation to balance map quality with processing throughput. Non-essential topic publishing
is disabled to save CPU. All processed points are accumulated and saved to PCD on shutdown.

### Step 1 — Launch FAST-LIO

**From rosbag (typical):**
```bash
ros2 launch fast_lio_ros2 dual_mapping_core.launch.py modo:=reconstruction
```

> `use_sim_time` defaults to `true` automatically when `modo:=reconstruction`.
> No need to pass it explicitly for rosbag playback.

**Live sensors:**
```bash
ros2 launch fast_lio_ros2 dual_mapping_core.launch.py modo:=reconstruction use_sim_time:=false
```

### Step 2 — Play the rosbag

```bash
ros2 bag play <path_to_bag>/ --clock --rate 0.5
```

> Use `--rate 0.5` on Jetson ORIN to prevent buffer overflow. Desktop systems
> can use `--rate 0.8` or higher.

### Step 3 — Wait for completion

Wait for the rosbag to finish playing. The FAST-LIO node will continue processing
any remaining buffered scans (the 500-scan buffer may still be draining).

### Step 4 — Save the map

Press `Ctrl+C` in the FAST-LIO terminal. The node will:
1. Deactivate cleanly (lifecycle shutdown)
2. Save the accumulated map to a PCD file in `PCD/`
3. Print: `Map saved to .../PCD/reconstruction_map_<timestamp>.pcd (N points)`

> **CRITICAL:** Always use `Ctrl+C` for a clean shutdown. Using `kill -9` will lose
> the unsaved PCD data.

### Step 5 — Verify the output

```bash
ls -lh ~/ros2_ws/src/fast_lio_ros2/PCD/reconstruction_map_*.pcd
```

### Published Topics

During reconstruction, only essential topics are published to save CPU:

| Topic | Description |
|-------|-------------|
| `/Odometry` | 6-DOF pose + velocity |
| `/path` | Trajectory visualization |
| `/cloud_registered` | Current scan (downsampled) |

> Topics `map_en`, `effect_map_en`, `dense_publish_en`, and `scan_bodyframe_pub_en`
> are disabled in reconstruction mode. The PCD save is independent of topic publishing.

---

## Rosbag Playback

### Available test bags

Located at: `/home/jetson/xtract-payload-orchestrator/data/rosbags/`

| Bag | Size | Duration | Environment |
|-----|------|----------|-------------|
| `Livox_20260210_141812` | 153MB | ~20s | Quick test |
| `Livox_20260210_142603` | 819MB | ~1.5 min | Short run |
| `Livox_20260210_151228` | 1.2GB | ~2.5 min | Medium run |
| `Livox_20260210_141623` | 1.8GB | ~4 min | Indoor/outdoor |
| `Livox_20260210_141934` | 2.0GB | ~5 min | Full reconstruction |

### Complete workflow

```bash
# Terminal 1: Launch FAST-LIO (use_sim_time auto-enabled)
ros2 launch fast_lio_ros2 dual_mapping_core.launch.py modo:=reconstruction

# Terminal 2: Play bag
ros2 bag play /home/jetson/xtract-payload-orchestrator/data/rosbags/Livox_20260210_141623/ \
    --clock --rate 0.5

# Wait for bag to finish, then Ctrl+C in Terminal 1
```

### Recommended playback rates

| Platform | Rate | Notes |
|----------|------|-------|
| Jetson ORIN | 0.5 | Prevents buffer overflow |
| Desktop (modern) | 0.8 - 1.0 | May handle real-time |

---

## Config Reference

### Reconstruction parameters (`config/reconstruction.yaml`)

| Parameter | Value | Effect |
|-----------|-------|--------|
| `point_filter_num` | 3 | Keep every 3rd point from LiDAR 1 |
| `point_filter_num2` | 3 | Keep every 3rd point from LiDAR 2 |
| `filter_size_surf` | 0.12m | Scan voxel filter before processing |
| `filter_size_map` | 0.12m | Map voxel filter (ikd-tree resolution) |
| `cube_side_length` | 2000m | Very large — prevents point trimming |
| `det_range` | 30m | Max detection range |
| `max_iteration` | 3 | EKF iterations per scan |
| `blind` | 0.5m | Min distance filter |
| `update_method` | 1 (ASYNC) | Independent LiDAR processing |
| `dense_publish_en` | false | Saves serialization CPU |
| `map_en` | false | Saves CPU — global map pub is expensive |
| `effect_map_en` | false | Saves CPU |
| `pcd_save_en` | true | Save accumulated map on shutdown |

### Tuning for higher density (if hardware permits)

To increase map resolution at the cost of slower processing:

```yaml
# Denser reconstruction (desktop or slower playback rate needed)
point_filter_num: 2       # Every 2nd point
point_filter_num2: 2
filter_size_surf: 0.08    # 8cm voxels
filter_size_map: 0.08
```

### Config file location

```
fast_lio_ros2/config/
  reconstruction.yaml     # 3D reconstruction (this branch)
```

---

## Important Notes

### Point cloud retention

- The **ikd-tree** (internal map) maintains a sliding local map of `cube_side_length` meters.
  With 2000m, this effectively retains all points during reconstruction.
- The **PCD save** accumulates ALL processed scans independently of the ikd-tree. It uses the
  undistorted point cloud before map voxel filtering.
- The `/cloud_registered` topic publishes only the **current scan**, not the accumulated map.

### Memory considerations (Jetson ORIN)

- Reconstruction mode accumulates all points in RAM until shutdown. For long runs (>5 min),
  monitor memory usage: `watch free -h`
- If the node is OOM-killed before saving, the PCD is lost. For very long runs, consider
  using `pcd_save.interval: 100` to save intermediate files every 100 scans.

### Buffer behavior

- Both LiDARs have a **buffer of 500 scans** (~50s at 10Hz). If processing can't keep up,
  the buffer overflows and oldest scans are dropped. The large buffer provides headroom
  during brief processing spikes.
- Play rosbags at reduced speed (`--rate 0.5`) to prevent buffer overflow on Jetson.
- After the rosbag finishes, wait ~10-20s for the buffer to drain before pressing `Ctrl+C`.

### Dual LiDAR behavior

- ASYNC mode (`update_method: 1`): LiDAR 1 and LiDAR 2 scans are processed alternately.
  Each scan independently updates the EKF state.
- Both LiDARs contribute equally to the reconstruction — expect ~equal scan counts.

### IMU transform

The MID-360 is mounted with non-standard orientation. The IMU transform
(roll=90, pitch=0, yaw=180) is **required** and must match the `livox_ros_driver2`
extrinsic configuration. Do not change these unless you change the physical mounting.

---

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| No points processed | `use_sim_time` is true but no `--clock` on bag play | Add `--clock` flag to `ros2 bag play` |
| "Buffer full, dropping oldest scan" | Processing too slow for incoming data | Reduce bag `--rate`; increase `filter_size_surf`/`point_filter_num` |
| No PCD saved on shutdown | Node killed with `kill -9` or OOM crash | Always use `Ctrl+C`; check `dmesg` for OOM |
| Map has holes/missing areas | `cube_side_length` too small, points trimmed | Increase `cube_side_length` (default: 2000m) |
| LiDAR 2 has fewer points | `point_filter_num2` > filter_num drops more | Ensure both use same `point_filter_num` value |
| "IMU Initial Done" then nothing | Waiting for LiDAR data after IMU init | Check LiDAR topics: `ros2 topic hz /livox/lidar_*` |
| Drift in long reconstructions | No loop closure in FAST-LIO | Expected — accumulates over distance |
| `rcutils` serialization errors | Known ROS2 Humble DDS issue with Livox msgs | Harmless warnings, can be ignored |
| "No Effective Points" after init | Sparse map from early dropped scans | Reduce `--rate` further; check extrinsics |
