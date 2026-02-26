# Dual LiDAR Reconstruction Verification Guide

## Overview
This guide explains how to verify the dual Livox MID-360 3D reconstruction output and diagnose alignment or quality issues.

---

## Quick Start — Reconstruction Test

### On Jetson
```bash
# Terminal 1: Launch reconstruction
ros2 launch fast_lio_ros2 dual_mapping_core.launch.py modo:=reconstruction

# Terminal 2: Play rosbag
ros2 bag play <path_to_bag>/ --clock --rate 0.5

# Wait for bag to finish, then Ctrl+C in Terminal 1
```

### Verify Output
```bash
# Check PCD was saved
ls -lh ~/ros2_ws/src/fast_lio_ros2/PCD/reconstruction_map_*.pcd

# Expected: File with millions of points, hundreds of MB
```

---

## Verification Checks

### Check 1: Both LiDARs Contributing

During reconstruction, verify both LiDARs are processing scans:

**Console output should show alternating scans:**
```
[LiDAR 1 Livox] Received scan #100, timestamp: ...
[LiDAR 2 Livox] Received scan #98, timestamp: ...
```

Both scan counts should be roughly equal. If one is significantly lower, check:
- Topic connectivity: `ros2 topic hz /livox/lidar_192_168_1_10` and `_18`
- Config: `multi_lidar: true` in `reconstruction.yaml`

### Check 2: No Buffer Overflow

Look for these warnings in the console:
```
[WARN] Buffer full (500), dropping oldest scan
```

If present:
- Reduce playback rate: `--rate 0.3`
- Increase `point_filter_num` / `filter_size_surf` in config

### Check 3: No "No Effective Points" Warnings

After the initial 1-2 scans, there should be zero "No Effective Points" warnings.
These indicate the EKF cannot find enough map correspondences and will produce gaps.

### Check 4: Odometry Publishing

```bash
ros2 topic hz /Odometry
# Expected: ~10-20 Hz (varies with processing speed during reconstruction)
```

---

## Visual Verification (Desktop)

### Using CloudCompare

Open the PCD file on a desktop machine:

1. **Density check**: The map should have uniform density throughout.
   Areas scanned while the robot was stationary may show slightly lower density
   due to ikd-tree downsampling (see [POINT_CLOUD_DELETION_ANALYSIS.md](POINT_CLOUD_DELETION_ANALYSIS.md)).

2. **Alignment check**: Walls and surfaces should appear as single planes, not doubled.
   Double walls indicate extrinsic calibration issues between LiDAR 1 and LiDAR 2.

3. **Drift check**: If the robot returned to the start position, check if the map
   closes properly. Note: FAST-LIO has no loop closure, so some drift is expected
   on long trajectories.

### Using RViz2 (Live Monitoring)

On a desktop connected to the same ROS2 domain:
```bash
rviz2
```

Add displays:
- `/cloud_registered` — Current scan in world frame
- `/Odometry` — Trajectory
- `/path` — Full path history

Note: In reconstruction mode, `map_en` and `dense_publish_en` are disabled to save CPU.
The `/cloud_registered` topic shows downsampled current scans only.

---

## Extrinsic Calibration Verification

### Static Scene Test (from rosbag)

1. Find a section of the bag where the robot is near a clear geometric feature (wall, corner)
2. In the PCD output, inspect overlapping FOV regions
3. Features should be crisp, not doubled or blurred

### Good Alignment
```
Wall cross-section:
L1 points: ═══════════
L2 points: ═══════════
Result: Clean single wall
```

### Bad Alignment (Translation Error)
```
Wall cross-section:
L1 points: ═══════════
L2 points:    ═══════════
Result: Double wall (adjust extrinsic_T_L2_wrt_L1)
```

### Bad Alignment (Rotation Error)
```
Wall cross-section:
L1 points: ═══════════
L2 points: ═══════╱
Result: Angled overlap (adjust extrinsic_R_L2_wrt_L1)
```

See [EXTRINSIC_CALIBRATION_GUIDE.md](EXTRINSIC_CALIBRATION_GUIDE.md) for adjustment procedure.

---

## Performance Monitoring

### CPU Usage (on Jetson)
```bash
top -p $(pgrep fastlio_mapping)
```
**Expected during reconstruction**: 50-80% CPU (single core)

### Memory Usage
```bash
watch free -h
```
**Monitor during long runs** — PCD accumulation grows memory continuously.
A 5-minute bag at 0.5x rate typically produces 20-25M points (~700MB PCD).

### Buffer Health

No "Buffer full" warnings = healthy. If they appear:

| Fix | Impact |
|-----|--------|
| Reduce `--rate` | Slows playback, gives more processing time |
| Increase `point_filter_num` | Fewer points per scan, less accuracy |
| Increase `filter_size_surf` | Coarser voxels, less detail |

---

## Troubleshooting

### Problem: Only one LiDAR shows data

```bash
# Verify topics exist in the bag
ros2 bag info <bag_path> | grep lidar

# Check data rate during playback
ros2 topic hz /livox/lidar_192_168_1_10
ros2 topic hz /livox/lidar_192_168_1_18
```

**Solution:**
- Verify `multi_lidar: true` in config
- Check that bag contains both LiDAR topics

### Problem: Severe misalignment in PCD

1. Verify physical sensor mounting matches config
2. Confirm sensor IPs match config topics
3. Check if sensors were swapped during installation
4. Re-run with `extrinsic_est_en: true` for online estimation

### Problem: Map density drops in stationary areas

This is expected behavior — see [POINT_CLOUD_DELETION_ANALYSIS.md](POINT_CLOUD_DELETION_ANALYSIS.md).
The ikd-tree downsamples when re-scanning the same area.

---

## Success Criteria

A successful reconstruction should show:
1. Both LiDAR scan counts roughly equal (~5% difference is normal)
2. Zero "Buffer full" warnings throughout the run
3. Zero "No Effective Points" warnings after initialization
4. PCD file contains millions of points with uniform density
5. Smooth, continuous trajectory with no jumps
6. Clean single-surface geometry (no double walls from misalignment)

---

## Version Info
- FAST-LIO: ROS2 Dual LiDAR 3D Reconstruction
- Branch: `jetson-dev-rec`
- Hardware: Livox MID-360 (Firmware >= 1.0)
- ROS2: Humble
- Platform: NVIDIA Jetson Orin
