# FAST-LIO Dual MID-360: BUNDLE vs ASYNC Mode Comparison

**Date:** 2026-01-22
**Hardware:** NVIDIA Jetson Orin + 2x Livox MID-360
**ROS2:** Humble
**Branch:** `jetson-dev-rec` (3D Reconstruction)
**Rosbag:** Dual LiDAR recording with ~100ms timestamp offset between sensors

---

## Executive Summary

**ASYNC mode (`update_method: 1`) is used for all 3D reconstruction** on this branch. BUNDLE mode requires tight temporal synchronization (<100ms) which is not achievable with standard MID-360 configurations. This document records the comparison tests that led to this decision.

---

## Test Configuration

### Hardware Setup
- **LiDAR 1:** MID-360 @ 192.168.1.10 (Front, y=+110mm, yaw=180°)
- **LiDAR 2:** MID-360 @ 192.168.1.18 (Rear, y=-110mm, yaw=0°)
- **IMU:** Built-in MID-360 IMU from LiDAR 1
- **Observed timestamp offset:** ~100ms between LiDARs (varies 100-200ms)

### Parameter Sets Tested

| Parameter | Standard | Dense |
|-----------|----------|-------|
| `filter_size_surf` | 0.2m | 0.1m |
| `filter_size_map` | 0.2m | 0.1m |
| `point_filter_num` | 2 | 1 |

---

## Test Results

### Test 1: Standard Parameters (filter_size: 0.2, point_filter_num: 2)

#### BUNDLE Mode (update_method: 0)
| Metric | Value | Status |
|--------|-------|--------|
| Odometry Rate | 7-10 Hz | Poor |
| Time Desync Warnings | ~50-70% of scans | Critical |
| Max Desync Observed | 400ms | Exceeds 100ms tolerance |
| Buffer Overflow | Frequent (up to 30 scans) | Critical |
| Scan Utilization | ~30-50% | Poor |
| "No Effective Points" | Occasional | Warning |

**Issues:**
- Hardcoded 100ms tolerance in `laserMapping.cpp:438` is exceeded
- Scans frequently discarded due to time desync
- Buffer buildup causes processing lag
- Inconsistent odometry output

#### ASYNC Mode (update_method: 1)
| Metric | Value | Status |
|--------|-------|--------|
| Odometry Rate | ~20 Hz | Excellent |
| Time Desync Warnings | 0 | Perfect |
| Buffer Overflow | None | Perfect |
| Scan Utilization | 100% | Excellent |
| Processing Stability | Smooth | Excellent |

**Advantages:**
- Each LiDAR processed independently - no sync required
- All scans utilized for mapping
- Double the odometry update rate
- No buffer issues

---

### Test 2: Dense Parameters (filter_size: 0.1, point_filter_num: 1)

#### BUNDLE Mode + Dense
| Metric | Value | Status |
|--------|-------|--------|
| Odometry Rate | ~1.3 Hz | Critical |
| Time Desync Warnings | 43+ | Critical |
| Max Desync Observed | 700ms | Critical |
| Buffer Overflow | Constant (100 scans) | Critical |
| "No Effective Points" | Frequent | Critical |
| Point Cloud Size | 16,162 pts | N/A |

**Result:** System cannot keep up. Not viable.

#### ASYNC Mode + Dense
| Metric | Value | Status |
|--------|-------|--------|
| Time Desync Warnings | 0 | Good |
| Buffer Overflow | LiDAR 2 only | Moderate |
| Processing | Continues despite buffer pressure | Acceptable |

**Result:** Better than BUNDLE but still struggling with Jetson hardware limits.

---

## Root Cause Analysis

### Why BUNDLE Mode Fails

1. **Timestamp Mismatch:** The two MID-360 LiDARs are not hardware-synchronized. Each has its own internal clock, resulting in ~100ms offset.

2. **Hardcoded Tolerance:** The BUNDLE mode has a hardcoded 100ms tolerance at `laserMapping.cpp:438`:
   ```cpp
   const double time_tolerance = 0.10;  // 100ms
   ```

3. **Synchronization Logic:** BUNDLE requires both LiDAR scans to be within the tolerance window to merge them. With ~100ms natural offset, this frequently fails.

4. **Cascading Effects:** Discarded scans → buffer buildup → processing lag → more discards

### Why ASYNC Mode Works

1. **Independent Processing:** Each LiDAR is processed separately, no synchronization required.

2. **Higher Update Rate:** Both LiDARs contribute to odometry independently (~10Hz each = ~20Hz total).

3. **No Buffer Pressure:** Without waiting for sync, scans are processed immediately.

---

## Recommendations

### For Dual MID-360 Without Hardware Sync

**Use ASYNC mode (update_method: 1)** - This is the only viable option.

```yaml
update_method: 1  # ASYNC mode
```

### For Denser Maps (if hardware permits)

Try intermediate values rather than aggressive dense settings:

```yaml
# Moderate density (recommended starting point)
filter_size_surf: 0.15
filter_size_map: 0.15
point_filter_num: 2

# Dense (only if Jetson can handle it)
filter_size_surf: 0.1
filter_size_map: 0.1
point_filter_num: 2  # Keep at 2 to reduce load
```

### If BUNDLE Mode is Required

Options to make BUNDLE work (requires code changes):

1. **Increase time tolerance** in `laserMapping.cpp:438`:
   ```cpp
   const double time_tolerance = 0.15;  // or 0.20
   ```

2. **Hardware time sync:** Use PTP/PPS synchronization between LiDARs (requires hardware support)

3. **Software timestamp alignment:** Pre-process rosbags to align timestamps

---

## Performance Summary Table

| Mode | Params | Odom Rate | Warnings | Stability | Recommendation |
|------|--------|-----------|----------|-----------|----------------|
| BUNDLE | Standard | 7-10 Hz | Many | Poor | Not Recommended |
| BUNDLE | Dense | 1.3 Hz | Critical | Failing | Not Viable |
| **ASYNC** | **Standard** | **20 Hz** | **0** | **Excellent** | **Recommended** |
| ASYNC | Dense | ~10 Hz | Buffer only | Moderate | Hardware Limited |

---

## Conclusion

For the dual MID-360 3D reconstruction setup on Jetson Orin:

1. **Always use ASYNC mode** (`update_method: 1`) — set in `reconstruction.yaml`
2. **Reconstruction parameters** (`filter_size: 0.12`, `point_filter_num: 3`) are tuned for Jetson throughput
3. **Non-essential publishing disabled** (`map_en: false`, `dense_publish_en: false`) to free CPU
4. **500-scan buffer** absorbs processing spikes during rosbag playback at `--rate 0.5`

The ~100ms timestamp offset between unsynchronized MID-360 LiDARs is fundamental and cannot be resolved without hardware sync or code modifications to increase the tolerance threshold.

---

## Files Modified

- `config/dual_mid360.yaml`: Set `update_method: 1` for ASYNC mode
- Future: Consider making `time_tolerance` a configurable parameter

## Test Environment

- Date: 2026-01-22
- Tester: Claude Code Analysis
- Rosbag: Dual MID-360 recording
- Platform: Jetson Orin, ROS2 Humble
