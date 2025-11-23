# Adaptive Multi-LiDAR Fusion Mode Guide

## Overview

The Adaptive Multi-LiDAR Fusion mode dynamically switches between **Bundle** and **Async** update strategies based on real-time feature density and field-of-view (FOV) coverage. This provides optimal performance across varying environmental conditions.

## Update Modes

### 1. Bundle Mode (`update_mode: 0`)
**How it works**: Merges point clouds from all LiDAR sensors before performing a single filter update.

**Advantages:**
- Maximum feature points per update
- Robust in sparse environments (tunnels, high altitude)
- Better handling of extreme scenarios

**Disadvantages:**
- Lower update rate (~10 Hz for dual setup)
- Higher latency between measurements
- Increased drift during rapid motion

**Best for:**
- Underground mines with sparse features
- High-altitude drone flight
- Environments with limited geometric features

### 2. Async Mode (`update_mode: 1`)
**How it works**: Updates the filter immediately upon receiving each LiDAR scan independently.

**Advantages:**
- Higher update rate (~20 Hz for dual setup)
- Lower drift from IMU-only propagation
- Faster response to motion changes

**Disadvantages:**
- Risk of insufficient features per update
- May diverge in sparse areas
- Requires good FOV coverage from each sensor

**Best for:**
- Feature-rich environments
- Rapid vehicle motion
- Good LiDAR overlap regions

### 3. Adaptive Mode (`update_mode: 2`) **RECOMMENDED**
**How it works**: Intelligently switches between Bundle and Async based on:
- **Feature density**: Points per cubic meter
- **FOV coverage**: Total points in current scan
- **Stability hysteresis**: Prevents rapid oscillation

**Algorithm:**
```
IF (points >= fov_threshold) AND (density >= feature_density_threshold):
    Use ASYNC mode  → Lower latency
ELSE:
    Use BUNDLE mode → More robust
```

**Advantages:**
- Best of both approaches
- Automatically adapts to environment
- Optimizes for current conditions

**Configuration Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_mode` | int | 2 | 0=Bundle, 1=Async, 2=Adaptive |
| `adaptive_fov_threshold` | int | 5000 | Min points to trigger ASYNC mode |
| `adaptive_feature_density` | double | 0.002 | Min density (pts/m³) for ASYNC |
| `adaptive_hysteresis_ratio` | double | 1.2 | Prevents rapid switching |
| `adaptive_stability_frames` | int | 3 | Frames before mode switch |
| `adaptive_debug_output` | bool | false | Enable verbose logging |

## Configuration Examples

### Mine Environment (Sparse Features)
```yaml
update_mode: 2                       # Adaptive
adaptive_fov_threshold: 3000         # Lower threshold (sparse environment)
adaptive_feature_density: 0.001      # Lower density requirement
```

**Expected behavior:**
- Uses BUNDLE in narrow tunnels
- Switches to ASYNC in larger chambers
- ~10-15 Hz average update rate

### Urban/Forest (Dense Features)
```yaml
update_mode: 2                       # Adaptive
adaptive_fov_threshold: 8000         # Higher threshold
adaptive_feature_density: 0.005      # Higher density requirement
```

**Expected behavior:**
- Primarily ASYNC mode
- Occasional BUNDLE in open areas
- ~18-20 Hz average update rate

### Conservative (Maximum Robustness)
```yaml
update_mode: 0                       # Pure Bundle
# (adaptive parameters ignored)
```

**Expected behavior:**
- Always bundles scans
- ~10 Hz update rate
- Maximum stability

### Aggressive (Minimum Latency)
```yaml
update_mode: 1                       # Pure Async
# (adaptive parameters ignored)
```

**Expected behavior:**
- Always async updates
- ~20 Hz update rate
- Requires good feature density

## Hysteresis Logic

The adaptive mode includes hysteresis to prevent rapid oscillation between modes:

1. **Counting mechanism**: Counts consecutive frames favoring each mode
2. **Stability requirement**: Must favor new mode for `adaptive_stability_frames` consecutive frames
3. **Reset on change**: Counter resets when conditions flip

**Example:**
```
Frame 1: 6000 points → Favors ASYNC, counter=1
Frame 2: 5500 points → Favors ASYNC, counter=2
Frame 3: 6200 points → Favors ASYNC, counter=3  → SWITCH TO ASYNC!
Frame 4: 4800 points → Favors BUNDLE, counter=1 (ASYNC counter reset)
Frame 5: 4500 points → Favors BUNDLE, counter=2
Frame 6: 4900 points → Favors BUNDLE, counter=3 → SWITCH TO BUNDLE!
```

## Performance Characteristics

| Metric | Bundle | Async | Adaptive (avg) |
|--------|--------|-------|----------------|
| Update Rate | 10 Hz | 20 Hz | 12-18 Hz |
| Latency | ~100ms | ~50ms | ~50-80ms |
| Feature Points/Update | High | Medium | Medium-High |
| CPU Usage | Medium | High | Medium-High |
| Memory Usage | Higher | Lower | Medium |
| Robustness (sparse) | Excellent | Poor | Good |
| Robustness (dense) | Good | Excellent | Excellent |

## Monitoring & Debugging

### Enable Debug Output
```yaml
adaptive_debug_output: true
```

**Log output:**
```
[AdaptiveFusion] Strategy switch: ASYNC -> BUNDLE (points: 4200, density: 0.0015)
[AdaptiveFusion] Strategy switch: BUNDLE -> ASYNC (points: 6800, density: 0.0028)
```

### ROS2 Runtime Monitoring

Check current mode:
```bash
ros2 param get /fastlio_mapping update_mode
```

Get statistics (if implemented):
```bash
ros2 topic echo /fastlio_mapping/adaptive_stats
```

### RViz Visualization

The scan rate in RViz indicates the current mode:
- **~10 Hz**: Bundle mode active
- **~20 Hz**: Async mode active
- **Variable**: Adaptive mode switching

## Tuning Guide

### Step 1: Start with Defaults
```yaml
update_mode: 2
adaptive_fov_threshold: 5000
adaptive_feature_density: 0.002
```

### Step 2: Monitor Performance

Run FAST-LIO and observe:
```bash
ros2 topic hz /cloud_registered
```

Enable debug output to see mode switches.

### Step 3: Adjust Based on Environment

**If spending too much time in BUNDLE:**
- Decrease `adaptive_fov_threshold`
- Decrease `adaptive_feature_density`

**If experiencing divergence/drift:**
- Increase `adaptive_fov_threshold`
- Increase `adaptive_feature_density`
- Increase `adaptive_stability_frames`

**If oscillating rapidly:**
- Increase `adaptive_stability_frames`
- Adjust `adaptive_hysteresis_ratio`

### Step 4: Fine-Tune for Specific Scenarios

**Tunnel navigation:**
```yaml
adaptive_fov_threshold: 3000   # Lower for sparse environment
adaptive_stability_frames: 5    # More stable switching
```

**Open pit mining:**
```yaml
adaptive_fov_threshold: 7000   # Higher for open areas
adaptive_stability_frames: 2    # Faster adaptation
```

## Implementation Details

### Feature Density Calculation

```cpp
density = point_count / scan_volume
```

Where `scan_volume` is estimated from:
- Detection range
- FOV coverage
- Sensor geometry

### Decision Logic (Pseudocode)

```python
def determine_strategy(point_count, scan_volume):
    density = point_count / scan_volume

    should_use_async = (
        point_count >= fov_threshold AND
        density >= feature_density_threshold
    )

    if should_use_async:
        frames_above_threshold += 1
        frames_below_threshold = 0
    else:
        frames_below_threshold += 1
        frames_above_threshold = 0

    # Hysteresis: require stability before switching
    if frames_above_threshold >= stability_frames:
        switch_to_async()
    elif frames_below_threshold >= stability_frames:
        switch_to_bundle()
```

## Troubleshooting

### Problem: Always in BUNDLE mode

**Possible causes:**
- `adaptive_fov_threshold` too high
- `adaptive_feature_density` too high
- Sparse environment

**Solutions:**
- Lower thresholds by 20-30%
- Check point cloud density: `ros2 topic echo /cloud_registered --field data | wc -l`
- Consider if BUNDLE mode is actually appropriate for your environment

### Problem: Always in ASYNC mode

**Possible causes:**
- `adaptive_fov_threshold` too low
- Dense environment (good!)
- Thresholds not matching environment

**Solutions:**
- If performance is good, no action needed
- Increase thresholds if you want more BUNDLE usage
- Monitor for divergence in sparse areas

### Problem: Rapid switching

**Possible causes:**
- `adaptive_stability_frames` too low
- Threshold set near average point count
- Highly variable environment

**Solutions:**
- Increase `adaptive_stability_frames` to 5-10
- Adjust thresholds away from average
- Consider pure BUNDLE or ASYNC mode

## Best Practices

1. **Start with Adaptive mode** for most applications
2. **Monitor first**, tune second
3. **Use debug output** during initial setup
4. **Record rosbags** of typical scenarios for offline tuning
5. **Validate in different conditions**: sparse/dense, indoor/outdoor
6. **Check drift** in long-duration tests
7. **Profile CPU usage** if real-time performance is critical

## References

- Original FAST-LIO2: [https://github.com/hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)
- FAST_LIO_MULTI: [https://github.com/engcang/FAST_LIO_MULTI](https://github.com/engcang/FAST_LIO_MULTI)
- Related Paper: "FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter"

## Related Files

- Implementation: `src/adaptive_fusion.hpp`
- Main loop: `src/laserMapping.cpp`
- Configuration: `config/dual_mid360_mine.yaml`
- This guide: `doc/ADAPTIVE_MODE_GUIDE.md`
