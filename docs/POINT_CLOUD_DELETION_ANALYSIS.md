# Point Cloud Deletion Analysis When Stationary

## Issue Description

When the robot is moving, point clouds are saved properly. However, when the robot stops (takes a break), parts of the map that are not being actively scanned start losing points. When analyzing the final map with CloudCompare, the last scanned area maintains better density while older areas show reduced point density.

---

## Root Cause Summary

| Mechanism | What Happens | Impact |
|-----------|--------------|--------|
| **FOV Cube Culling** | Deletes points beyond sliding window when robot moves | Permanent loss of distant past scans |
| **IKD-Tree Downsampling** | When re-scanning same area, collapses multiple points per grid cell to 1 point | **Main cause** of density loss when stationary |
| **Map Incremental Logic** | Rejects new points if old points are "close enough" to grid center | Prevents replacing degraded areas |

---

## Detailed Technical Analysis

### 1. FOV-Based Map Culling (`lasermap_fov_segment()`)

**Location:** `src/laserMapping.cpp:325-371`

**Purpose:** Maintains a sliding cube of map points around the robot's current position.

#### Key Parameters:
```cpp
float DET_RANGE = 300.0f;                    // Detection range
const float MOV_THRESHOLD = 1.5f;            // Movement trigger threshold
double cube_len = 200.0 (default config)     // Local map cube side length
```

#### Critical Logic:
```cpp
// Check if robot has approached map edge
if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
    dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
    need_move = true;
```

When the robot moves and triggers FOV shift, boxes from the trailing edge are queued for deletion:
```cpp
if(cub_needrm.size() > 0)
    kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
```

**Result:** Every time the robot moves significantly, it deletes the trailing edge of the map cube.

---

### 2. IKD-Tree Downsampling - The Primary Culprit

**Location:** `include/ikd-Tree/ikd_Tree.cpp:478-547`

This is the **PRIMARY cause** of point loss during stationary operation.

#### The Downsampling Process:

When `Add_Points()` is called with `downsample_on=true`:

```cpp
int KD_TREE<PointType>::Add_Points(PointVector &PointToAdd, bool downsample_on) {
    for (int i = 0; i < PointToAdd.size(); i++) {
        if (downsample_switch) {
            // 1. Create a grid cell box for this point
            // 2. Calculate grid cell center
            // 3. FIND ALL EXISTING POINTS IN THIS GRID CELL
            Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);

            // 4. Find which point is closest to cell center
            for (int index = 0; index < Downsample_Storage.size(); index++) {
                tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
                if (tmp_dist < min_dist) {
                    min_dist = tmp_dist;
                    downsample_result = Downsample_Storage[index];
                }
            }

            // 5. THE CRITICAL DECISION:
            if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)) {
                if (Downsample_Storage.size() > 0)
                    Delete_by_range(&Root_Node, Box_of_Point, true, true);  // DELETE ALL IN CELL!
                Add_by_point(&Root_Node, downsample_result, true, ...);     // Re-add only best one
            }
        }
    }
}
```

#### What Happens During Stationary Re-scanning:

```
Grid Cell [0.5m x 0.5m x 0.5m] contains:
  Before: [Point_A, Point_B, Point_C] (from previous scans)

New scan arrives -> Point_D enters same grid cell

IKD-Tree Downsampling:
  Step 1: Find closest to center -> downsample_result = Point_B
  Step 2: Downsample_Storage.size() = 3 > 1? YES!
  Step 3: Delete_by_range() -> DELETES all 3 points
  Step 4: Add_by_point() -> RE-ADDS only Point_B

RESULT: Points A and C are GONE! Only one point survives per cell!
```

---

### 3. Map Incremental Point Addition (`map_incremental()`)

**Location:** `src/laserMapping.cpp:940-987`

The revisit logic compares new points against existing points:

```cpp
for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
    if (points_near.size() < NUM_MATCH_POINTS) break;
    if (calc_dist(points_near[readd_i], mid_point) < dist) {
        need_add = false;  // REJECT the new point!
        break;
    }
}
```

Combined with IKD-Tree downsampling, this creates a **point erosion pattern**.

---

## Why Last-Scanned Area Stays Dense

1. **Fresh scans** = few existing points per cell -> mild downsampling
2. **Old scans** = many points per cell (from multiple passes) -> aggressive downsampling
3. **Result**: Newest area keeps 5-10 points/cell, oldest areas collapse to 1 point/cell

---

## Scenario Timeline

```
T=0: Robot at origin (0,0,0)
     - Map cube initialized
     - Points added, density: HIGH

T=100s: Robot moves to (150, 0, 0)
     - FOV shift triggered
     - Old trailing edge DELETED
     - Points from x in [0, 50] are GONE (irrecoverable)

T=100-200s: Robot STATIONARY at (150, 0, 0)
     - Re-scans same area repeatedly
     - ikdtree.Add_Points(PointToAdd, true) -> downsample_on=TRUE
     - For each grid cell with multiple existing points + new points:
       * All old points in cell are DELETED
       * Only 1 "best" point is RE-ADDED
     - Result: Point density DROPS in old scanned area

T=200+: Process repeats
     - ONLY most recent scan has HIGH density
     - Older scans have LOW density (1 point per grid cell)
```

---

## Recommended Fixes

### Fix 1: Disable Downsampling When Stationary (RECOMMENDED)

**File:** `src/laserMapping.cpp` around line 983

```cpp
// Current (problematic):
add_point_size = ikdtree.Add_Points(PointToAdd, true);  // Always downsample

// Fixed - detect if robot is moving:
V3D velocity = state_point.vel;
bool is_moving = velocity.norm() > 0.05;  // 5 cm/s threshold
add_point_size = ikdtree.Add_Points(PointToAdd, is_moving);  // Only downsample when moving
ikdtree.Add_Points(PointNoNeedDownsample, false);
```

**Effect:** Prevents point erosion during stationary re-scans while maintaining downsampling efficiency during exploration.

---

### Fix 2: Increase Map Cube Size

**File:** `config/fastlio.yaml`

```yaml
mapping:
    cube_side_length: 1000.0    # Increase from 200m to 1000m
    det_range: 100.0            # Keep same
```

**Effect:** Maintains historical data for much longer before FOV culling deletes it. With 1000m cube, provides ~850m lookback instead of ~50m.

---

### Fix 3: Use Selective Downsampling in IKD-Tree

**File:** `include/ikd-Tree/ikd_Tree.cpp:515-521`

```cpp
// Current (collapses to single point):
if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)) {
    if (Downsample_Storage.size() > 0)
        Delete_by_range(&Root_Node, Box_of_Point, true, true);
    Add_by_point(&Root_Node, downsample_result, true, Root_Node->division_axis);
}

// Fixed (add stationary mode check):
if (Downsample_Storage.size() > 1 && !same_point(PointToAdd[i], downsample_result)) {
    if (Downsample_Storage.size() > 0 && !is_stationary_mode)  // Skip deletion when stationary
        Delete_by_range(&Root_Node, Box_of_Point, true, true);
    Add_by_point(&Root_Node, downsample_result, true, Root_Node->division_axis);
}
```

---

### Fix 4: Tune Filter Parameters

**File:** `config/fastlio.yaml`

```yaml
filter_size_map: 0.3      # Reduce from 0.5 for finer resolution
filter_size_surf: 0.3     # Input downsampling

mapping:
    det_range: 150.0      # Increase to reduce FOV trigger frequency
```

---

## Verification Steps

Add this logging to confirm the issue:

```cpp
// In map_incremental():
RCLCPP_INFO_THROTTLE(this->get_logger(), *log_clock, 2000,
    "Map points: %d, Added: %d, Downsample: true, Robot vel: %.3f m/s",
    ikdtree.validnum(), add_point_size, state_point.vel.norm());

// In lasermap_fov_segment():
RCLCPP_INFO_THROTTLE(this->get_logger(), *log_clock, 2000,
    "Map shift needed: %s, Deleting %zu boxes, Delete counter: %d",
    (need_move ? "true" : "false"), cub_needrm.size(), kdtree_delete_counter);
```

If you see `Robot vel: 0.00 m/s` while points are still being modified, that confirms the stationary downsampling issue.

---

## Parameter Reference

| Parameter | Default | Description | Impact on Issue |
|-----------|---------|-------------|-----------------|
| `cube_side_length` | 200m | Local map cube size | Larger = more history retained |
| `MOV_THRESHOLD` | 1.5 | Movement trigger multiplier | Lower = more frequent shifts |
| `DET_RANGE` | 100-300m | Detection range | Affects cube shift calculation |
| `filter_size_map` | 0.5m | Grid cell size for map | Smaller = finer resolution |
| `filter_size_surf` | 0.5m | Input preprocessing grid | Affects initial point count |
| `downsample_size` | 0.2m | IKD-Tree grid size | Mismatch with filter_size_map causes issues |

---

## Conclusion

The point deletion issue is caused by a combination of:

1. **FOV-based culling** removing old scans beyond the active cube
2. **Aggressive downsampling** collapsing multiple points into single representatives during re-scans
3. **Grid granularity mismatch** between different system layers

**Primary recommendation:** Implement Fix 1 (conditional downsampling based on robot velocity) combined with Fix 2 (increased map cube size) to maintain efficiency during active mapping while preserving point density during stationary validation scans.

---

## Related Issues

- Issue #52: Single Mutex for Multiple Resources (fixed)
- Issue #53: Unprotected Global State (fixed)

---

*Document created: January 2026*
*Analysis performed on: fast_lio_ros2 jetson-dev branch*
