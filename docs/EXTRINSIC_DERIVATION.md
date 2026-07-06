# Derivation of the Dual MID-360 Extrinsic Values

This document shows, step by step, the mathematical operations used to obtain every
extrinsic value in `config/navigation.yaml` / `config/reconstruction.yaml`:

```yaml
extrinsic_T_1:           [0.011, -0.06588, -0.02329]   # base_link -> L1 IMU body
extrinsic_R_1:           identity
extrinsic_T_2:           [0.011, -0.06588, -0.02329]   # base_link -> L1 IMU body
extrinsic_R_2:           identity
extrinsic_T_L2_wrt_L1:   [0.0, -0.220, 0.0]            # L2 relative to L1
extrinsic_R_L2_wrt_L1:   identity
extrinsic_T_L1_wrt_drone:[0.0, 0.0, 0.0]               # L1 IMU body -> drone base
extrinsic_R_L1_wrt_drone:identity
```

These values are **not** fitted from data (no GICP/ICP, `extrinsic_est_en: false`). They are
derived in **closed form** from (a) the mechanical design / CAD of the payload and (b) the
Livox MID-360 datasheet. The geometry is rigid, so a one-time analytical derivation is exact.

---

## 0. Conventions and inputs

### 0.1 Coordinate frames

| Frame | Meaning |
|-------|---------|
| `L_i native` | Raw optical frame of LiDAR *i* as the sensor outputs it |
| `IMU_i native` | Raw frame of the IMU built into MID-360 *i* (datasheet offset from `L_i native`) |
| `base_link` | Physical payload reference frame; the Livox driver publishes both clouds here |
| `L1 IMU body` | **FAST-LIO origin (0,0,0)** — the L1 IMU after `slam_tools` rotational alignment |
| `drone` | Platform body frame published on TF (handheld / M600 / future airframe) |

A rigid transform is written `T = [R | t]` (3×3 rotation `R`, 3×1 translation `t`).
Applying it to a point: `p' = R·p + t`.

### 0.2 Manufacturer input (MID-360 datasheet)

The IMU is offset from the LiDAR optical origin, in the **sensor's native frame**, by:

```
p_imu_native = [0.011, 0.02329, -0.04412]  m      (x, y, z)
```

### 0.3 Mechanical / CAD input (payload design)

Both sensors are mounted **rolled 90° outward** on opposite lateral faces, on a 22 cm
transverse baseline (±110 mm from the centerline along the body **y**-axis):

| Quantity | LiDAR 1 (front) | LiDAR 2 (rear) |
|----------|-----------------|----------------|
| Mount roll | `Rx(90°)` | `Rx(90°)` |
| Mount yaw | `Rz(180°)` | `Rz(0°)` |
| Position in `base_link` | `[0, +0.110, 0]` | `[0, -0.110, 0]` |

### 0.4 Elementary rotation matrices

With `c = cos θ`, `s = sin θ`:

```
        | 1  0  0 |            |  c  0  s |            | c -s  0 |
Rx(θ) = | 0  c -s |    Ry(θ) = |  0  1  0 |    Rz(θ) = | s  c  0 |
        | 0  s  c |            | -s  0  c |            | 0  0  1 |
```

For θ = 90°: c = 0, s = 1.  For θ = 180°: c = -1, s = 0.

```
          | 1  0  0 |              | -1  0  0 |
Rx(90°) = | 0  0 -1 |    Rz(180°) =|  0 -1  0 |
          | 0  1  0 |              |  0  0  1 |
```

---

## 1. `extrinsic_R_1` — rotation, LiDAR 1 → IMU body  → **identity**

FAST-LIO does **not** receive the raw MID-360 IMU. Upstream, the `slam_tools/imu_transformer`
node rotates the IMU stream into the **L1 IMU body frame** by applying

```
R_imu_body = (Rz(180°) · Rx(90°))^T
```

so that by the time the IMU reaches FAST-LIO, the IMU and LiDAR-1 already share one orientation.
The residual rotation FAST-LIO must apply is therefore the **identity**:

```
extrinsic_R_1 = | 1 0 0 |
                | 0 1 0 |
                | 0 0 1 |
```

> If the IMU were **not** pre-rotated, `extrinsic_R_1` would have to be `R_L1 = Rz(180°)·Rx(90°)`
> (computed in §3.1). We deliberately moved that rotation into `slam_tools`, leaving the
> FAST-LIO rotation clean.

---

## 2. `extrinsic_T_1` — translation, base_link → L1 IMU body → **[0.011, -0.06588, -0.02329]**

This is the only non-trivial vector. Five operations.

### 2.1 Build the L1 mount rotation (matrix × matrix)

```
R_L1 = Rz(180°) · Rx(90°)
```

Multiply (element = row of Rz · column of Rx):

```
       | -1  0  0 |   | 1  0  0 |   | -1   0   0 |
R_L1 = |  0 -1  0 | · | 0  0 -1 | = |  0   0   1 |
       |  0  0  1 |   | 0  1  0 |   |  0   1   0 |
```

Check of one entry — (row2,col3): (0)(0) + (-1)(-1) + (0)(0) = **1** ✓

### 2.2 Rotate the native IMU offset (matrix × vector)

```
p_rot = R_L1 · p_imu_native
```

```
x = (-1)(0.011) + (0)(0.02329) + (0)(-0.04412) = -0.011
y = ( 0)(0.011) + (0)(0.02329) + (1)(-0.04412) = -0.04412
z = ( 0)(0.011) + (1)(0.02329) + (0)(-0.04412) =  0.02329
```

```
p_rot = [-0.011, -0.04412, 0.02329]
```

> Physical reading: the 90° roll swaps the native Y and Z axes — native `z = -0.04412`
> becomes the new `y`, and native `y = 0.02329` becomes the new `z`.

### 2.3 Add the L1 mount translation (vector + vector), in `base_link`

L1 sits at `[0, +0.110, 0]` in `base_link`:

```
p_imu_in_base = p_rot + [0, 0.110, 0]
              = [-0.011, -0.04412 + 0.110, 0.02329]
              = [-0.011, 0.06588, 0.02329]
```

This is the **physical position of the L1 IMU inside `base_link`**.

### 2.4 Invert for the extrinsic (negate; pure translation, R = I)

FAST-LIO needs the transform that takes points **from `base_link` to the IMU body frame**.
For a pure translation that is just the negation:

```
extrinsic_T_1 = -p_imu_in_base = [0.011, -0.06588, -0.02329]   ✓
```

### 2.5 Assemble the homogeneous transform (4×4)

With R = identity (§1):

```
                | 1 0 0   0.011  |
T_base->L1imu = | 0 1 0  -0.06588|
                | 0 0 1  -0.02329|
                | 0 0 0   1      |
```

---

## 3. LiDAR 2 values

### 3.1 `extrinsic_T_2` and `extrinsic_R_2` → **same as L1**

Key point: the Livox driver publishes **both** clouds already expressed in `base_link`
(it applies each sensor's own mount R and translation internally). So once in `base_link`,
**both** clouds need the *same* `base_link → L1 IMU body` transform to reach the FAST-LIO origin:

```
extrinsic_T_2 = extrinsic_T_1 = [0.011, -0.06588, -0.02329]
extrinsic_R_2 = identity
```

(The L2-specific mount rotation `R_L2 = Rx(90°)` and its position are consumed earlier, by the
driver, when it brings L2 into `base_link` — they are not repeated here.)

### 3.2 `extrinsic_T_L2_wrt_L1` → **[0.0, -0.220, 0.0]** (subtraction of positions)

Relative translation of L2 with respect to L1, both expressed in `base_link`:

```
extrinsic_T_L2_wrt_L1 = pos(L2) - pos(L1)
                      = [0, -0.110, 0] - [0, +0.110, 0]
                      = [0, -0.220, 0]            (220 mm = 22 cm baseline) ✓
```

### 3.3 `extrinsic_R_L2_wrt_L1` → **identity**

Both clouds live in the common `base_link` orientation, so the relative rotation between
them (as used by Bundle-mode fusion) is the identity:

```
extrinsic_R_L2_wrt_L1 = I
```

> Physically the two sensors are rolled to opposite sides, but that orientation difference is
> already removed by the driver when both clouds are placed in `base_link`. What remains is the
> pure 220 mm `y` separation in §3.2.

---

## 4. L1 → drone base

### 4.1 `extrinsic_T_L1_wrt_drone` → **[0,0,0]**, `extrinsic_R_L1_wrt_drone` → **identity**

This matrix decouples the SLAM/IMU frame from the platform body frame published on TF.
In this configuration the drone body frame is **defined to coincide** with the L1 IMU body
frame (FAST-LIO origin), so the transform is the identity:

```
extrinsic_T_L1_wrt_drone = [0, 0, 0]
extrinsic_R_L1_wrt_drone = I
```

> This is the **only** matrix expected to change per airframe (handheld → M600 → future
> platforms). Setting it to identity here means "report odometry in the L1 IMU frame".
> To publish odometry in a different mechanical reference (e.g. the M600 flight-controller
> origin), measure that offset from CAD and place it here; the rest of the stack is unchanged.

---

## 5. Summary of operations

| Output value | Operations applied |
|--------------|--------------------|
| `extrinsic_R_1`, `R_2` | identity (rotation pushed into `slam_tools` upstream) |
| `extrinsic_T_1`, `T_2` | (1) `Rz(180°)·Rx(90°)` matrix×matrix → (2) R·p matrix×vector → (3) `+[0,0.110,0]` vector add → (4) negate |
| `extrinsic_T_L2_wrt_L1` | subtraction of the two mount positions in `base_link` |
| `extrinsic_R_L2_wrt_L1` | identity (common `base_link` orientation) |
| `extrinsic_T/R_L1_wrt_drone` | identity (drone frame ≡ L1 IMU body here) |

### Sanity checks
- `‖extrinsic_T_L2_wrt_L1‖ = 0.220 m` = the designed 22 cm baseline. ✓
- `extrinsic_T_1` magnitude `≈ 0.0706 m`, dominated by the `y` component (mount offset). ✓
- All rotations are valid (orthonormal, det = +1). ✓

---

## 6. Reproducible check (NumPy)

```python
import numpy as np

# inputs
p_native = np.array([0.011, 0.02329, -0.04412])   # MID-360 datasheet IMU offset
t_mount_L1 = np.array([0.0, 0.110, 0.0])          # CAD: L1 at +110 mm in base_link
t_mount_L2 = np.array([0.0, -0.110, 0.0])         # CAD: L2 at -110 mm in base_link

def Rx(d):
    r=np.deg2rad(d); c,s=np.cos(r),np.sin(r)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Rz(d):
    r=np.deg2rad(d); c,s=np.cos(r),np.sin(r)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

R_L1 = Rz(180) @ Rx(90)               # step 2.1
p_rot = R_L1 @ p_native               # step 2.2
p_in_base = p_rot + t_mount_L1        # step 2.3
extrinsic_T_1 = -p_in_base            # step 2.4
print(np.round(extrinsic_T_1, 5))     # -> [ 0.011  -0.06588 -0.02329]

extrinsic_T_L2_wrt_L1 = t_mount_L2 - t_mount_L1
print(np.round(extrinsic_T_L2_wrt_L1, 5))  # -> [ 0.    -0.22   0.  ]
```

Expected output:
```
[ 0.011  -0.06588 -0.02329]
[ 0.     -0.22     0.     ]
```

---

**Related:** see `EXTRINSIC_CALIBRATION_GUIDE.md` for the full frame hierarchy, the Livox-driver
config files (`mid360_L1.json`, `mid360_L2.json`), and the `slam_tools` IMU transformer setup.
