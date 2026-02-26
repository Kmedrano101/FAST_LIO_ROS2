# FAST-LIVO2 Point Cloud Colorization Analysis

Analysis of [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) — how it colorizes point clouds and the camera hardware used.

## How FAST-LIVO2 Colorizes Point Clouds

The colorization is a **LiDAR-to-camera projection pipeline** with bilinear interpolation. The process runs during VIO (Visual-Inertial Odometry) processing and publishes `PointCloudXYZRGB` messages.

### Step 1: LiDAR-Camera Extrinsic Calibration

Each 3D LiDAR point is transformed from world coordinates into the camera frame using calibrated extrinsic parameters (rotation + translation between LiDAR and camera). Calibration is done with the **FAST-Calib** toolkit and stored in YAML config files.

### Step 2: Projection onto the Image Plane

```cpp
V3D pf(vio_manager->new_frame_->w2f(p_w));  // world -> camera frame
if (pf[2] < 0) continue;                     // skip points behind camera
V2D pc(vio_manager->new_frame_->w2c(p_w));   // world -> pixel coords
```

Each 3D point is projected to 2D pixel coordinates using a **pinhole camera model** (`fx, fy, cx, cy`). Points behind the camera (`z < 0`) or outside the image frame are discarded.

### Step 3: Visibility Check

```cpp
if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3))
```

Only points that project within the camera frame boundaries (with a 3-pixel margin) are colorized. This prevents out-of-bounds access on the image.

### Step 4: Sub-pixel Bilinear Interpolation

```cpp
V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
```

Rather than taking the nearest pixel, the system uses **bilinear interpolation** across 4 neighboring pixels for sub-pixel accuracy. The weighting is based on fractional pixel position:

```cpp
const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
// ... (4 corner weights computed)
```

This avoids aliasing artifacts and produces smoother color transitions.

### Step 5: BGR to RGB Assignment

```cpp
pointRGB.r = pixel[2];  // OpenCV stores BGR
pointRGB.g = pixel[1];
pointRGB.b = pixel[0];
```

OpenCV images are stored in BGR order, so the channels are swapped when assigned to the PCL `PointCloudXYZRGB` structure.

### Step 6: Depth Filtering

```cpp
if (pf.norm() > blind_rgb_points)
    laserCloudWorldRGB->push_back(pointRGB);
```

Points too close to the sensor (within `blind_rgb_points` distance) are excluded to avoid distorted projections from parallax.

### Debug Visualization (Separate from Real Colorization)

`voxel_map.cpp` contains a **procedural color mode** (`RGBFromVoxel()`) that assigns R/G/B based on spatial voxel index (modulo 3). This is for debug/visualization purposes only, not real camera-based colorization. There is also a jet colormap mode for visualizing plane covariance uncertainty.

## Image Acquisition Pipeline

```cpp
cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}
```

Images are received as ROS `sensor_msgs/Image` via `cv_bridge` and converted to OpenCV BGR format. The processing pipeline then:

1. Resizes the image based on `image_resize_factor`
2. Converts to grayscale for VIO tracking: `cv::cvtColor(img, img, CV_BGR2GRAY)`
3. Creates a `Frame` object with the camera model
4. Retains the original BGR image separately for colorization

## Camera Model

The system uses an **abstract camera model** (`vk::AbstractCamera`) from the Vikit library with a **pinhole camera** implementation:

```cpp
pinhole_cam = dynamic_cast<vk::PinholeCamera*>(cam);
```

Supported camera models (via config files):
- `camera_pinhole.yaml` — standard pinhole model
- `camera_fisheye_HILTI22.yaml` — fisheye distortion model
- `camera_MARS_LVIG.yaml`, `camera_NTU_VIRAL.yaml` — dataset-specific configs

Intrinsic parameters: `fx, fy, cx, cy` (focal lengths and principal point).

## Hardware: Camera and LiDAR

### Official Setup (LIV-Eye / LIV_handhold_2)

| Component | Model | Details |
|---|---|---|
| **Camera** | Hikvision MV-CU013-A0UC | USB 3.0 machine vision, 1280x720, trigger mode |
| **Lens** | Hikvision MVL-HF0628M-6MPE | 6mm focal length, 6MP |
| **LiDAR** | Livox Mid-360 | ~$560, built-in IMU |
| **Synchronizer** | Custom STM32-based PCB | ~$70, provides 10Hz trigger + 1Hz pseudo-GPS |
| **Power** | 12V DC battery | 16V input, 12V/5V regulated outputs |

### Alternative Camera Options (Community)

| Camera | Notes |
|---|---|
| Hikvision MV-CU050-90UC | Higher resolution (>1920x1080) for 3DGS applications |
| FLIR cameras | Modified FLIR driver for shared timestamps with Livox |

### Hardware Synchronization

The system requires **hardware-level time synchronization** (<20ms between LiDAR and camera):

- Custom STM32-based synchronizer PCB generates trigger signals
- Camera operates in **trigger mode** (not free-running)
- 10Hz synchronization signals + 1Hz pseudo-GPS signal
- The camera and LiDAR ROS drivers are modified to support millisecond-level hardware sync
- Synchronizer operates independently of the computing platform

## ROS2 Support Status

| Aspect | Status |
|---|---|
| **FAST-LIVO2 framework** | **ROS1 only** — uses `catkin_make`, `roslaunch`, `rosbag`, `sensor_msgs::ImageConstPtr` |
| **Camera ROS driver** | `mvs_ros_pkg` (Hikvision MVS SDK wrapper) — **ROS1 only**, no native ROS2 driver |
| **LiDAR driver** | `livox_ros_driver` (ROS1) for Avia; `livox_ros_driver2` for Mid-360 |
| **Ubuntu support** | 18.04–20.04 |

**The Hikvision cameras are industrial machine vision cameras that do not have native ROS2 drivers.** They require the Hikvision MVS SDK, wrapped by the community `mvs_ros_pkg` package for ROS1.

### ROS2-Compatible Camera Alternatives

If implementing colorization in a ROS2 pipeline, these cameras have native ROS2 support:

| Camera | ROS2 Package | Notes |
|---|---|---|
| Intel RealSense D435/D455 | `realsense-ros` | Depth + RGB, well-maintained ROS2 support |
| FLIR Blackfly/Chameleon | `flir_camera_driver` | Spinnaker SDK, hardware trigger support |
| Basler cameras | `pylon-ros-camera` | Industrial cameras with ROS2 support |
| USB/CSI cameras | `v4l2_camera` or `usb_cam` | Generic driver, works with most webcams |

## Key Dependencies

- **PCL** >= 1.8 (point cloud processing)
- **Eigen** >= 3.3.4 (linear algebra)
- **OpenCV** >= 4.2 (image processing, `cv_bridge`)
- **Sophus** (Lie group algebra for SE3 transformations)
- **Vikit** (camera model abstraction — custom catkin package)

## Relevance to FAST-LIO ROS2 Colorization

To add camera-based colorization to the existing FAST-LIO ROS2 system:

1. **The colorization algorithm is straightforward** — project LiDAR points onto the camera image plane, sample RGB with bilinear interpolation
2. **The hard part is time synchronization** — FAST-LIVO2 requires hardware-level sync for <20ms alignment; software-based `message_filters::ApproximateTimeSynchronizer` may suffice for lower-accuracy applications
3. **Required calibration**: LiDAR-camera extrinsic parameters (rotation + translation) via `FAST-Calib` or similar tools
4. **For Jetson/ROS2**: Intel RealSense or FLIR cameras with native ROS2 drivers would be the path of least resistance

## Sources

- [FAST-LIVO2 Repository](https://github.com/hku-mars/FAST-LIVO2)
- [LIV-Eye / LIV_handhold_2](https://github.com/hku-mars/LIV_handhold_2)
- [Mid-360 Reproduction Guide (Issue #120)](https://github.com/hku-mars/FAST-LIVO2/issues/120)
- [Hardware Detail (Issue #32)](https://github.com/hku-mars/FAST-LIVO2/issues/32)
- [FLIR Timestamp Issue (Issue #217)](https://github.com/hku-mars/FAST-LIVO2/issues/217)
