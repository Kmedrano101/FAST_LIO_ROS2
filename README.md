ROS2 Fork repo maintainer: [Kmedrano101](https://github.com/Kmedrano101)
# FastLIO2 ROS2 Jetson Development

## About the `jetson-dev` Branch

This branch is tailored for running fastlio2 on ARM CPU architectures, such as NVIDIA Jetson devices. It is optimized for headless and embedded workflows, without graphical interfaces or visualization tools (e.g., GazeboSim, RViz).

### Getting Started

#### 1. Clone the Repository

Open a terminal and navigate to your ROS2 workspace's `src` directory (e.g., `cd ~/ros2_ws/src`). Then run:

```bash
git clone -b jetson-dev https://github.com/Kmedrano101/FAST_LIO_ROS2.git
```

#### 2. Install Dependencies

From the root of your ROS2 workspace (e.g., `cd ~/ros2_ws`), update and install the required dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### 3. Build the Package

Still in your ROS2 workspace root, build the package and source the setup file:

```bash
colcon build --packages-select fast_lio_ros2
source install/setup.bash
```

> **Note:** This package is intended for use with NVIDIA Jetson platforms and assumes a ROS2 base installation. Please ensure all hardware-specific dependencies are installed as needed.

#### 4. Run the Package

After building, you can launch the PX4 drone nodes (from anywhere with the workspace sourced):

```bash
ros2 launch fast_lio_ros2 mapping.launch.py
```

This will start the PX4 ROS2 integration on your ARM-based Jetson device, ready for headless operation.