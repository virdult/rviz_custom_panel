# ITUSCT RViz2 Dashboard

A custom set of RViz2 panels built with C++ and Qt5 for real-time monitoring of an autonomous vehicle's telemetry, localization health, and high-frequency sensor drivers. 

Designed specifically to handle heavy ROS 2 data without causing RViz GUI thread throttling or DDS packet drops.

## Features

This package provides three distinct RViz plugins:

### 1. Vehicle Metrics Panel
Monitors the core physical state of the vehicle.
* **Drivetrain:** Live speedometer dial.
* **Power System:** Battery voltage and percentage with color-coded safety thresholds.
* **Status:** Displays current autonomous planner state (Centerline, Lattice, A-Star, Parking) or Manual Control overrides.

### 2. Vehicle Extras Panel
Visualizes vehicle kinematics and localization confidence.
* **Steering Visualization:** Real-time, anti-aliased rendering of the physical steering wheel angle.
* **GNSS Standard Deviation:** Calculates and displays the Standard Deviation (in mm) for Front and Rear GPS units based on real-time covariance matrices.

### 3. Driver / Topic Health Panel
An advanced, multi-threaded topic frequency monitor. 
* **Zero-Copy Architecture:** Uses `rclcpp::GenericSubscription` and an isolated `rclcpp::Context` with a dedicated `MultiThreadedExecutor`. This allows monitoring of massive payloads (like 5MB Ouster point clouds) and high-frequency data (200Hz IMUs) with ~0% CPU overhead, completely bypassing RViz's 30 FPS rendering loop.
* **EMA Smoothing:** Uses an Exponential Moving Average to display rock-solid Hz values, matching native `ros2 topic hz` output without UI flickering.
* **Status Colors:** Dynamically changes from Green (Healthy) to Orange (Warning) to Red (Critical/Dead) based on predefined Hz thresholds.

## Prerequisites

* **ROS 2** (Tested on Humble)
* **Qt5** (`qtbase5-dev`)
* Custom vehicle messages (`gae_msgs`)
* Standard ROS 2 messages (`std_msgs`, `sensor_msgs`, `geometry_msgs`)

## Installation & Build

Clone this repository into the `src` folder of your ROS 2 workspace:

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/virdult/rviz_custom_panel.git
cd ..
```

Install dependencies and build:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rviz_custom_panel
```

## Usage

1. Source your workspace: `source install/setup.bash`
2. Launch RViz2: `rviz2`
3. In the top menu bar, go to **Panels > Add New Panel**.
4. Expand the `rviz_custom_panel` folder and add:
   * `VehicleMetricsPanel`
   * `VehicleExtrasPanel`
   * `VehicleDriversPanel`

## Maintainer
* **virdult** - [virdult@gmail.com](mailto:virdult@gmail.com)