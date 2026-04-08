# Blueprint Oculus Sonar ROS2 Driver

ROS2 metapackage for the Blueprint Oculus M750d forward-looking sonar, containing:

- **oculus_interfaces** -- ROS2 message definitions (`Ping`, `OculusStatus`, etc.)
- **oculus_ros2** -- ROS2 node wrapping the [ENSTABretagneRobotics/oculus_driver](https://github.com/ENSTABretagneRobotics/oculus_driver.git) C++ library

Tested on Ubuntu 22.04 / ROS2 Humble with the M750d sonar.

For a detailed walkthrough of the data pipeline, message construction, and image processing internals, see [PIPELINE.md](PIPELINE.md).

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `oculus/ping` | `oculus_interfaces::msg::Ping` | Full ping message with raw data |
| `oculus/image` | `sensor_msgs::msg::Image` | Cartesian fan-shaped image |
| `oculus/raw_image` | `sensor_msgs::msg::Image` | Polar (bin-beam) image |
| `oculus/pointcloud` | `sensor_msgs::msg::PointCloud2` | 3D point cloud (x, y, z, intensity) |
| `oculus/pressure` | `sensor_msgs::msg::FluidPressure` | External pressure (bar) |
| `oculus/temperature` | `sensor_msgs::msg::Temperature` | External temperature (C) |
| `oculus/depth/odometry` | `nav_msgs::msg::Odometry` | Depth computed from pressure |
| `oculus/status` | `oculus_interfaces::msg::OculusStatus` | Sonar connection status |

## Sonar Configuration

Configuration flags sent to the sonar via `SonarDriver::request_ping_config()`:

| Flag | Bit | Description |
|------|-----|-------------|
| `RANGE_AS_METERS` | 0 | Range interpreted as meters (always on) |
| `DATA_DEPTH` | 1 | 8-bit vs 16-bit data |
| `SEND_GAINS` | 2 | Include per-row gain data (always on) |
| `SIMPLE_PING` | 3 | Use simple ping format (always on) |
| `GAIN_ASSIST` | 4 | Automatic gain assist |
| `NBEAMS` | 6 | 256 (off) vs 512 (on) beams |

ROS parameters (`frequency_mode`, `range`, `gain_percent`, `sound_speed`) are exposed as dynamic reconfigure parameters and synced bidirectionally with the sonar hardware.

## Dependencies

```bash
sudo apt-get install libboost-dev libboost-system-dev libboost-thread-dev ros-humble-cv-bridge
```

## Installation

```bash
cd ~/YOUR_WS/src
git clone https://github.com/BumblebeeAS/blueprint_oculus_sonar.git
cd blueprint_oculus_sonar
git checkout humble
cd ../..
colcon build --packages-select oculus_interfaces oculus_ros2
```

## Launch

```bash
ros2 launch oculus_ros2 default.launch.py
```

## Troubleshooting

The sonar has a fixed IP address which may or may not be indicated on the box. The driver broadcasts to discover the sonar's IP automatically. Ensure your system and the sonar are on the same subnet.

## Acknowledgement

Modified from [ENSTABretagneRobotics/oculus_ros2](https://github.com/ENSTABretagneRobotics/oculus_ros2). We appreciate their work.
