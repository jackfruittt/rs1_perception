# RS1 Perception Package

A customized ROS2 perception package for drone swarm scenario detection using AprilTags. This package extends the original AprilTag detector to support multi-drone operations and emergency scenario identification.

## Overview

The RS1 Perception package is built upon Bernd Pfrommer's AprilTag detector and customized for drone swarm applications. It detects AprilTags in the gazebo environment and identifies emergency scenarios such as stranded hikers, wildfires, and debris obstructions.

## Features

- **AprilTag Detection**: Robust tag detection using MIT or UMich detector implementations
- **Scenario Classification**: Automatic identification of emergency scenarios based on tag IDs
- **Multi-Drone Support**: Architecture designed for drone swarm operations (currently supports single drone)
- **Depth Estimation**: Calculates distance to detected tags using camera focal length
- **Real-time Processing**: Live scenario detection and reporting

## Package Structure

```
rs1_perception/
├── include/
│   └── apriltag_detector/
│       ├── detector_component.hpp    # Main detector component header
│       └── detector.hpp              # Base detector interface
├── launch/
│   └── perception.launch.py          # Launch file for perception node
├── src/
│   ├── main.cpp                      # Main executable entry point
│   └── perception_node.cpp           # Core perception node implementation
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package metadata and dependencies
├── .gitignore                        # Git ignore rules
└── readme.md                         # This documentation
```

## Supported Scenarios

| Tag ID | Scenario | Description |
|--------|----------|-------------|
| 0 | STRANDED_HIKER | Person requiring rescue assistance |
| 1 | WILDFIRE | Fire detection requiring immediate response |
| 2 | DEBRIS_OBSTRUCTION | Blocked pathways or obstacles |

## Dependencies

### System Requirements
- ROS2 Humble
- OpenCV
- AprilTag libraries

### ROS2 Dependencies
Install the required AprilTag packages:
```bash
sudo apt install ros-humble-apriltag-detector ros-humble-apriltag-detector-mit ros-humble-apriltag-detector-umich
```

### Package Dependencies
- `rclcpp` & `rclcpp_components`
- `sensor_msgs` & `geometry_msgs` & `std_msgs`
- `nav_msgs`
- `apriltag_msgs`
- `cv_bridge`
- `image_transport`
- `tf2` & `tf2_geometry_msgs`
- `pluginlib`

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/your_ws/src
git clone https://github.com/jackfruittt/rs1_perception.git
```

2. Build the package:
```bash
cd ~/your_ws
colcon build --packages-select rs1_perception
source install/setup.bash
```

## Usage

### Basic Launch
```bash
ros2 launch rs1_perception perception.launch.py
```

### Launch with Custom Parameters
```bash
ros2 launch rs1_perception perception.launch.py type:=mit drone_count:=1
```

### Launch Arguments
- `type`: Detector type (`mit` or `umich`) - Default: `mit`
- `drone_count`: Number of drones in swarm - Default: `1`

## Topics

### Subscribed Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/rs1_drone_1/front/image` | `sensor_msgs/Image` | Front camera feed for tag detection |
| `/rs1_drone_1/bottom/image` | `sensor_msgs/Image` | Bottom camera feed for scenario imaging |
| `/rs1_drone_1/odom` | `nav_msgs/Odometry` | Drone position and orientation |
| `/rs1_drone_1/imu` | `sensor_msgs/Imu` | IMU data for orientation |

### Published Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/rs1_drone_1/tags` | `apriltag_msgs/AprilTagDetectionArray` | Detected AprilTags |
| `/rs1_drone_1/scenario_detection` | `std_msgs/String` | Scenario information (CSV format) |
| `/rs1_drone_1/scenario_img` | `sensor_msgs/Image` | Bottom camera view of detected scenario |

### Scenario Detection Message Format
The `/scenario_detection` topic publishes CSV-formatted strings:
```
SCENARIO_NAME,severity,x_position,y_position,z_position,yaw_angle,respond:1/0
```

Example:
```
STRANDED_HIKER,4,10.5,5.2,2.1,1.57,1
```

## Configuration

### Camera Parameters
The package uses hardcoded camera parameters based on rs1_robot drone urdf:
- **Front Camera Focal Length**: 185.7 pixels
- **Tag Size**: 1m x 1m x 1m gazebo cube (for depth calculation)

### Detector Settings
Configure detector parameters via ROS parameters:
- `tag_family`: AprilTag family (default: `tf36h11`)
- `image_transport`: Transport type (default: `raw`)
- `decimate_factor`: Image decimation factor
- `blur`: Quad sigma blur value
- `num_threads`: Number of detection threads
- `max_allowed_hamming_distance`: Maximum Hamming distance

## Testing

### Monitor Detection Output
```bash
# tag detections, this is the AprilTag Array Message from apriltag_msgs
ros2 topic echo /rs1_drone_1/tags

# Monitor scenario detection when tag is detected
ros2 topic echo /rs1_drone_1/scenario_detection

# View scenario images if applicable
ros2 run image_view image_view image:=/rs1_drone_1/scenario_img
```

### Verify Node Status
```bash
ros2 node info /detector
```

## Architecture

### Key Components
- **DetectorComponent**: Main ROS2 node handling detection and scenario classification
- **Detector Interface**: Plugin-based detector system supporting MIT/UMich implementations
- **Multi-Drone Handler**: Framework for swarm operations (expandable)

### Depth Estimation
The system calculates tag distance relative to the drone's front camera using pinhole model:
```cpp
z_depth = focal_length / (corner_y1 - corner_y4)
```

### Position Adjustment
The estimated scenario location is based on drone yaw angle to provide world coordinates relative to the drone's orientation.

## Future Enhancements

- [ ] Full multi-drone swarm support
- [ ] Scenario severity rating
- [ ] Additional scenario types (If required)

## License

Licensed under the Apache License, Version 2.0. See the original copyright notice from Bernd Pfrommer.

## Troubleshooting

### Common Issues

**No scenario message published when echoed:**
- AprilTag may not be detected by drone camera, ensure it is in view.

**Image doesn't update:** 
- Might be frozen, just relaunch the simulation + this package.

**No tag detections:**
- Simulation lighting conditions affect the detection reliability, adjust drone position to fix this.
- AprilTag may be obstructed behind objects in the environment. 

**Missing dependencies:**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Acknowledgements
- The custom code is built on top of the pre-existing AprilTag Detector ros2 humble package by Bernd Pfrommer.
- The apriltag_detector libraries are included here for detection functionality, retrieved from [apriltag_detector](https://github.com/ros-misc-utilities/apriltag_detector)
