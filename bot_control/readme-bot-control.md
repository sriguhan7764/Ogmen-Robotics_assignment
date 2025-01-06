# Bot Control Package

This package contains control and sensor processing nodes for the robot.

## Package Contents

- `scripts/`: Contains the laser scan filter implementation
- `launch/`: Contains launch files for running the nodes

## Dependencies

- ROS 2 Humble
- rclcpp
- std_msgs
- geometry_msgs
- sensor_msgs
- rviz2

## Installation


# Clone the repository into your workspace
cd ~/guhan_ws/src
# Build the workspace
cd ~/your_name_ws
colcon build
source install/setup.bash


## Usage

### Launch Laser Scan Filter
ros2 launch bot_control laser_filter.launch.py


## Node Description

### Laser Scan Filter Node
- Subscribes to: `/scan`
- Publishes to: `/filtered_scan`
- Filters laser scan data to 0-120 degrees field of view
- Visualizes filtered data in RViz

## Topics

### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): Raw laser scan data

### Published Topics
- `/filtered_scan` (sensor_msgs/LaserScan): Filtered laser scan data (0-120 degrees)
