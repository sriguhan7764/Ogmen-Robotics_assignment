# Bot Description Package

This package contains the URDF description and launch files for a custom differential drive robot with LiDAR and camera sensors.

## Package Contents

- `urdf/`: Contains the robot's URDF description
- `launch/`: Contains launch files for different functionalities

## Dependencies

- ROS 2 Humble
- gazebo_ros_pkgs
- xacro
- robot_state_publisher
- joint_state_publisher
- rviz2
- teleop_twist_keyboard

## Installation


# Clone the repository into your workspace
cd ~/guhan_ws/src
# Build the workspace
cd ~/your_name_ws
colcon build
source install/setup.bash


## Usage

### Visualize robot in RViz

ros2 launch bot_description rviz.launch.py


### Spawn robot in Gazebo

ros2 launch bot_description spawn.launch.py


### Control robot with keyboard

ros2 launch bot_description control.launch.py


## Robot Specifications

- Base dimensions: 200x200x100mm
- Differential drive system
- 360° LiDAR sensor
- RGB camera (800x600 resolution)
- Wheel diameter: 120mm
- Wheel separation: 239mm

# Bot World Package

This package contains custom Gazebo world definitions and launch files for spawning the robot in these environments.

## Package Contents

- `worlds/`: Contains Gazebo world definitions
- `launch/`: Contains launch files for loading worlds and spawning robots

## Dependencies

- ROS 2 Humble
- gazebo_ros
- bot_description package

## Installation



# Clone the repository into your workspace
cd ~/gyhan_ws/src
# Build the workspace
cd ~/your_name_ws
colcon build
source install/setup.bash



## Usage

### Launch Custom World with Robot


ros2 launch bot_world custom_world.launch.py



## World Description

The custom world includes:
- Cafe environment
- Multiple tables and chairs
- Bookshelves
- Suitable for navigation and obstacle avoidance testing

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
