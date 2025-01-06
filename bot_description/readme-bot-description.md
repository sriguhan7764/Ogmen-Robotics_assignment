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
