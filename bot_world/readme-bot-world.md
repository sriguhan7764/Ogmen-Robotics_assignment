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
