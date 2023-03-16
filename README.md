# ROS2 Unitree Go1 Nav2 Integration
This repository has code and launch files for running the Unitree Go1 in high level mode with ROS2 Humble. It also has setup for a Nav2 integration.

It is a work in progress and further developments and documentation will be coming soon.

# Use VCS Tool for dependencies:
* cd ws/
* vcs import < src/unitree_nav/nav.repos 
* cd src/rslidar_sdk_ros
* git submodule init
* git submodule update

# To send a goal:
`ros2 service call /unitree_nav_to_pose unitree_nav_interfaces/srv/NavToPose "x: 0.0 y: 0.0 theta: 0.0"`
