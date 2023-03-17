# ROS 2 Unitree Go1 Nav2 Integration
This repository has code and launch files for running the Unitree Go1 in high level mode with ROS 2 Humble, the RoboSense RS-Helios-16P it comes with, RTAB-Map, and the Nav2 stack. 

# Dependencies:
Dependencies for this repository are listed in the `nav.repos` file. To import all dependencies, clone this repository into the `src` directory in your workspace root. Then from the workspace root directory, run the following commands:

```
vcs import < src/unitree_nav/nav.repos 
cd src/rslidar_sdk_ros2
git submodule init
git submodule update
```

# Launch Files
Use `ros2 launch unitree_nav ${launch_file_name} --show-args` to view arguments for each launch file.

- `unitree_nav.launch.py` - launches everything necessary for controlling the Unitree Go1 with the Nav2 stack, including control and mapping nodes.
- `control.launch.py` - launches everything needed for ROS 2 control of the Unitree Go1 with services and `cmd_vel` publishing.
- `mapping.launch.py` - launches RS LiDAR and RTAB-Map nodes to map with point cloud data from the RS-Helios-16P.
- `rslidar_robosense.launch.py` - launches only RTAB-Map nodes set up in a configuration that is compatiable with the RS-Helios-16P.

# Nodes
## cmd_processor
This node handles commands and converts them into `HighCmd` messages which can be read by the high level UDP node in [this repository](https://github.com/katie-hughes/unitree_ros2) and then sent to the Go1 for motion control.

It processes `geometry_msgs/Twist` messages sent on the `cmd_vel` topic to cause the Go1 to move. It also provides several services to trigger built in functions on the Go1.

### Services
- `reset_state` - resets the commanded state to `IDLE`.
- `stand_up` - commands the Go1 to stand up.
- `recover_stand` - commands the Go1 to stand up, including from a fallen state. This is preferred over the `stand_up` service.
- `lay_down` - commands the Go1 to lay down.
- `damping` - puts the Go1's motors into "damping" mode. **Use caution with this service** - do NOT call this service while the robot is standing unsupported, it will cause it to fall.
- `jump_yaw` - commands the Go1 to jump 90° counterclockwise. **Use caution with this service**.
- `beg` - commands the Go1 to jump back into a begging stance. **Use caution with this service**.
- `dance1` - commands the Go1 to perform its first dance. **Use caution with this service**.
- `dance2` - commands the Go1 to perform its second dance. **Use caution with this service**.
- `set_body_rpy` - set the values for the roll, pitch, and yaw of the body in radians. **Use caution with this service** - specifying values that are too large may make the robot fall over. Consult the suggested limits specified in [this file](unitree_nav_interfaces/srv/SetBodyRPY.srv).

## nav_to_pose
This node is an example for working with the Nav2 stack to command the Go1 to a certain pose in the map. It can be used as an example for a custom node or, for basic use, can be ran and its services used.

### Services
- `unitree_nav_to_pose` - begins action client sequence to navigate the Go1 to a specified pose.
- `unitree_cancel_nav` - cancels all navigation goals.
