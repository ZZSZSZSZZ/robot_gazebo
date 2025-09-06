### Run the following commands to install: 

```bash

$ sudo apt-get install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gripper-controllers ros-${ROS_DISTRO}-gazebo-ros2-control
$ cd robot_ws/src
$ git clone https://github.com/ZZSZSZSZZ/robot_gazebo.git
$ cd ..
$ colcon build
$ source install/setup.bash
$ ros2 launch robot_gazebo gazebo.launch.py
```
