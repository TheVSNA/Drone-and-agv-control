# Drone-and-agv-control
2 packages for ROS2 to control a group of drones and agvs (automated guided vehicle).
robots_control contains all the necessary function to move the vehicles, in particulare it is possible to observe a function for:
* takoff
* landing
* using the camera
* using depth sensor
* moving (both drone and agv)
This package is used in combo with user_interface, that is used to show to the user a map of the area to explore, with the possibility to add some points that the vehicle will visit. This package then communicates with the planning service that calculate the best path to reach every point added by the user. This paths are then sent to robots_control in form of actions that the vehicles can perform

# Requirements
* Ubuntu Linux OS
* [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html)
* [PX4-Autopilot](https://px4.io/)

# Installation
1. Install ROS2 and PX4 following their installation instructions
2. It may be neceeeary to install the following libraries (if not already installed):
  * OpenCV2 and cv_bridge (for C++)
  * Matplotlib (Python)
  * mpl_toolkits  (Python)
  * numpy (Python)
  * rclpy (Python)
  * PIL (Python)
  * requests  (Python)
3. Clone this repository
4. Follow [Planning-drones-ROS2 readme.md](https://github.com/jvj00/Planning-drones-ROS2) for a correct setup of that module
5. build all the packages using the command 
  ```
  colcon build
  ```
6. esecute
  ```
  source install/setup.bash
  ```
7. To test the program:
  * run the following inside robots_control/launch:
  ```
    ros2 launch start0.launch.py
    ros2 launch start1.launch.py
  ```
  * start the program that control the vehicles:
  ```
    ros2 run robots_control control
  ```
  * start the planning service:
  ```
    ros2 run py_planner service
  ```
  * start the user interface:
  ```
    ros2 run user_interface ui
  ```
  Setting the variable DEMO_MODE to true inside robots_control/src/control.h before starting everithing can be useful if the user just want to test the moving    functions of robots_control

  
  
  
