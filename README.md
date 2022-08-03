# Drone-and-agv-control
2 packages for ROS2 to control a group of drones and agvs (automated guided vehicle).
robots_control contains all the necessary function to move the vehicles, in particulare it is possible to observe a function for:
* takoff
* landing
* using the camera
* using depth sensor
* moving (both drone and agv)
This package is used in combo with user_interface, that is used to show to the user a map of the area to explore, with the possibility to add some points that the vehicle will visit. This package then communicates with the planning service that calculate the best path to reach every point added by the user. This paths are then sent to robots_control in form of actions that the vehicles can perform
