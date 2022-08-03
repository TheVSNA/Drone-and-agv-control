#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <stdint.h>
#include <cmath>
#include <chrono>
#include <thread> 
#include <mutex>
#include <iostream>
#include <fstream>
#include <string>

#include "interfaces/msg/mymsg.hpp"
#include "interfaces/msg/msg_ret.hpp"
#include "planner_interface/srv/msg_json.hpp"

using namespace px4_msgs::msg;

#define DOUBLE_MAX 1.79769e+308
#define PI 3.141592654
#define TARGET_PRECISION 0.3
#define DEBUG_MODE false
#define TEST_MODE false
#define DEMO_MODE true
#define NDRONE 2
#define NROVER 1
#define WAITTIME 500


//function to arm a drone and do a takeoff (default height 5.0m)

void takeoff_drone(int droneid,double z);


//on spot landing

void land_at_actual_location_drone(int droneid);

//move the drone to specific coordinates

void position_drone(int droneid,double x_dest,double y_dest,double z_dest,double yaw_dest);

//rotate drone on the spot

void yaw_rotation_drone(int droneid,double new_yaw);

//disarm a drone

void disarm_drone(int droneid);

//function to get an image from a drone

void get_image(int droneid);

//function to mesure the distance from ground

void get_depth(int droneid);

//function to initialize all the publishers ad subscribers

void init_all(int argc,char* argv[]);

void perform_mission_drone(int droneid);

void perform_mission_rover(int roverid);

#endif