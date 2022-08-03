cmake_minimum_required(VERSION 3.5)
project(robots_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

find_package(interfaces REQUIRED)
find_package(planner_interface REQUIRED)

find_package(jsoncpp REQUIRED)


add_executable(control src/control.cpp)


ament_target_dependencies(control rclcpp px4_msgs mavros mavros_msgs sensor_msgs OpenCV cv_bridge interfaces planner_interface jsoncpp)

install(TARGETS
  control
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(control
  MAVSDK::mavsdk             #All apps link against mavsdk library
  MAVSDK::mavsdk_action      # If action plugin used by app ...
  MAVSDK::mavsdk_telemetry   #If telemetry plugin used by app ...
  # ... Any other linked libraries
)

ament_package()