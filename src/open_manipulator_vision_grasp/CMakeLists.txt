cmake_minimum_required(VERSION 3.8)
project(open_manipulator_vision_grasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(color_grasp_moveit src/color_grasp_moveit.cpp)
target_compile_features(color_grasp_moveit PUBLIC cxx_std_17)
ament_target_dependencies(color_grasp_moveit
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  std_srvs
)

install(TARGETS color_grasp_moveit
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/detect_color_object.py
  scripts/detect_depth_object.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
