cmake_minimum_required(VERSION 3.8)
project(arm_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add your executable
add_executable(send_trajectory src/send_trajectory.cpp)

# Link dependencies
ament_target_dependencies(send_trajectory
  rclcpp
  std_msgs
  trajectory_msgs
  control_msgs
  sensor_msgs
)

# Install the executable
install(TARGETS
  send_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
