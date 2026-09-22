cmake_minimum_required(VERSION 3.8)
project(drone_exploration)

#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 17)
#  set(CMAKE_CXX_STANDARD_REQUIRED ON)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

# create executable
add_executable(
	exploration_test_executable src/exploration_test_node.cpp
)

add_executable(
	exploration_drone_executable src/exploration_node.cpp
)

add_executable(
	laser_tf_filtered_node src/Laser_tf_filtered_node.cpp
)

# add executable
ament_target_dependencies(exploration_test_executable
	rclcpp
	std_msgs
	geometry_msgs)

#target_compile_features(exploration_drone_executable PUBLIC cxx_std_17)

ament_target_dependencies(exploration_drone_executable
	rclcpp
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs)

ament_target_dependencies(laser_tf_filtered_node
	rclcpp
 	sensor_msgs
  	tf2_ros
	tf2_sensor_msgs
  	tf2
  	message_filters
  	geometry_msgs)

#install dependencies
install(TARGETS exploration_drone_executable 
	exploration_test_executable 
	laser_tf_filtered_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
