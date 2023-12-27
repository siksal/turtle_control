cmake_minimum_required(VERSION 3.5)
project(turtle_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(turtle_control REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MessageCounter.msg"
  "srv/Stop.srv"
  "action/Move.action"
)

# Create cpp executable
add_executable(avoid_wall src/avoid_wall.cpp)
ament_target_dependencies(avoid_wall rclcpp geometry_msgs turtlesim)

add_executable(counter src/counter.cpp)
ament_target_dependencies(counter rclcpp turtle_control)

add_executable(euclidean_distance_server src/euclidean_distance_server.cpp)
ament_target_dependencies(euclidean_distance_server rclcpp turtlesim example_interfaces)

add_executable(euclidean_distance_client src/euclidean_distance_client.cpp)
ament_target_dependencies(euclidean_distance_client rclcpp example_interfaces)

add_executable(stop_robot_server src/stop_robot_server.cpp)
ament_target_dependencies(stop_robot_server rclcpp turtle_control)

add_executable(move_linear src/move_linear.cpp)
ament_target_dependencies(move_linear rclcpp geometry_msgs)

add_executable(rotate src/rotate.cpp)
ament_target_dependencies(rotate rclcpp geometry_msgs)

add_executable(catch_turtle src/catch_turtle.cpp)
ament_target_dependencies(catch_turtle rclcpp geometry_msgs turtlesim)

add_executable(triangle src/triangle.cpp)
ament_target_dependencies(triangle rclcpp geometry_msgs turtlesim)

add_executable(move_action_server src/move_action_server.cpp)
ament_target_dependencies(move_action_server "rclcpp" "rclcpp_action" "turtle_control" "geometry_msgs")

# Install cpp executable
install(TARGETS
  avoid_wall
  counter
  euclidean_distance_server
  euclidean_distance_client
  stop_robot_server
  move_action_server
  move_linear
  rotate
  catch_turtle
  triangle
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/avoid_wall.py
  scripts/counter.py
  scripts/eds.py
  scripts/edc.py
  scripts/stop_robot_client.py
  scripts/move_action_server.py
  scripts/turtle_catcher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
 
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
