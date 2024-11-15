cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

set(node_plugins "")

include_directories(include)

add_executable(my_first_node src/my_first_node.cpp)
ament_target_dependencies(my_first_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp std_msgs)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter
    rclcpp
    std_msgs
    example_interfaces
)

add_executable(reset_counter_client src/reset_counter_client.cpp)
ament_target_dependencies(reset_counter_client
    rclcpp
    std_msgs
    example_interfaces
)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(hardware_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hardware_status_publisher rclcpp my_robot_interfaces)

add_executable(battery_node src/battery_node.cpp)
ament_target_dependencies(battery_node rclcpp my_robot_interfaces)

add_executable(led_panel_node src/led_panel_node.cpp)
ament_target_dependencies(led_panel_node rclcpp my_robot_interfaces)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller
  rclcpp
  turtlesim
  geometry_msgs
  std_srvs
)

add_executable(turtle_spawner src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner
  rclcpp
  turtlesim
  std_srvs
)

install(TARGETS
  my_first_node
  robot_news_station
  smartphone
  number_publisher
  number_counter
  reset_counter_client
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  hardware_status_publisher
  battery_node
  led_panel_node
  turtle_controller
  turtle_spawner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
