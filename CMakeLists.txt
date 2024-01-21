cmake_minimum_required(VERSION 3.8)
project(turtlebot_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include/turtlebot_bt
)

install(DIRECTORY bt_xml DESTINATION share/${PROJECT_NAME})

add_executable(executable src/exec.cpp)

target_link_libraries(executable BT::behaviortree_cpp)

set(dependencies
rclcpp
rclcpp_action
rclcpp_lifecycle
std_msgs
geometry_msgs
nav_msgs
nav2_msgs
behaviortree_cpp_v3
std_srvs
)

ament_target_dependencies(executable ${dependencies})

install(TARGETS 
  executable
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
