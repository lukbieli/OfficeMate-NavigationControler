cmake_minimum_required(VERSION 3.5)
project(navigation_controler)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)  
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


add_executable(nav_ctrl nav_ctrl.cpp)
ament_target_dependencies(nav_ctrl
  "rclcpp"
  "rclcpp_action"
  "example_interfaces"
  "nav2_msgs"
  "rosgraph_msgs"
  "geometry_msgs"
  "std_msgs")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  nav_ctrl
  DESTINATION lib/${PROJECT_NAME})

ament_package()
