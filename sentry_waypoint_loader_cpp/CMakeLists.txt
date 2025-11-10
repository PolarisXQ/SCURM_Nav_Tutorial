cmake_minimum_required(VERSION 3.5)
project(sentry_waypoint_loader_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_components REQUIRED)  # 新增：添加组件依赖

add_executable(waypoint_loader src/waypoint_loader.cpp)
ament_target_dependencies(waypoint_loader
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  yaml-cpp
  rclcpp_components  # 新增：链接组件库
)
target_link_libraries(waypoint_loader yaml-cpp)  # 链接yaml库

install(TARGETS
  waypoint_loader
  DESTINATION lib/${PROJECT_NAME})

ament_package()
