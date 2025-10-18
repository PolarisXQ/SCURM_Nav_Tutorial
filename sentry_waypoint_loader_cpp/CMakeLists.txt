cmake_minimum_required(VERSION 3.5)
project(sentry_waypoint_loader_cpp)

# 1. 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)  # 查找yaml-cpp库

# 2. 定义可执行目标
add_executable(waypoint_loader_cpp src/waypoint_loader.cpp)

# 3. 链接ROS 2依赖（ament_target_dependencies内部使用普通签名）
ament_target_dependencies(waypoint_loader_cpp
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

# 4. 链接yaml-cpp（使用普通签名，与上面保持一致）
target_include_directories(waypoint_loader_cpp PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(waypoint_loader_cpp ${YAML_CPP_LIBRARIES})  # 去掉PRIVATE，用库变量

# 5. 安装文件
install(TARGETS
  waypoint_loader_cpp
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
