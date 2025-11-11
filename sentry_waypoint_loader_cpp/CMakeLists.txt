cmake_minimum_required(VERSION 3.5)
project(sentry_waypoint_loader_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # 添加调试编译选项（-g生成调试符号，-O0关闭优化）
  add_compile_options(-g -O0)
endif()

set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)  # 插件需要的生命周期节点依赖
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_core REQUIRED)  # Nav2插件核心接口
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)  # 插件加载必需

add_library(
  sentry_waypoint_task_plugin SHARED
  ./plugins/sentry_waypoint_task.cpp
)

ament_target_dependencies(sentry_waypoint_task_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  pluginlib
  geometry_msgs
  nav_msgs
)

add_executable(waypoint_loader src/waypoint_loader.cpp)
ament_target_dependencies(waypoint_loader
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav2_waypoint_follower
  tf2
  yaml-cpp
  rclcpp_components 
)
target_link_libraries(waypoint_loader
  yaml-cpp
  sentry_waypoint_task_plugin  # 链接插件库（若主程序中直接使用插件类）
)

install(TARGETS
  sentry_waypoint_task_plugin
  waypoint_loader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(FILES ./plugin_description/sentry_waypoint_task_plugin.xml
  DESTINATION share/nav2_core/plugins
)
pluginlib_export_plugin_description_file(nav2_core 
  ./plugin_description/sentry_waypoint_task_plugin.xml
)

ament_package()