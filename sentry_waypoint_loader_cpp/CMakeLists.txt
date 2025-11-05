cmake_minimum_required(VERSION 3.5)
project(sentry_waypoint_loader_cpp)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)  # 启用严格编译检查
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # 新增：用于上升动作的Bool消息
find_package(yaml-cpp REQUIRED)  # 查找yaml-cpp库
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)  # tf2核心库（可能需要）
find_package(lifecycle_msgs REQUIRED)

# 检查依赖是否找到
if(NOT yaml-cpp_FOUND)
  message(FATAL_ERROR "yaml-cpp library not found. Please install it.")
endif()

# 2. 定义可执行目标
add_executable(waypoint_loader_cpp
  src/waypoint_loader.cpp
  # 若有其他源文件，在此添加
)
# 3. 链接ROS 2依赖
ament_target_dependencies(waypoint_loader_cpp
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs  # 新增：链接std_msgs
  tf2_ros  # 新增：tf2_ros依赖
  tf2      # 新增：tf2核心依赖（若需要）
  lifecycle_msgs
)

# 4. 链接yaml-cpp库
target_include_directories(waypoint_loader_cpp
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(waypoint_loader_cpp
  ${YAML_CPP_LIBRARIES}
)

# 5. 安装规则
# 安装可执行文件
install(TARGETS
  waypoint_loader_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（若有）
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件（YAML航点文件等）
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 6. 测试与打包
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
