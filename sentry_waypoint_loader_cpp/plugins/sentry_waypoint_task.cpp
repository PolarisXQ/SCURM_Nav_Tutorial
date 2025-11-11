#include "/home/sentry_ws/src/sentry_waypoint_loader_cpp/include/plugins/sentry_waypoint_task.hpp"
#include "pluginlib/class_list_macros.hpp"  // 插件注册宏

namespace sentry_waypoint_loader_cpp
{

SentryWaypointTask::SentryWaypointTask()
: is_enabled_(true), current_z_(0.0)
{
}

SentryWaypointTask::~SentryWaypointTask()
{
}

void SentryWaypointTask::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  // 锁定父节点（waypoint_follower）
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("无法锁定父节点，插件初始化失败！");
  }
  node_ = node;
  rclcpp::Logger logger = node->get_logger();
  clock_ = node->get_clock();

  // 声明 "enabled" 参数（默认值 true）
  node->declare_parameter(
    plugin_name + ".enabled",
    rclcpp::ParameterValue(true),
    rcl_interfaces::msg::ParameterDescriptor().set__description("是否启用插件")
  );

  // 读取参数值
  node->get_parameter(plugin_name + ".enabled", is_enabled_);

  // 创建速度发布器（控制高度，与主程序一致）
  vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10); 

  // 订阅里程计（获取当前高度）
  height_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 10,
    std::bind(&SentryWaypointTask::heightCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(logger, "Sentry航点任务插件初始化完成（名称：%s）", plugin_name.c_str());
}

bool SentryWaypointTask::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const int & curr_waypoint_index)
{
  try {

  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("SentryWaypointTask"), "[processAtWaypoint]节点指针为空，跳过航点%d任务", curr_waypoint_index);
    return false;
  }
  rclcpp::Logger logger = node_->get_logger();

  // 插件未启用时直接返回成功
  if (!is_enabled_) {
    RCLCPP_DEBUG(logger, "插件未启用，跳过航点%d的任务", curr_waypoint_index);
    return true;
  }

  // 检查主程序是否传递了必要的映射函数
  if (!get_wp_id_by_index_) {
    RCLCPP_ERROR(logger, "未设置航点索引→ID的映射函数，无法执行航点%d任务", curr_waypoint_index);
    return false;
  }

  // 1. 通过索引获取当前航点的ID（如"-2_front"）
  std::string wp_id = get_wp_id_by_index_(curr_waypoint_index);
  if (wp_id.empty()) {
    RCLCPP_WARN(logger, "航点索引%d未找到对应的ID，跳过任务", curr_waypoint_index);
    return true;
  }

  // 2. 检查该航点是否有绑定任务
  if (task_map_.find(wp_id) == task_map_.end()) {
    RCLCPP_DEBUG(logger, "航点%s无绑定任务，继续导航", wp_id.c_str());
    return true;
  }

  // 3. 解析任务信息
  WaypointTaskInfo task = task_map_.at(wp_id);
  RCLCPP_INFO(
    logger, "航点%s触发任务：action=%s，高度=%dmm",
    wp_id.c_str(), task.action.c_str(), task.height_mm
  );

  // 检查是否收到过Z轴数据
  if (!has_received_z_) {
    RCLCPP_ERROR(logger, "ERROR：从未收到过Z轴高度数据！请检查/state_estimation话题");
    return false;  // 没收到数据，直接终止任务
  }

  // 检查current_z_是否有效
  if (std::isnan(current_z_) || std::isinf(current_z_)) {
    RCLCPP_ERROR(logger, "ERROR：收到无效的Z轴高度数据（%.3fm）", current_z_);
    return false;
  }

  if (!vel_pub_) {
    RCLCPP_ERROR(logger, "速度发布器未初始化，无法执行航点%s任务", wp_id.c_str());
    return false;
  }
  if (!clock_) {
    RCLCPP_ERROR(logger, "时钟未初始化，无法执行航点%s任务", wp_id.c_str());
    return false;
  }
  // 4. 根据任务类型执行对应操作
  bool success = false;
  if (task.action == "ascend") {
    if (task.height_mm == 200) {
      success = executeAscend200mm();
    } else if (task.height_mm == 400) {
      success = executeAscend400mm();
    } else {
      RCLCPP_ERROR(logger, "未知上升高度：%dmm（航点%s）", task.height_mm, wp_id.c_str());
      return false;
    }
  } else if (task.action == "delayed_descend") {
    if (task.height_mm == 200) {
      success = executeDelayedDescend200mm();
    } else if (task.height_mm == 400) {
      success = executeDelayedDescend400mm();
    } else {
      RCLCPP_ERROR(logger, "未知降落高度：%dmm（航点%s）", task.height_mm, wp_id.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(logger, "未知任务类型：%s（航点%s）", task.action.c_str(), wp_id.c_str());
    return false;
  }

  return success;

  } catch (const std::exception& e) {  // 捕获标准异常
    RCLCPP_FATAL(node_->get_logger(), "插件执行任务异常：%s", e.what());
    return false;
  } catch (...) {  // 捕获未知异常
    RCLCPP_FATAL(node_->get_logger(), "插件执行未知异常！");
    return false;
  }
}

void SentryWaypointTask::heightCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_ERROR(rclcpp::get_logger("SentryWaypointTask"), "收到空的里程计消息！");
    return;
  }
  current_z_ = msg->pose.pose.position.z;  // 从里程计获取当前高度
  has_received_z_ = true;
  if (node_) { 
    rclcpp::Logger logger = node_->get_logger();
    RCLCPP_INFO(logger, "收到Z轴高度：%.3fm（话题：/state_estimation）", current_z_);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("SentryWaypointTask"), "收到Z轴高度：%.3fm（node_为空）", current_z_);
  }
}

bool SentryWaypointTask::executeAscend200mm()
{
  if (!node_) return false;
  rclcpp::Logger logger = node_->get_logger();
  if (!vel_pub_ || !clock_) {
    RCLCPP_ERROR(logger, "速度发布器或时钟未初始化，无法执行上升200mm任务");
    return false;
  }
  RCLCPP_INFO(logger, "开始上升200mm（当前高度：%.3fm）", current_z_);

  const double target_z = current_z_ + 0.25;  // 目标高度（当前+0.25m）
  const double tolerance = 0.1;               // 允许误差（±0.1m）
  const double speed = 0.15;                  // 上升速度（m/s）

  // 发布上升指令
  geometry_msgs::msg::Twist cmd;
  cmd.linear.z = speed; 
  vel_pub_->publish(cmd);

  rclcpp::Rate rate(10);
  auto start_time = clock_->now();
  const auto timeout = std::chrono::seconds(8);


  while (rclcpp::ok()) {
    // 超时退出
    if (clock_->now() - start_time > timeout) {
      RCLCPP_ERROR(logger, "上升200mm超时！当前%.3fm / 目标%.3fm", current_z_, target_z);
      cmd.linear.z = 0.0;
      vel_pub_->publish(cmd);
      return false;
    }
    // 达标退出
    if (current_z_ >= target_z - tolerance) {
      break;
    }
    RCLCPP_DEBUG(logger, "上升中：当前%.3fm / 目标%.3fm", current_z_, target_z);
    rate.sleep();
  }

  // 停止上升
  cmd.linear.z = 0.0;
  vel_pub_->publish(cmd);
  RCLCPP_INFO(logger, "上升200mm完成（最终高度：%.3fm）", current_z_);
  return true;
}

bool SentryWaypointTask::executeAscend400mm()
{
  if (!node_) return false;
  rclcpp::Logger logger = node_->get_logger();
  if (!vel_pub_ || !clock_) {
    RCLCPP_ERROR(logger, "速度发布器或时钟未初始化，无法执行上升400mm任务");
    return false;
  }
  RCLCPP_INFO(logger, "开始上升400mm（当前高度：%.3fm）", current_z_);

  const double target_z = current_z_ + 0.45;  // 目标高度（当前+0.45m）
  const double tolerance = 0.1;
  const double speed = 0.15;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.z = speed;
  vel_pub_->publish(cmd);

  rclcpp::Rate rate(10);
  auto start_time = clock_->now();
  const auto timeout = std::chrono::seconds(8);


  while (rclcpp::ok()) {
    // 超时退出
    if (clock_->now() - start_time > timeout) {
      RCLCPP_ERROR(logger, "上升200mm超时！当前%.3fm / 目标%.3fm", current_z_, target_z);
      cmd.linear.z = 0.0;
      vel_pub_->publish(cmd);
      return false;
    }
    // 达标退出
    if (current_z_ >= target_z - tolerance) {
      break;
    }
    RCLCPP_DEBUG(logger, "上升中：当前%.3fm / 目标%.3fm", current_z_, target_z);
    rate.sleep();
  }

  cmd.linear.z = 0.0;
  vel_pub_->publish(cmd);
  RCLCPP_INFO(logger, "上升400mm完成（最终高度：%.3fm）", current_z_);
  return true;
}

bool SentryWaypointTask::executeDelayedDescend200mm()
{
  if (!node_) return false;
  rclcpp::Logger logger = node_->get_logger();
  if (!vel_pub_ || !clock_) {
    RCLCPP_ERROR(logger, "速度发布器或时钟未初始化，无法执行延时降落200mm任务");
    return false;
  }
  RCLCPP_INFO(logger, "开始延时降落200mm（当前高度：%.3fm）", current_z_);

  const double target_z = current_z_ - 0.2;  // 目标高度（当前-0.2m）
  const double tolerance = 0.02;
  const double speed = -0.1;                 // 降落速度（负号表示向下）

  // 第一步：延时2秒
  RCLCPP_INFO(logger, "延时2秒...");
  clock_->sleep_for(std::chrono::seconds(2));

  // 第二步：发布降落指令
  geometry_msgs::msg::Twist cmd;
  cmd.linear.z = speed;
  vel_pub_->publish(cmd);

  rclcpp::Rate rate(10);
  auto start_time = clock_->now();
  const auto timeout = std::chrono::seconds(8);

  // 第三步：等待到达目标高度
  while (rclcpp::ok()) {
    // 超时退出
    if (clock_->now() - start_time > timeout) {
      RCLCPP_ERROR(logger, "上升200mm超时！当前%.3fm / 目标%.3fm", current_z_, target_z);
      cmd.linear.z = 0.0;
      vel_pub_->publish(cmd);
      return false;
    }
    // 达标退出
    if (current_z_ <= target_z + tolerance) {
      break;
    }
    RCLCPP_DEBUG(logger, "降落中：当前%.3fm / 目标%.3fm", current_z_, target_z);
    rate.sleep();
  }

  // 停止降落
  cmd.linear.z = 0.0;
  vel_pub_->publish(cmd);
  RCLCPP_INFO(logger, "延时降落200mm完成（最终高度：%.3fm）", current_z_);
  return true;
}

bool SentryWaypointTask::executeDelayedDescend400mm()
{
  if (!node_) return false;
  rclcpp::Logger logger = node_->get_logger();
  if (!vel_pub_ || !clock_) {
    RCLCPP_ERROR(logger, "速度发布器或时钟未初始化，无法执行延时降落400mm任务");
    return false;
  }
  RCLCPP_INFO(logger, "开始延时降落400mm（当前高度：%.3fm）", current_z_);

  const double target_z = current_z_ - 0.4;  // 目标高度（当前-0.4m）
  const double tolerance = 0.02;
  const double speed = -0.1;

  // 延时2秒
  RCLCPP_INFO(logger, "延时2秒...");
  clock_->sleep_for(std::chrono::seconds(2));

  // 发布降落指令
  geometry_msgs::msg::Twist cmd;
  cmd.linear.z = speed;
  vel_pub_->publish(cmd);

  rclcpp::Rate rate(10);
  auto start_time = clock_->now();
  const auto timeout = std::chrono::seconds(8);

  // 等待到达目标高度
  while (rclcpp::ok()) {
    // 超时退出
    if (clock_->now() - start_time > timeout) {
      RCLCPP_ERROR(logger, "上升200mm超时！当前%.3fm / 目标%.3fm", current_z_, target_z);
      cmd.linear.z = 0.0;
      vel_pub_->publish(cmd);
      return false;
    }
    // 达标退出
    if (current_z_ <= target_z + tolerance) {
      break;
    }
    RCLCPP_DEBUG(logger, "降落中：当前%.3fm / 目标%.3fm", current_z_, target_z);
    rate.sleep();
  }

  cmd.linear.z = 0.0;
  vel_pub_->publish(cmd);
  RCLCPP_INFO(logger, "延时降落400mm完成（最终高度：%.3fm）", current_z_);
  return true;
}

}  // namespace sentry_waypoint_loader_cpp

// 注册插件（必须，让Nav2识别）
PLUGINLIB_EXPORT_CLASS(
  sentry_waypoint_loader_cpp::SentryWaypointTask,
  nav2_core::WaypointTaskExecutor
)
