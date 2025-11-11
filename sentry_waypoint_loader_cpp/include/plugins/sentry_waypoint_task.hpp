#ifndef SENTRY_WAYPOINT_LOADER_CPP__SENTRY_WAYPOINT_TASK_HPP_
#define SENTRY_WAYPOINT_LOADER_CPP__SENTRY_WAYPOINT_TASK_HPP_
#pragma once

#include <map> 
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "/home/sentry_ws/src/sentry_waypoint_loader_cpp/include/common_structs.hpp"

namespace sentry_waypoint_loader_cpp
{
/**
 * @brief 自定义航点任务插件，根据YAML中的task执行上升/降落操作
 */
class SentryWaypointTask : public nav2_core::WaypointTaskExecutor
{
public:
  /**
   * @brief 构造函数
   */
  SentryWaypointTask();

  /**
   * @brief 析构函数
   */
  ~SentryWaypointTask();

  /**
   * @brief 初始化插件（读取参数、创建发布/订阅器）
   * @param parent 父节点（waypoint_follower）
   * @param plugin_name 插件名称（在参数中配置）
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  /**
   * @brief 到达航点时执行任务（核心接口）
   * @param curr_pose 当前航点坐标
   * @param curr_waypoint_index 当前航点索引
   * @return 任务是否成功执行
   */
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const int & curr_waypoint_index) override;

  /**
   * @brief 供主程序调用：设置航点ID与任务的映射关系
   * @param task_map 航点ID→任务信息的映射
   */
  void setTaskMap(const std::map<std::string, WaypointTaskInfo> & task_map)
  {
    task_map_ = task_map;
  }

  /**
   * @brief 供主程序调用：设置通过索引查询航点ID的函数
   * @param func 主程序中实现的索引→ID映射函数
   */
  void setGetWpIdFunc(std::function<std::string(int)> func)
  {
    get_wp_id_by_index_ = func;
  }

private:
  /**
   * @brief 高度订阅回调函数（更新当前Z轴高度）
   */
  void heightCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief 任务实现：上升200mm
   */
  bool executeAscend200mm();

  /**
   * @brief 任务实现：上升400mm
   */
  bool executeAscend400mm();

  /**
   * @brief 任务实现：延时降落200mm
   */
  bool executeDelayedDescend200mm();

  /**
   * @brief 任务实现：延时降落400mm
   */
  bool executeDelayedDescend400mm();

  // 节点与日志相关
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;  // 父节点指针           
  rclcpp::Clock::SharedPtr clock_;                   // 时钟（用于延时）
  bool is_enabled_;                                  // 插件是否启用
  bool has_received_z_ = false;

  // 发布/订阅器
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;  // 速度发布器（控制高度）
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr height_sub_;      // 高度订阅器

  // 状态变量
  double current_z_;  // 当前Z轴高度（米）

  // 从主程序传递的数据
  std::map<std::string, WaypointTaskInfo> task_map_;  // 航点ID→任务映射
  std::function<std::string(int)> get_wp_id_by_index_;          // 索引→航点ID的映射函数
};

}  // namespace sentry_waypoint_loader_cpp

#endif  // SENTRY_WAYPOINT_LOADER_CPP__SENTRY_WAYPOINT_TASK_HPP_
