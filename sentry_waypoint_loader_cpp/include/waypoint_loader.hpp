#ifndef SENTRY_WAYPOINT_LOADER_CPP_WAYPOINT_LOADER_HPP_
#define SENTRY_WAYPOINT_LOADER_CPP_WAYPOINT_LOADER_HPP_

#include <vector>
#include <string>
#include <map>
#include <mutex>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"  // 引入Nav2的FollowWaypoints Action

namespace sentry_waypoint_loader_cpp {

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

enum class WaypointRelation {
    RELATION_PLUS_3,
    RELATION_MINUS_3,
    RELATION_PLUS_1,
    RELATION_MINUS_1,
    RELATION_ERROR
};

// 继承rclcpp::Node，使其成为ROS 2节点类
class WaypointLoader : public rclcpp::Node {
public:
    // 构造函数：初始化节点名称和参数
    explicit WaypointLoader(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~WaypointLoader() override = default;

private:
    // 1. 核心成员变量
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_action_client_;  // Action客户端
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;     // 速度发布者
    rclcpp::TimerBase::SharedPtr start_timer_;                                // 启动延迟定时器

    std::map<std::string, geometry_msgs::msg::PoseStamped> all_wp_map_;       // 所有航点（主+过渡）
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;                  // 主航点列表
    std::vector<geometry_msgs::msg::PoseStamped> full_waypoints_;             // 完整路径（主+过渡）

    // 状态变量
    std::mutex waypoint_mutex_;
    bool rise_triggered_ = false;
    int last_completed_waypoint_ = -1;
    int rise_trigger_wp_index_;  // 触发上升动作的航点索引
    double start_delay_;         // 启动延迟时间
    std::string waypoints_path_; // 航点文件路径

    // 2. 航点解析与路径构建
    bool parse_all_waypoints_from_yaml();  // 解析所有航点（主+过渡）
    bool build_full_waypath();             // 构建完整路径
    WaypointRelation judge_waypoint_relation(int start_id, int end_id);  // 判断航点关系
    bool get_transition_points(int start_id, int end_id, 
        geometry_msgs::msg::PoseStamped& trans1, geometry_msgs::msg::PoseStamped& trans2);  // 获取过渡点

    // 3. 启动与导航逻辑
    void on_start_timer();  // 启动延迟定时器回调
    bool wait_for_action_server_with_timeout();  // 等待Action服务器
    void execute_post_waypoint_actions(int current_wp);  // 执行航点到达后的动作（上升）

    // 4. Action回调
    void goal_response_callback(const GoalHandleFollow::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleFollow::SharedPtr, 
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void result_callback(const GoalHandleFollow::WrappedResult& result);
};

}  // namespace sentry_waypoint_loader_cpp

#endif  // SENTRY_WAYPOINT_LOADER_CPP_WAYPOINT_LOADER_HPP_
