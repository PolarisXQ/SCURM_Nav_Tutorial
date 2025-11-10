#ifndef SENTRY_WAYPOINT_LOADER_CPP_WAYPOINT_LOADER_HPP_
#define SENTRY_WAYPOINT_LOADER_CPP_WAYPOINT_LOADER_HPP_

#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <chrono> 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

namespace sentry_waypoint_loader_cpp {

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class WaypointLoader : public rclcpp::Node {
public:
    explicit WaypointLoader(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~WaypointLoader() override = default;
    // 航点关系枚举
    enum class WaypointRelation {
        RELATION_PLUS_3,
        RELATION_MINUS_3,
        RELATION_PLUS_1,
        RELATION_MINUS_1,
        RELATION_ERROR
    };

private:
    // 核心成员变量
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_action_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr start_timer_;

    std::map<std::string, geometry_msgs::msg::PoseStamped> all_wp_map_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    std::vector<geometry_msgs::msg::PoseStamped> full_waypoints_;
    std::vector<std::string> prepoints_;  // 存储航点ID列表
    std::vector<int> main_waypoint_ids_;  // 存储主航点ID
    // 状态变量
    std::mutex waypoint_mutex_;
    uint32_t last_processed_waypoint_ = UINT_MAX;
    bool rise_triggered_ = false;
    int last_completed_waypoint_ = -1;
    int rise_trigger_wp_index_;
    double start_delay_;
    std::string waypoints_path_;

    void init_parameters();  // 初始化参数
    void init_components();  // 初始化组件
    void send_nav_goal();    // 发送导航目标

    // 过渡点动作函数
    void execute_transition1_action();  // Z轴上升动作
    void execute_transition2_action();  // 延迟动作

    // 已有函数声明修正
    bool parse_all_waypoints_from_yaml();
    bool build_full_waypath();
    WaypointRelation judge_waypoint_relation(int start_id, int end_id);
    bool get_transition_points(int start_id, int end_id, 
        geometry_msgs::msg::PoseStamped& trans1, geometry_msgs::msg::PoseStamped& trans2);

    // 启动与导航逻辑
    void on_start_timer();
    // 添加超时参数
    bool wait_for_action_server_with_timeout(const std::chrono::seconds& timeout);
    void execute_post_waypoint_actions(int current_wp);  // 若未使用可删除

    // Action回调
    void goal_response_callback(const GoalHandleFollow::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleFollow::SharedPtr, 
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void result_callback(const GoalHandleFollow::WrappedResult& result);
};

}  // namespace sentry_waypoint_loader_cpp

#endif  // SENTRY_WAYPOINT_LOADER_CPP_WAYPOINT_LOADER_HPP_
