#ifndef SENTRY_WAYPOINT_LOADER_CPP_WAYPOINT_LOADER_HPP_
#define SENTRY_WAYPOINT_LOADER_CPP_WAYPOINT_LOADER_HPP_

#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <chrono> 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"  
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
// #include "nav2_msgs/msg/waypoint_task.hpp"   
#include "nav_msgs/msg/odometry.hpp"
#include "/home/sentry_ws/src/sentry_waypoint_loader_cpp/include/plugins/sentry_waypoint_task.hpp" 
#include "/home/sentry_ws/src/sentry_waypoint_loader_cpp/include/common_structs.hpp"

namespace sentry_waypoint_loader_cpp {

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

// 任务类型常量
constexpr const char* TASK_ASCEND_200MM = "ascend_200mm";
constexpr const char* TASK_ASCEND_400MM = "ascend_400mm";
constexpr const char* TASK_DELAY_DESCEND_200MM = "delay_descend_200mm";
constexpr const char* TASK_DELAY_DESCEND_400MM = "delay_descend_400mm";

class SimpleWaypointTaskExecutor {
public:
    using TaskFn = std::function<bool()>;

    // 注册任务：name -> 可执行函数
    void registerTask(const std::string& name, TaskFn fn) {
        registry_[name] = fn;
    }

    // 以 nav2_msgs::msg::WaypointTask 格式执行（返回 true/false）
    bool executeTask(const std::string& task_type) {
        auto it = registry_.find(task_type);
        if (it == registry_.end()) {
            return false;
        }
        // 调用注册函数
        try {
            return it->second();
        } catch (...) {
            return false;
        }
    }

private:
    std::unordered_map<std::string, TaskFn> registry_;
};

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
    std::map<std::string, WaypointTaskInfo> wp_task_map_;  // 航点ID → 任务信息

    // 核心成员变量
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_action_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;  // 高度控制发布器
    rclcpp::TimerBase::SharedPtr start_timer_;

    // 任务执行相关组件
    std::shared_ptr<sentry_waypoint_loader_cpp::SentryWaypointTask> task_plugin_;
    std::shared_ptr<SimpleWaypointTaskExecutor> task_executor_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr height_sub_;  // 订阅里程计（Odometry）
    double current_z_ = 0.0;  // 当前高度

    std::map<std::string, geometry_msgs::msg::PoseStamped> all_wp_map_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    std::vector<geometry_msgs::msg::PoseStamped> full_waypoints_;
    std::vector<std::string> prepoints_;  // 存储航点ID列表
    std::vector<int> main_waypoint_ids_;  // 存储主航点ID

    // 状态变量
    std::mutex waypoint_mutex_;
    uint32_t last_processed_waypoint_ = UINT_MAX;
    // uint32_t last_processed_waypoint_ = 0;
    bool rise_triggered_ = false;
    double start_delay_;
    std::string waypoints_path_;

    // 初始化函数
    void init_parameters();  // 初始化参数
    void init_components();  // 初始化组件（包含任务执行器初始化）
    void send_nav_goal();    // 发送导航目标

    // // 过渡点动作函数
    // void execute_transition1_action();  // Z轴上升动作
    // void execute_transition2_action();  // 延迟动作

    // 任务执行函数
    bool execute_ascend_200mm();
    bool execute_ascend_400mm();
    bool execute_delay_descend_200mm();
    bool execute_delay_descend_400mm();

    // 回调函数
    void height_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void on_start_timer();
    bool wait_for_action_server_with_timeout(const std::chrono::seconds& timeout);
    void goal_response_callback(const GoalHandleFollow::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleFollow::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void result_callback(const GoalHandleFollow::WrappedResult& result);

    // 航点处理函数
    bool parse_all_waypoints_from_yaml();
    bool build_full_waypath();
    WaypointRelation judge_waypoint_relation(int start_id, int end_id);
    bool get_transition_points(int start_id, int end_id, geometry_msgs::msg::PoseStamped& trans1, geometry_msgs::msg::PoseStamped& trans2);
    std::string get_wp_id_by_index(uint32_t index);

    // 未使用函数（保留声明，避免编译错误）
    void execute_post_waypoint_actions(int current_wp);
};

}  // namespace sentry_waypoint_loader_cpp

#endif  // SENTRY_WAYPOINT_LOADER_CPP_WAYPOINT_LOADER_HPP_
