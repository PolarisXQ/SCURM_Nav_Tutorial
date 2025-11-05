#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>          
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <cmath>

using namespace std::chrono_literals;

namespace sentry_waypoint_loader_cpp {

class WaypointLoader : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  WaypointLoader(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("waypoint_loader_cpp", options),
    waypoints_file_(""),
    startup_delay_(6.0),
    rise_trigger_wp_index_(1),
    rise_trigger_distance_(0.5),
    waypoint_arrival_distance_(0.3),
    rise_triggered_(false),
    last_completed_waypoint_(-1),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_),
    wp1_main_(),  // 航点变量先初始化
    wp2_trans1_(),
    wp3_trans2_(),
    wp4_main_(),
    current_goal_handle_(nullptr),  // 再初始化 current_goal_handle_
    goal_sent_(false),
    goal_succeeded_(false),
    goal_send_failed_(false),
    goal_send_start_time_(this->now())
  {
    // 声明参数
    this->declare_parameter<std::string>(
      "waypoints_file", 
      std::string(get_home() + "/sentry_ws/src/sentry_waypoint_loader_cpp/config/waypoints.yaml"),
      rcl_interfaces::msg::ParameterDescriptor{}
        .set__description("航点YAML文件的绝对路径")
    );
    this->declare_parameter<double>("startup_delay", 6.0);
    this->declare_parameter<int>("rise_trigger_wp_index", 1);
    this->declare_parameter<double>("rise_trigger_distance", 0.5);
    this->declare_parameter<double>(
      "pre_trans2_wait_time", 3.0,
      rcl_interfaces::msg::ParameterDescriptor{}
        .set__description("发布第二个过渡航点前的停留时间")
    );
    this->declare_parameter<double>(
      "waypoint_arrival_distance", 0.3,
      rcl_interfaces::msg::ParameterDescriptor{}
        .set__description("进入此距离范围即判定为到达航点（米）")
    );

    // 获取参数
    waypoints_file_ = this->get_parameter("waypoints_file").as_string();
    startup_delay_ = this->get_parameter("startup_delay").as_double();
    rise_trigger_wp_index_ = this->get_parameter("rise_trigger_wp_index").as_int();
    rise_trigger_distance_ = this->get_parameter("rise_trigger_distance").as_double();
    pre_trans2_wait_time_ = this->get_parameter("pre_trans2_wait_time").as_double();
    waypoint_arrival_distance_ = this->get_parameter("waypoint_arrival_distance").as_double();

    RCLCPP_INFO(
      get_logger(), 
      "初始化完成：航点到达阈值=%.2fm，上升触发距离=%.2fm", 
      waypoint_arrival_distance_, rise_trigger_distance_
    );

    // 创建通信对象
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    follow_action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "/follow_waypoints");

    // 启动延迟定时器
    start_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(startup_delay_),
      [this]() {
        start_timer_->cancel();
        this->on_start_timer();
      });

    // 轮询定时器：替代临时执行器，检查目标状态
    poll_timer_ = this->create_wall_timer(
      100ms,  // 每100ms检查一次
      [this]() { this->poll_goal_status(); });
  }

private:
  // 成员变量
  std::string waypoints_file_;
  double startup_delay_;
  int rise_trigger_wp_index_;
  double rise_trigger_distance_;
  double waypoint_arrival_distance_;
  double pre_trans2_wait_time_;
  bool rise_triggered_;
  int last_completed_waypoint_;
  std::mutex waypoint_mutex_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_action_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr poll_timer_;  // 轮询定时器
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  GoalHandleFollow::SharedPtr current_goal_handle_;  // 先声明此变量

  // 航点存储
  geometry_msgs::msg::PoseStamped wp1_main_;
  geometry_msgs::msg::PoseStamped wp2_trans1_;
  geometry_msgs::msg::PoseStamped wp3_trans2_;
  geometry_msgs::msg::PoseStamped wp4_main_;

  geometry_msgs::msg::PoseStamped current_target_pose_;  // 当前目标航点
  bool goal_sent_;  // 目标是否已发送
  bool goal_succeeded_;  // 目标是否成功完成
  int current_wp_index_;  // 当前处理的航点索引
  bool goal_send_failed_;  // 标记目标发送失败
  rclcpp::Time goal_send_start_time_;  // 记录目标发送开始时间（用于超时判断）


  // 主航点关系枚举
  enum class WaypointRelation {
    RELATION_PLUS_3, RELATION_MINUS_3, RELATION_PLUS_1, RELATION_MINUS_1, RELATION_ERROR
  };

  // 获取HOME目录
  static std::string get_home()
  {
    const char * h = std::getenv("HOME");
    return h ? std::string(h) : std::string(".");
  }

  // 等待Action服务器
  bool wait_for_action_server_with_timeout(double timeout_s = 30.0)
  {
    auto t0 = this->now();
    while (!follow_action_client_->wait_for_action_server(1s)) {
      auto elapsed = (this->now() - t0).seconds();
      if (elapsed >= timeout_s) {
        RCLCPP_ERROR(get_logger(), "等待FollowWaypoints服务器超时（%ds）", static_cast<int>(timeout_s));
        return false;
      }
      RCLCPP_INFO(get_logger(), "等待FollowWaypoints服务器...（已等待%.1fs）", elapsed);
    }
    return true;
  }

  // 判断主航点关系
  WaypointRelation judge_waypoint_relation(int start_id, int end_id)
  {
    int diff = end_id - start_id;
    if (diff == 3) return WaypointRelation::RELATION_PLUS_3;
    if (diff == -3) return WaypointRelation::RELATION_MINUS_3;
    if (diff == 1) return WaypointRelation::RELATION_PLUS_1;
    if (diff == -1) return WaypointRelation::RELATION_MINUS_1;
    RCLCPP_ERROR(get_logger(), "非法主航点关系：%d→%d（仅支持±1/±3）", start_id, end_id);
    return WaypointRelation::RELATION_ERROR;
  }

  // 提取过渡节点
  std::vector<geometry_msgs::msg::PoseStamped> get_transition_points(int start_id, int end_id)
  {
    std::vector<geometry_msgs::msg::PoseStamped> transition_points;
    auto relation = judge_waypoint_relation(start_id, end_id);
    if (relation == WaypointRelation::RELATION_ERROR) return transition_points;

    std::string tp1_id, tp2_id;
    switch (relation) {
      case WaypointRelation::RELATION_PLUS_3:
        tp1_id = std::to_string(start_id) + "_front";
        tp2_id = std::to_string(end_id) + "_back";
        break;
      case WaypointRelation::RELATION_MINUS_3:
        tp1_id = std::to_string(start_id) + "_back";
        tp2_id = std::to_string(end_id) + "_front";
        break;
      case WaypointRelation::RELATION_PLUS_1:
        tp1_id = std::to_string(start_id) + "_left";
        tp2_id = std::to_string(end_id) + "_right";
        break;
      case WaypointRelation::RELATION_MINUS_1:
        tp1_id = std::to_string(start_id) + "_right";
        tp2_id = std::to_string(end_id) + "_left";
        break;
      default:
        return transition_points;
    }

    std::map<std::string, geometry_msgs::msg::PoseStamped> all_wp_map_;
    parse_all_waypoints_to_map(all_wp_map_);

    if (all_wp_map_.find(tp1_id) != all_wp_map_.end()) {
      transition_points.push_back(all_wp_map_[tp1_id]);
      RCLCPP_INFO(get_logger(), "提取过渡点1：%s（坐标：%.2f, %.2f）", 
                  tp1_id.c_str(), all_wp_map_[tp1_id].pose.position.x, all_wp_map_[tp1_id].pose.position.y);
    } else {
      RCLCPP_WARN(get_logger(), "过渡点1 %s 未找到", tp1_id.c_str());
    }
    if (all_wp_map_.find(tp2_id) != all_wp_map_.end()) {
      transition_points.push_back(all_wp_map_[tp2_id]);
      RCLCPP_INFO(get_logger(), "提取过渡点2：%s（坐标：%.2f, %.2f）", 
                  tp2_id.c_str(), all_wp_map_[tp2_id].pose.position.x, all_wp_map_[tp2_id].pose.position.y);
    } else {
      RCLCPP_WARN(get_logger(), "过渡点2 %s 未找到", tp2_id.c_str());
    }

    return transition_points;
  }

  // 解析所有航点到map
  void parse_all_waypoints_to_map(std::map<std::string, geometry_msgs::msg::PoseStamped>& all_wp_map)
  {
    try {
      if (!std::ifstream(waypoints_file_)) {
        RCLCPP_ERROR(get_logger(), "航点文件不存在：%s", waypoints_file_.c_str());
        return;
      }
      YAML::Node yaml_node = YAML::LoadFile(waypoints_file_);

      for (const auto& node : yaml_node) {
        std::string wp_id = node.first.as<std::string>();
        if (wp_id == "waypoints" || wp_id == "prepoints") continue;

        YAML::Node wp = yaml_node[wp_id];
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = wp["header"]["frame_id"].as<std::string>("map");
        pose.header.stamp = this->now();
        pose.pose.position.x = wp["pose"]["position"]["x"].as<double>(0.0);
        pose.pose.position.y = wp["pose"]["position"]["y"].as<double>(0.0);
        pose.pose.position.z = wp["pose"]["position"]["z"].as<double>(0.0);
        pose.pose.orientation.x = wp["pose"]["orientation"]["x"].as<double>(0.0);
        pose.pose.orientation.y = wp["pose"]["orientation"]["y"].as<double>(0.0);
        pose.pose.orientation.z = wp["pose"]["orientation"]["z"].as<double>(0.0);
        pose.pose.orientation.w = wp["pose"]["orientation"]["w"].as<double>(1.0);

        all_wp_map[wp_id] = pose;
      }
    } catch (const YAML::BadFile& e) {
      RCLCPP_ERROR(get_logger(), "打开YAML失败：%s", e.what());
    } catch (const YAML::ParserException& e) {
      RCLCPP_ERROR(get_logger(), "YAML格式错误：%s", e.what());
    }
  }

  // 解析4个航点并单独赋值
  bool parse_four_waypoints()
  {
    try {
      if (!std::ifstream(waypoints_file_)) {
        RCLCPP_ERROR(get_logger(), "航点文件不存在：%s", waypoints_file_.c_str());
        return false;
      }
      YAML::Node yaml_node = YAML::LoadFile(waypoints_file_);
      std::map<std::string, geometry_msgs::msg::PoseStamped> all_wp_map_;

      for (const auto& node : yaml_node) {
        std::string wp_id = node.first.as<std::string>();
        if (wp_id == "waypoints" || wp_id == "prepoints") continue;

        YAML::Node wp = yaml_node[wp_id];
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = wp["header"]["frame_id"].as<std::string>("map");
        pose.header.stamp = this->now();
        pose.pose.position.x = wp["pose"]["position"]["x"].as<double>(0.0);
        pose.pose.position.y = wp["pose"]["position"]["y"].as<double>(0.0);
        pose.pose.position.z = wp["pose"]["position"]["z"].as<double>(0.0);
        pose.pose.orientation.x = wp["pose"]["orientation"]["x"].as<double>(0.0);
        pose.pose.orientation.y = wp["pose"]["orientation"]["y"].as<double>(0.0);
        pose.pose.orientation.z = wp["pose"]["orientation"]["z"].as<double>(0.0);
        pose.pose.orientation.w = wp["pose"]["orientation"]["w"].as<double>(1.0);

        all_wp_map_[wp_id] = pose;
        RCLCPP_INFO(get_logger(), "解析航点：%s（坐标：%.2f, %.2f）", 
                    wp_id.c_str(), pose.pose.position.x, pose.pose.position.y);
      }

      if (!yaml_node["waypoints"]) {
        RCLCPP_ERROR(get_logger(), "YAML缺少'waypoints'字段");
        return false;
      }
      std::vector<std::string> main_wp_ids;
      for (const auto& wp_name_node : yaml_node["waypoints"]) {
        main_wp_ids.push_back(wp_name_node.as<std::string>());
      }
      if (main_wp_ids.size() < 2) {
        RCLCPP_ERROR(get_logger(), "主航点数量不足（至少2个）");
        return false;
      }

      int start_main_id = std::stoi(main_wp_ids[0]);
      int end_main_id = std::stoi(main_wp_ids[1]);
      auto transition_points = get_transition_points(start_main_id, end_main_id);

      // 主航点1
      if (all_wp_map_.find(main_wp_ids[0]) != all_wp_map_.end()) {
        wp1_main_ = all_wp_map_[main_wp_ids[0]];
        RCLCPP_INFO(get_logger(), "单独解析：主航点1（%s）（坐标：%.2f, %.2f）", 
                    main_wp_ids[0].c_str(), wp1_main_.pose.position.x, wp1_main_.pose.position.y);
      } else {
        RCLCPP_ERROR(get_logger(), "主航点1 %s 未找到", main_wp_ids[0].c_str());
        return false;
      }

      // 过渡点1
      if (transition_points.size() >= 1) {
        wp2_trans1_ = transition_points[0];
        RCLCPP_INFO(get_logger(), "单独解析：过渡点1（坐标：%.2f, %.2f）", 
                    wp2_trans1_.pose.position.x, wp2_trans1_.pose.position.y);
      } else {
        RCLCPP_ERROR(get_logger(), "过渡点1未找到");
        return false;
      }

      // 过渡点2
      if (transition_points.size() >= 2) {
        wp3_trans2_ = transition_points[1];
        RCLCPP_INFO(get_logger(), "单独解析：过渡点2（坐标：%.2f, %.2f）", 
                    wp3_trans2_.pose.position.x, wp3_trans2_.pose.position.y);
      } else {
        RCLCPP_ERROR(get_logger(), "过渡点2未找到");
        return false;
      }

      // 主航点2
      if (all_wp_map_.find(main_wp_ids[1]) != all_wp_map_.end()) {
        wp4_main_ = all_wp_map_[main_wp_ids[1]];
        RCLCPP_INFO(get_logger(), "单独解析：主航点2（%s）（坐标：%.2f, %.2f）", 
                    main_wp_ids[1].c_str(), wp4_main_.pose.position.x, wp4_main_.pose.position.y);
      } else {
        RCLCPP_ERROR(get_logger(), "主航点2 %s 未找到", main_wp_ids[1].c_str());
        return false;
      }

      RCLCPP_INFO(get_logger(), "4个航点单独解析完成");
      return true;

    } catch (const YAML::BadFile& e) {
      RCLCPP_ERROR(get_logger(), "打开YAML失败：%s", e.what());
      return false;
    } catch (const YAML::ParserException& e) {
      RCLCPP_ERROR(get_logger(), "YAML格式错误：%s", e.what());
      return false;
    }
  }

  // 计算两个2D点之间的直线距离
  double calculate_2d_distance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2, bool print = false)
  {
    if (print) {
      RCLCPP_INFO(get_logger(), "当前位置：(%.2f, %.2f)，目标位置：(%.2f, %.2f)",
                  pose1.pose.position.x, pose1.pose.position.y,
                  pose2.pose.position.x, pose2.pose.position.y);
    }

    if (pose1.header.frame_id != "map" || pose2.header.frame_id != "map") {
      RCLCPP_WARN(get_logger(), "距离计算坐标系错误，必须为map");
      return 1000.0;
    }
    double dx = pose2.pose.position.x - pose1.pose.position.x;
    double dy = pose2.pose.position.y - pose1.pose.position.y;
    return std::sqrt(dx*dx + dy*dy);
  }

  // 获取机器人当前位置（从TF获取map→base_link）
  bool get_current_robot_pose(geometry_msgs::msg::PoseStamped& current_pose)
  {
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = this->now();
    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero, 500ms);
      
      current_pose.pose.position.x = transform.transform.translation.x;
      current_pose.pose.position.y = transform.transform.translation.y;
      current_pose.pose.position.z = transform.transform.translation.z;
      current_pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (tf2::TransformException& ex) {
      RCLCPP_DEBUG(get_logger(), "获取机器人位置失败：%s（重试）", ex.what());
      return false;
    }
  }

  // Z轴上升动作
  void execute_post_waypoint_actions(int current_wp_index)
  {
    if (rise_triggered_) return;

    RCLCPP_INFO(get_logger(), "执行第%d个航点的Z轴上升动作", current_wp_index + 1);
    rclcpp::sleep_for(500ms);

    // 连续发布上升指令
    geometry_msgs::msg::Twist z_cmd;
    z_cmd.linear.z = 1.0;
    for (int i = 0; i < 10; ++i) {
      cmd_vel_pub_->publish(z_cmd);
      rclcpp::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(), "Z轴上升（1.0m/s，持续1秒）");

    // 连续发布停止指令
    z_cmd.linear.z = 0.0;
    for (int i = 0; i < 5; ++i) {
      cmd_vel_pub_->publish(z_cmd);
      rclcpp::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(), "Z轴停止");

    rise_triggered_ = true;
  }

  // 轮询检查目标状态（替代临时执行器的核心函数）
  void poll_goal_status()
  {
    if (!goal_sent_ || !current_goal_handle_) return;

    // 检查是否到达目标范围
    geometry_msgs::msg::PoseStamped robot_pose;
    if (get_current_robot_pose(robot_pose)) {
      double distance = calculate_2d_distance(robot_pose, current_target_pose_, true);
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                          "航点%d进度：距离目标%.2f米（到达阈值%.2f米）", 
                          current_wp_index_ + 1, distance, waypoint_arrival_distance_);

      if (distance < waypoint_arrival_distance_) {
        RCLCPP_INFO(get_logger(), "航点%d进入到达范围，主动终止导航", current_wp_index_ + 1);
        follow_action_client_->async_cancel_goal(current_goal_handle_);
        goal_succeeded_ = true;
        goal_sent_ = false;
      }
    }

    // 检查目标是否完成（服务器反馈）
    auto status = current_goal_handle_->get_status();
    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "航点%d到达完成（服务器结果码：成功）", current_wp_index_ + 1);
      goal_succeeded_ = true;
      goal_sent_ = false;
    } else if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
               status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
      RCLCPP_ERROR(get_logger(), "航点%d导航失败（状态码：%d）", current_wp_index_ + 1, status);
      goal_succeeded_ = false;
      goal_sent_ = false;
    }
  }

  // 发送航点并等待完成（基于轮询的非阻塞版本）
  bool send_waypoint_and_wait(const geometry_msgs::msg::PoseStamped& waypoint, int wp_index)
  {
    if (!wait_for_action_server_with_timeout()) {
      return false;
    }

    // 初始化状态变量
    goal_sent_ = false;
    goal_succeeded_ = false;
    goal_send_failed_ = false;
    current_wp_index_ = wp_index;
    current_target_pose_ = waypoint;
    current_goal_handle_ = nullptr;
    goal_send_start_time_ = this->now();  // 记录发送开始时间

    // 构建目标消息
    FollowWaypoints::Goal goal_msg;
    goal_msg.poses = {waypoint};

    // 发送目标（纯异步，通过回调处理结果，无 spin）
    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

    // 目标响应回调（替代 spin_until_future_complete）
    // 替换原 goal_response_callback 代码块
    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleFollow> goal_handle) {  // 修正参数类型
        try {
          if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "航点%d被服务器拒绝", current_wp_index_ + 1);
            goal_send_failed_ = true;
            goal_sent_ = false;
          } else {
            current_goal_handle_ = goal_handle;
            goal_sent_ = true;
            goal_send_failed_ = false;
          }
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "航点%d获取目标句柄失败", current_wp_index_ + 1);
          goal_send_failed_ = true;
          goal_sent_ = false;
        }
      };


    // 结果回调（不变）
    send_goal_options.result_callback =
    [this](const GoalHandleFollow::WrappedResult & result) {
      // 1. 打印结果码和描述
      std::string result_desc;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        result_desc = "成功（SUCCEEDED）";
      } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        result_desc = "中止（ABORTED）";
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        result_desc = "取消（CANCELED）";
      } else {
        result_desc = "未知（UNKNOWN）";
      }
      RCLCPP_INFO(get_logger(), "航点%d Result：%s（代码：%d）",
                  current_wp_index_ + 1, result_desc.c_str(), static_cast<int>(result.code));

      // 2. 打印 Result 中的具体数据（如 missed_waypoints）
      // FollowWaypoints 的 Result 包含“未到达的航点索引列表”
      if (!result.result->missed_waypoints.empty()) {
        RCLCPP_WARN(get_logger(), "航点%d 未到达的航点索引：", current_wp_index_ + 1);
        for (auto idx : result.result->missed_waypoints) {
          RCLCPP_WARN(get_logger(), "  - 索引 %d", idx);
        }
      } else {
        RCLCPP_INFO(get_logger(), "航点%d 所有航点均成功到达（missed_waypoints 为空）", current_wp_index_ + 1);
      }

      // 3. 更新状态变量（原有逻辑不变）
      goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      goal_sent_ = false;
    };


    // 发送目标（纯异步，无阻塞）
    follow_action_client_->async_send_goal(goal_msg, send_goal_options);

    // 等待目标发送结果/超时（通过轮询状态变量，无 spin）
    auto start_time = this->now();
    while (rclcpp::ok()) {
      // 超时检查
      if ((this->now() - start_time).seconds() > 20.0) {
        RCLCPP_ERROR(get_logger(), "航点%d发送超时", wp_index + 1);
        return false;
      }
      // 目标发送成功/失败，退出等待
      if (goal_sent_ || goal_send_failed_) {
        break;
      }
      rclcpp::sleep_for(100ms);  // 让出CPU，避免占用
    }

    // 目标发送失败，返回false
    if (goal_send_failed_) {
      return false;
    }

    // 等待航点完成（通过轮询状态变量，无 spin）
    start_time = this->now();
    while (rclcpp::ok()) {
      // 总超时检查（300秒）
      if ((this->now() - start_time).seconds() > 300.0) {
        RCLCPP_ERROR(get_logger(), "航点%d导航超时（300s）", wp_index + 1);
        if (current_goal_handle_) {
          follow_action_client_->async_cancel_goal(current_goal_handle_);
        }
        return false;
      }
      // 航点完成/失败，退出等待
      if (!goal_sent_) {
        break;
      }
      rclcpp::sleep_for(100ms);
    }

    // 补算最终距离（不变）
    geometry_msgs::msg::PoseStamped robot_pose;
    if (get_current_robot_pose(robot_pose)) {
      double final_distance = calculate_2d_distance(robot_pose, waypoint, true);
      RCLCPP_INFO(get_logger(), "航点%d完成后补算：距离目标%.2f米", 
                  wp_index + 1, final_distance);
    }

    return goal_succeeded_;
  }


  // 按顺序发布航点
  void publish_waypoints_step_by_step()
  {
    // 步骤1：发布主航点1
    RCLCPP_INFO(get_logger(), "\n===== 发布第1个航点：主航点1（到达阈值%.2fm） =====", waypoint_arrival_distance_);
    if (!send_waypoint_and_wait(wp1_main_, 0)) {
      RCLCPP_ERROR(get_logger(), "主航点1导航失败，终止流程");
      rclcpp::shutdown();
      return;
    }

    // 步骤2：发布过渡点1
    RCLCPP_INFO(get_logger(), "\n===== 发布第2个航点：过渡点1（到达阈值%.2fm） =====", waypoint_arrival_distance_);
    if (!send_waypoint_and_wait(wp2_trans1_, 1)) {
      RCLCPP_ERROR(get_logger(), "过渡点1导航失败，终止流程");
      rclcpp::shutdown();
      return;
    }

    // 过渡点1完成后停留检查
    RCLCPP_INFO(get_logger(), "\n===== 过渡点1导航完成，停留%.1f秒检查上升条件 =====", pre_trans2_wait_time_);
    auto start_wait = this->now();
    while ((this->now() - start_wait).seconds() < pre_trans2_wait_time_ && rclcpp::ok()) {
      geometry_msgs::msg::PoseStamped robot_pose;
      if (get_current_robot_pose(robot_pose)) {
        double distance = calculate_2d_distance(robot_pose, wp2_trans1_, true);
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                            "等待期间：距离过渡点1 %.2f米（阈值%.2f米）", distance, rise_trigger_distance_);
        if (!rise_triggered_ && distance < rise_trigger_distance_) {
          execute_post_waypoint_actions(1);
        }
      }
      rclcpp::sleep_for(100ms);
    }
    if (!rise_triggered_) {
      RCLCPP_WARN(get_logger(), "停留期间未满足上升条件，强制触发");
      execute_post_waypoint_actions(1);
    }

    // 步骤3：发布过渡点2
    RCLCPP_INFO(get_logger(), "\n===== 发布第3个航点：过渡点2（到达阈值%.2fm） =====", waypoint_arrival_distance_);
    if (!send_waypoint_and_wait(wp3_trans2_, 2)) {
      RCLCPP_ERROR(get_logger(), "过渡点2导航失败，终止流程");
      rclcpp::shutdown();
      return;
    }

    // 步骤4：发布主航点2
    RCLCPP_INFO(get_logger(), "\n===== 发布第4个航点：主航点2（到达阈值%.2fm） =====", waypoint_arrival_distance_);
    if (!send_waypoint_and_wait(wp4_main_, 3)) {
      RCLCPP_ERROR(get_logger(), "主航点2导航失败，终止流程");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "所有航点导航完成！");
    rclcpp::shutdown();
  }

  // 启动逻辑
  void on_start_timer()
  {
    RCLCPP_INFO(get_logger(), "开始解析4个航点...");

    if (!parse_four_waypoints()) {
      RCLCPP_ERROR(get_logger(), "解析4个航点失败，退出");
      rclcpp::shutdown();
      return;
    }

    if (rise_trigger_wp_index_ != 1) {
      RCLCPP_WARN(get_logger(), "上升索引自动调整为过渡点1（索引=1）");
      rise_trigger_wp_index_ = 1;
    }

    publish_waypoints_step_by_step();
  }

};  // 闭合WaypointLoader类括号

}  // 闭合sentry_waypoint_loader_cpp命名空间

// 主函数
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<sentry_waypoint_loader_cpp::WaypointLoader>(options);
  rclcpp::spin(node);  // 仅使用主执行器

  rclcpp::shutdown();
  return 0;
}

