#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class WaypointLoader : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  // 👉 关键修改1：构造函数新增执行器指针参数，用于统一管理
  WaypointLoader(rclcpp::executors::SingleThreadedExecutor* executor, 
                const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("waypoint_loader_cpp", options), executor_(executor)  // 初始化执行器指针
  {
    // 声明参数：航点文件路径（默认自定义路径）、启动延迟
    this->declare_parameter<std::string>(
      "waypoints_file", 
      std::string(get_home() + "/sentry_ws/src/sentry_waypoint_loader_cpp/config/waypoints.yaml")
    );
    this->declare_parameter<double>("startup_delay", 6.0);

    // 读取参数
    waypoints_file_ = this->get_parameter("waypoints_file").as_string();
    startup_delay_ = this->get_parameter("startup_delay").as_double();

    RCLCPP_INFO(
      get_logger(), 
      "WaypointLoader: 航点文件=%s，启动延迟=%.1fs", 
      waypoints_file_.c_str(), startup_delay_
    );

    // 创建动作客户端（连接 Nav2 的 follow_waypoints 接口）
    follow_action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "/follow_waypoints");

    // 延迟启动（确保 Nav2 节点就绪）
    start_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(startup_delay_),
      [this]() {
        start_timer_->cancel();  // 仅执行一次
        this->on_start_timer();
      });
  }

private:
  std::string waypoints_file_;
  double startup_delay_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;  // 存储航点列表（供回调访问）
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_action_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  // 👉 关键修改2：添加执行器指针成员（由 main 函数传递，统一管理节点）
  rclcpp::executors::SingleThreadedExecutor* executor_;

  // 获取用户主目录
  static std::string get_home()
  {
    const char * h = std::getenv("HOME");
    return h ? std::string(h) : std::string(".");
  }

  // 等待动作服务器就绪（带超时）
  bool wait_for_action_server_with_timeout(double timeout_s = 20.0)
  {
    auto t0 = this->now();
    while (!follow_action_client_->wait_for_action_server(1s)) {
      auto elapsed = (this->now() - t0).seconds();
      if (elapsed >= timeout_s) {
        RCLCPP_ERROR(get_logger(), "等待动作服务器超时（%ds）", static_cast<int>(timeout_s));
        return false;
      }
      RCLCPP_INFO(get_logger(), "等待 FollowWaypoints 动作服务器...");
    }
    return true;
  }

  // 从 YAML 解析航点
  std::vector<geometry_msgs::msg::PoseStamped> parse_waypoints_from_yaml()
  {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    YAML::Node yaml_node;

    // 打开 YAML 文件
    try {
      yaml_node = YAML::LoadFile(waypoints_file_);
    } catch (const YAML::BadFile& e) {
      RCLCPP_ERROR(get_logger(), "打开航点文件失败：%s，错误：%s", waypoints_file_.c_str(), e.what());
      return waypoints;
    }

    // 解析航点列表（waypoints: [wp0, wp1, ...]）
    if (!yaml_node["waypoints"]) {
      RCLCPP_ERROR(get_logger(), "YAML 文件无 'waypoints' 字段");
      return waypoints;
    }

    for (const auto& wp_name : yaml_node["waypoints"]) {
      std::string name = wp_name.as<std::string>();
      if (!yaml_node[name]) {
        RCLCPP_WARN(get_logger(), "跳过不存在的航点：%s", name.c_str());
        continue;
      }

      YAML::Node wp = yaml_node[name];
      geometry_msgs::msg::PoseStamped pose;

      // 解析坐标系（默认 map）
      pose.header.frame_id = wp["header"]["frame_id"].as<std::string>("map");
      pose.header.stamp = this->now();

      // 解析位置（x/y 必选，z 默认 0）
      pose.pose.position.x = wp["pose"]["position"]["x"].as<double>();
      pose.pose.position.y = wp["pose"]["position"]["y"].as<double>();
      pose.pose.position.z = wp["pose"]["position"]["z"].as<double>(0.0);

      // 解析朝向（默认 x 轴正方向）
      pose.pose.orientation.x = wp["pose"]["orientation"]["x"].as<double>(0.0);
      pose.pose.orientation.y = wp["pose"]["orientation"]["y"].as<double>(0.0);
      pose.pose.orientation.z = wp["pose"]["orientation"]["z"].as<double>(0.0);
      pose.pose.orientation.w = wp["pose"]["orientation"]["w"].as<double>(1.0);

      waypoints.push_back(pose);
      RCLCPP_INFO(
        get_logger(), 
        "解析航点 %s：(%.2f, %.2f)，坐标系：%s", 
        name.c_str(), pose.pose.position.x, pose.pose.position.y, pose.header.frame_id.c_str()
      );
    }

    return waypoints;
  }

  // 延迟到期后执行：解析航点 + 发送动作
  void on_start_timer()
  {
    RCLCPP_INFO(get_logger(), "开始加载航点...");

    // 1. 检查航点文件是否存在
    if (!std::ifstream(waypoints_file_)) {
      RCLCPP_ERROR(get_logger(), "航点文件不存在：%s", waypoints_file_.c_str());
      rclcpp::shutdown();
      return;
    }

    // 2. 解析航点（赋值给类成员 waypoints_）
    waypoints_ = parse_waypoints_from_yaml();
    if (waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "未解析到有效航点");
      rclcpp::shutdown();
      return;
    }

    // 3. 等待动作服务器就绪
    if (!wait_for_action_server_with_timeout(30.0)) {
      rclcpp::shutdown();
      return;
    }

    // 4. 构造动作目标（填入航点列表）
    FollowWaypoints::Goal goal_msg;
    goal_msg.poses = waypoints_;  // 从类成员获取航点

    // 5. 设置动作回调（Result + Feedback）
    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

    // Result 回调（Humble 无 success 成员，靠状态码判断）
    send_goal_options.result_callback =
      [this](const GoalHandleFollow::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "所有航点导航完成（整体成功）！");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "航点导航被中止（如遇障无法继续、目标不可达）");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "航点导航被手动取消");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "航点导航未知错误，状态码：%d", static_cast<int>(result.code));
        }
        rclcpp::shutdown();
      };

    // Feedback 回调（仅 current_waypoint，总数量从 waypoints_ 获取）
    send_goal_options.feedback_callback =
      [this](GoalHandleFollow::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        int current = feedback->current_waypoint + 1;  // 0→1（直观显示）
        int total = waypoints_.size();                  // 总数量从类成员获取
        RCLCPP_INFO(get_logger(), "航点进度：正在执行第 %d/%d 个航点", current, total);
      };

    // 6. 发送动作目标
    RCLCPP_INFO(
      get_logger(), 
      "发送 %zu 个航点到 FollowWaypoints 动作服务器...", 
      waypoints_.size()
    );
    auto goal_future = follow_action_client_->async_send_goal(goal_msg, send_goal_options);

    // 👉 关键修改3：用显式执行器等待目标结果（避免隐式临时执行器）
    if (executor_->spin_until_future_complete(goal_future, 10s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "发送目标超时或失败");
      rclcpp::shutdown();
      return;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "航点目标被服务器拒绝");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "航点目标已接受，等待导航完成...");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // 👉 关键修改4：显式创建单线程执行器（统一管理所有节点逻辑）
  rclcpp::executors::SingleThreadedExecutor executor;

  // 创建节点时，将执行器指针传递给节点
  auto node = std::make_shared<WaypointLoader>(&executor, options);

  // 将节点添加到执行器（仅添加一次）
  executor.add_node(node);

  // 执行器自旋（替代原 rclcpp::spin(node)，统一处理回调）
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
