#include "rm_decision_cpp/behaviors/nav2pose.hpp"

using namespace BT;
namespace rm_decision
{
  Nav2Pose::Nav2Pose(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
  : StatefulActionNode(name, config), node_(node)
  {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  }

  NodeStatus Nav2Pose::onStart()
  {
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal)
    {
      RCLCPP_ERROR(node_->get_logger(), "goal is not set");
      return NodeStatus::FAILURE;
    }

    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    navigation_goal_.pose = goal.value();
    navigation_goal_.pose.header.frame_id = "map";
    navigation_goal_.pose.header.stamp = node_->now();
    // navigation_goal_.behavior_tree = "navigate_to_pose";
    // auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // send_goal_options.goal_response_callback = std::bind(&Nav2Pose::goal_response_callback, this, std::placeholders::_1);
    // send_goal_options.feedback_callback = std::bind(&Nav2Pose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    // send_goal_options.result_callback = std::bind(&Nav2Pose::result_callback, this, std::placeholders::_1);
    // send_goal_options.goal = navigation_goal_;

    auto future_goal_handle = action_client_->async_send_goal(navigation_goal_);
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "send goal failed");
      return NodeStatus::FAILURE;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(node_->get_logger(), "goal handle is null");
      return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Navigating to pose [%f, %f, %f]", goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
    return NodeStatus::RUNNING;
  }

  NodeStatus Nav2Pose::onRunning()
  {
    switch (goal_handle_->get_status())
    {
    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
      RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_UNKNOWN");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_ACCEPTED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_EXECUTING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_CANCELING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_SUCCEEDED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_CANCELED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_ABORTED");
      break;
    default:
      RCLCPP_INFO(node_->get_logger(), "goal status: ERROR CODE");
      break;
    }

    if (goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
    {
      return NodeStatus::SUCCESS;
    }
    else if (goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
              goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      return NodeStatus::FAILURE;
    }
    else
    {
      return NodeStatus::RUNNING;
    }
  }

  void Nav2Pose::onHalted()
  {
    RCLCPP_INFO(node_->get_logger(), "goal halted");
    if (goal_handle_)
    {
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
      if (rclcpp::spin_until_future_complete(node_, cancel_future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "cancel goal failed");
      }
      RCLCPP_INFO(node_->get_logger(), "goal canceled");
    }
  }

  PortsList Nav2Pose::providedPorts()
  {
    const char *description = "goal send to navigator.";
    return {InputPort<geometry_msgs::msg::PoseStamped>("goal", description)};
  }

} // end namespace rm_decision