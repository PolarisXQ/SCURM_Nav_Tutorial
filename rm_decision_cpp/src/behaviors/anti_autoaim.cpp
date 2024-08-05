#include "rm_decision_cpp/behaviors/anti_autoaim.hpp"

using namespace BT;
namespace rm_decision
{
  AntiAutoAim::AntiAutoAim(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
  : StatefulActionNode(name, config), node_(node)
  {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  }

  NodeStatus AntiAutoAim::onStart()
  {
    // rclcpp::spin_some(node_);
    if (!getInput("msec", msec_))
    {
      msec_ = 4;
    }

    if (!getInput("speed", speed_))
    {
      speed_ = 1.0;
    }

    // create a timestamp
    start_time_ = node_->now();

    return NodeStatus::RUNNING;
  }

  NodeStatus AntiAutoAim::onRunning()
  {
    // rclcpp::spin_some(node_);
    auto now = node_->now();
    auto elapsed_time = (now - start_time_).seconds();
    geometry_msgs::msg::Twist cmd_vel_msg;
    if (elapsed_time >= msec_)
    {
      cmd_vel_msg.linear.x = 0;
      cmd_vel_msg.linear.y = 0;
      cmd_vel_msg.linear.z = 0;
      cmd_vel_msg.angular.x = 0;
      cmd_vel_msg.angular.y = 0;
      cmd_vel_msg.angular.z = 0;
      cmd_vel_pub_->publish(cmd_vel_msg);
      return NodeStatus::SUCCESS;
    }
    else
    {
      float angle = 30.0;
      // change direction every 0.5 seconds
      switch((int)elapsed_time*2 % 4)
      {
        case 0:
          cmd_vel_msg.linear.y = speed_;
          break;
        case 1:
          cmd_vel_msg.linear.x = -sin(angle/180*3.14)*speed_*cos(angle/180*3.14);
          cmd_vel_msg.linear.y = -cos(angle/180*3.14)*speed_*cos(angle/180*3.14);
          break;
        case 2:
          cmd_vel_msg.linear.y = speed_;
          break;
        case 3:
          cmd_vel_msg.linear.x = sin(angle/180*3.14)*speed_*cos(angle/180*3.14);
          cmd_vel_msg.angular.y = cos(angle/180*3.14)*speed_*cos(angle/180*3.14);
          break;
      }
      cmd_vel_pub_->publish(cmd_vel_msg);
      return NodeStatus::RUNNING;
    }
  }
  

  void AntiAutoAim::onHalted()
  {
    // stop the robot
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  PortsList AntiAutoAim::providedPorts()
  {
    return {InputPort<float>("msec"),
            InputPort<float>("speed")
            };
  }
} // end namespace rm_decision