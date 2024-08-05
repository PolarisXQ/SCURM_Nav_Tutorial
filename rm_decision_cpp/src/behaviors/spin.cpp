#include "rm_decision_cpp/behaviors/spin.hpp"
using namespace BT;
namespace rm_decision
{

  Spin::Spin(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
  : SyncActionNode(name, config),node_(node)
  {
    chassis_type_pub_ = node_->create_publisher<std_msgs::msg::Int8>("/chassis_type", rclcpp::QoS(1).transient_local());
    last_hp_ = 600;
  }

  NodeStatus Spin::tick()
  {
    // rclcpp::spin_some(node_);
    auto spin=getInput<bool>("spin");
    if(!spin)
    {
      throw RuntimeError("error reading port [spin]:",spin.error());
    }
    spin_=spin.value();

    auto current_hp = getInput<std::uint16_t>("current_hp");
    std::uint16_t current_hp_value;
    if(!current_hp)
    {
      RCLCPP_WARN(rclcpp::get_logger("SPIN"), "error reading port [current_hp]");
      current_hp_value = last_hp_;
    }
    else
    {
      current_hp_value = current_hp.value();
    }

    auto hurt_type = getInput<std::uint8_t>("hurt_type");
    std::uint8_t hurt_type_value;
    if(!hurt_type)
    {
      RCLCPP_WARN(rclcpp::get_logger("SPIN"), "error reading port [hurt_type]");
      hurt_type_value = 0;
    }
    else
    {
      hurt_type_value = hurt_type.value();
    }

    if (spin_)
    {
      std_msgs::msg::Int8 chassis_type;
      chassis_type.data = 4;
      chassis_type_pub_->publish(chassis_type);
    }
    else
    {
      if (current_hp_value < last_hp_ && hurt_type_value == 0)
      {
          std_msgs::msg::Int8 chassis_type;
          chassis_type.data = 4;
          chassis_type_pub_->publish(chassis_type);
      }
      else
      {
        std_msgs::msg::Int8 chassis_type;
        chassis_type.data = 1;
        chassis_type_pub_->publish(chassis_type);
      }
    }
    last_hp_ = current_hp_value;
    return NodeStatus::SUCCESS;
  }

  PortsList Spin::providedPorts()
  {
    const char *description = "spin or not.";
    return {InputPort<bool>("spin", description),
            InputPort<std::uint16_t>("current_hp"),
            InputPort<std::uint8_t>("hurt_type")
            };
  }

} // end namespace rm_decision