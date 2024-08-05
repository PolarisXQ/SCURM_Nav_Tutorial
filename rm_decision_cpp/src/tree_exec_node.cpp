#include "rm_decision_cpp/behaviors/attack.hpp"
#include "rm_decision_cpp/behaviors/spin.hpp"
#include "rm_decision_cpp/behaviors/nav2pose.hpp"
#include "rm_decision_cpp/behaviors/topics2blackboard.hpp"
#include "rm_decision_cpp/behaviors/anti_autoaim.hpp"
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <chrono>
#include "rm_decision_cpp/custume_types.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tree_exec");

  //declare parameters
  node->declare_parameter<std::string>("tree_xml_file", "");
  node->declare_parameter("tick_period_milliseconds", 3000);
  node->declare_parameter("groot_port", 5556);
  node->declare_parameter("tree_node_model_export_path","");
  //get parameters
  std::string tree_xml_file;
  node->get_parameter("tree_xml_file",tree_xml_file);
  int tick_period_milliseconds;
  node->get_parameter("tick_period_milliseconds",tick_period_milliseconds);
  std::chrono::system_clock::duration timeout;
  timeout = std::chrono::milliseconds(tick_period_milliseconds);
  unsigned int groot_port = 5556;
  node->get_parameter("groot_port",groot_port);
  std::string tree_node_model_export_path;
  node->get_parameter("tree_node_model_export_path",tree_node_model_export_path);

  BT::BehaviorTreeFactory factory;

  // Register the custom nodes
  factory.registerNodeType<rm_decision::Nav2Pose>("Nav2Pose", node);
  factory.registerNodeType<rm_decision::AntiAutoAim>("AntiAutoAim",node);
  factory.registerNodeType<rm_decision::Spin>("Spin",node);
  factory.registerNodeType<rm_decision::Attack>("Attack",node);
  factory.registerNodeType<rm_decision::Topics2Blackboard>("Topics2Blackboard",node);
  RCLCPP_INFO(node->get_logger(), "Loaded all custom nodes");

  // Visualize custom types in the Blackboard
  BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>(PoseStampedToJson);
  BT::RegisterJsonDefinition<geometry_msgs::msg::PointStamped>(PointStampedToJson);


  // generate xml file
  std::string xml_models = BT::writeTreeNodesModelXML(factory);
  // save to file
  std::ofstream file(tree_node_model_export_path);
  file << xml_models;
  file.close();
  RCLCPP_INFO(node->get_logger(), "Generated XML file");
  
  auto tree = factory.createTreeFromFile(tree_xml_file.c_str());
  std::unique_ptr<BT::Groot2Publisher> groot2publisher_ptr_;
  groot2publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree, groot_port);

  unsigned long int tick_count = 0;
  while(rclcpp::ok()){
    tick_count++;
    RCLCPP_INFO(node->get_logger(), "----------Tick %lu---------", tick_count);
    rclcpp::spin_some(node);
    auto status = tree.tickOnce();
    tree.sleep(timeout);
  }

  rclcpp::shutdown();
  return 0;
}