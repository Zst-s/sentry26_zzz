#pragma once

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace rm_behavior_tree
{
// Publishes a request, then lets the following navigation node run immediately.
// SUCCESS means request published, not mechanical folding completed.
class PreparePassage : public BT::SyncActionNode
{
public:
  PreparePassage(const std::string& name, const BT::NodeConfig& config,
                 const BT::RosNodeParams& params);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
};
}  // namespace rm_behavior_tree
