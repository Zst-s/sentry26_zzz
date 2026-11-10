#include "rm_behavior_tree/plugins/action/prepare_passage.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{
PreparePassage::PreparePassage(const std::string& name, const BT::NodeConfig& config,
                               const BT::RosNodeParams& params)
  : BT::SyncActionNode(name, config), node_(params.nh)
{
  if (!node_) {
    throw BT::RuntimeError("PreparePassage requires a ROS node");
  }
  // Retain the request so startup DDS discovery cannot lose this one-shot node.
  publisher_ = node_->create_publisher<std_msgs::msg::UInt8>(
      params.default_port_value.empty() ? "/if_guodong" : params.default_port_value,
      rclcpp::QoS(1).reliable().transient_local());
}

BT::PortsList PreparePassage::providedPorts()
{
  return { BT::InputPort<int>("if_guodong", "MCU request: 0=fold, 1=raise") };
}

BT::NodeStatus PreparePassage::tick()
{
  auto value = getInput<int>("if_guodong");
  if (!value || value.value() < 0 || value.value() > 1) {
    RCLCPP_ERROR(node_->get_logger(), "PreparePassage: if_guodong must be in [0,1]");
    return BT::NodeStatus::FAILURE;
  }
  std_msgs::msg::UInt8 message;
  message.data = static_cast<uint8_t>(value.value());
  publisher_->publish(message);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::PreparePassage, "PreparePassage");
