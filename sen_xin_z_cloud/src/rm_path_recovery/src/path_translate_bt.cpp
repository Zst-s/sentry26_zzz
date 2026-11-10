#include "nav2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rm_path_recovery/action/path_translate.hpp"
#include <cmath>

namespace rm_path_recovery
{
class PathTranslateBT : public nav2_behavior_tree::BtActionNode<action::PathTranslate>
{
public:
  PathTranslateBT(const std::string & name, const BT::NodeConfiguration & config)
  : BtActionNode(name, "path_translate", config) {}
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<double>("distance", 0.3, "Maximum translation in metres"),
      BT::InputPort<double>("speed", 0.1, "Translation speed in metres per second"),
      BT::InputPort<double>("lookahead", 0.5, "Forward arc length on path"),
      BT::InputPort<double>("time_allowance", 5.0, "Timeout in seconds")});
  }
  void on_tick() override
  {
    double timeout = 0;
    should_send_goal_ = getInput("path", goal_.path) &&
      getInput("distance", goal_.distance) && getInput("speed", goal_.speed) &&
      getInput("lookahead", goal_.lookahead) && getInput("time_allowance", timeout);
    if (!should_send_goal_ || !std::isfinite(timeout) || timeout <= 0 || timeout > 30) {
      should_send_goal_ = false;
      return;
    }
    goal_.time_allowance = rclcpp::Duration::from_seconds(timeout);
    increment_recovery_count();
  }
};
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_path_recovery::PathTranslateBT>("PathTranslate");
}
