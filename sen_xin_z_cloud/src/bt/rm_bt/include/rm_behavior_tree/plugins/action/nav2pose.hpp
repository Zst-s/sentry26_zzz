#ifndef RM_DECISION_NAV2POSE_HPP_
#define RM_DECISION_NAV2POSE_HPP_
#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <action_msgs/msg/goal_status.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "rm_behavior_tree/bt_conversions.hpp"

#include <atomic>
#include <chrono>
#include <memory>

using namespace BT;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace rm_behavior_tree
{

class Nav2Pose : public StatefulActionNode
{
public:
  Nav2Pose(const std::string& name, const NodeConfig& config, std::shared_ptr<rclcpp::Node> parent_node);

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

  static PortsList providedPorts();

private:
  // 使用父节点，所有回调在主线程处理
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  static std::atomic<int> instance_counter_;
  int instance_id_;

  // 状态通过原子变量管理
  std::atomic<bool> goal_received_{ false };
  std::atomic<bool> goal_accepted_{ false };
  std::atomic<bool> result_received_{ false };
  std::atomic<int> result_code_{ 0 };

  // 用于匹配请求防止旧回调干扰
  std::atomic<uint64_t> current_goal_seq_{ 0 };

  int send_goal_timeout_{ 10000 };
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  double speed_limit_{-1.0};
  bool speed_limit_sent_{false};
  std::chrono::steady_clock::time_point speed_limit_sent_at_;
  void publishSpeedLimit();
  bool goal_sent_{false};
  NodeStatus sendGoal();
  NavigateToPoseGoalHandle::SharedPtr goal_handle_;
  std::chrono::steady_clock::time_point goal_sent_at_;
  void cancelGoal(const NavigateToPoseGoalHandle::SharedPtr& handle);
};

}  // namespace rm_behavior_tree

#endif
