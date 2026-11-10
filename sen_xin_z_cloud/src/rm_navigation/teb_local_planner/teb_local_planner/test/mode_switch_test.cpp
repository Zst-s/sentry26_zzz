#include <gtest/gtest.h>
#include <fstream>
#include <thread>
#include <unistd.h>
#include "teb_local_planner/teb_local_planner_ros.h"

namespace teb_local_planner {
class ModeSwitchTestPeer {
public:
  static void configure(TebLocalPlannerROS &p, nav2_util::LifecycleNode::SharedPtr node) {
    p.name_ = "FollowPath";
    p.cfg_->robot.max_vel_x = 1.0;
    p.cfg_->robot.max_vel_x_backwards = 0.5;
    p.cfg_->robot.max_vel_y = 1.0;
    p.configureModes(node);
    p.mode_pub_->on_activate();
  }
  static bool pending(TebLocalPlannerROS &p, const std::string &s) {
    std::lock_guard<std::mutex> lock(p.mode_mutex_);
    return p.pending_mode_ == s;
  }
  static void boundary(TebLocalPlannerROS &p) {
    std::lock_guard<std::mutex> lock(p.cfg_->configMutex());
    p.applyPendingMode();
  }
  static double vx(TebLocalPlannerROS &p) { return p.cfg_->robot.max_vel_x; }
  static void limit(TebLocalPlannerROS &p, double value, bool percent) {
    p.setSpeedLimit(value, percent);
  }
  static double vy(TebLocalPlannerROS &p) { return p.cfg_->robot.max_vel_y; }
  static double weight(TebLocalPlannerROS &p) { return p.cfg_->optim.weight_obstacle; }
  static std::string active(TebLocalPlannerROS &p) { return p.active_mode_; }
};

TEST(ModeSwitch, TopicQueuesUntilBoundaryAndPreservesExternalLimit) {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  const std::string file = "/tmp/teb_mode_test_" + std::to_string(getpid()) + ".yaml";
  { std::ofstream out(file); out << "profiles: {slow: {max_vel_x: 0.2, weight_obstacle: 80}, fast: {max_vel_x: 0.8}}"; }
  auto node = std::make_shared<nav2_util::LifecycleNode>("mode_switch_test");
  node->declare_parameter("FollowPath.mode_profiles_file", file);
  node->declare_parameter("FollowPath.mode_topic", "/teb_mode_test");
  TebLocalPlannerROS planner;
  ModeSwitchTestPeer::configure(planner, node);
  std::remove(file.c_str()); // 后续切换不需要重新读取文件。
  auto client = std::make_shared<rclcpp::Node>("mode_switch_test_client");
  auto pub = client->create_publisher<std_msgs::msg::String>("/teb_mode_test", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client);
  auto request = [&](const std::string &mode) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      std_msgs::msg::String msg; msg.data = mode; pub->publish(msg);
      executor.spin_some();
      if (ModeSwitchTestPeer::pending(planner, mode)) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  };
  ASSERT_TRUE(request("slow"));
  EXPECT_DOUBLE_EQ(ModeSwitchTestPeer::vx(planner), 1.0);
  ModeSwitchTestPeer::boundary(planner);
  EXPECT_DOUBLE_EQ(ModeSwitchTestPeer::vx(planner), 0.2);
  EXPECT_DOUBLE_EQ(ModeSwitchTestPeer::weight(planner), 80.0);
  ModeSwitchTestPeer::limit(planner, 50.0, true);
  EXPECT_DOUBLE_EQ(ModeSwitchTestPeer::vx(planner), 0.1);
  ASSERT_TRUE(request("fast"));
  ModeSwitchTestPeer::boundary(planner);
  EXPECT_DOUBLE_EQ(ModeSwitchTestPeer::vx(planner), 0.4);
  EXPECT_DOUBLE_EQ(ModeSwitchTestPeer::vy(planner), 0.5);
  EXPECT_EQ(ModeSwitchTestPeer::active(planner), "fast");
  // 同一模式不重复应用；恢复 default 后保持 50% 外部限速。
  ASSERT_TRUE(request("default"));
  ModeSwitchTestPeer::boundary(planner);
  EXPECT_DOUBLE_EQ(ModeSwitchTestPeer::vx(planner), 0.5);
  ModeSwitchTestPeer::limit(planner, nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_DOUBLE_EQ(ModeSwitchTestPeer::vx(planner), 1.0);
  EXPECT_DOUBLE_EQ(ModeSwitchTestPeer::vy(planner), 1.0);
}
}  // namespace teb_local_planner
