#include "rm_path_recovery/geometry.hpp"
#include "rm_path_recovery/action/path_translate.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include <chrono>
#include <vector>
#include <stdexcept>

namespace rm_path_recovery {
class PathTranslate : public nav2_behaviors::TimedBehavior<action::PathTranslate> {
  using Goal = action::PathTranslate::Goal;
  geometry_msgs::msg::PoseStamped start_;
  Point direction_{};
  double distance_ = 0, speed_ = 0;
  rclcpp::Time deadline_;
  rclcpp::Clock::SharedPtr clock_;

  void onConfigure() override {
    auto node = node_.lock();
    if (!node) throw std::runtime_error("PathTranslate lifecycle node expired");
    clock_ = node->get_clock();
  }

  std::shared_ptr<action::PathTranslate::Feedback> feedback_ =
    std::make_shared<action::PathTranslate::Feedback>();

  nav2_behaviors::Status onRun(const std::shared_ptr<const Goal> goal) override {
    if (!goal || goal->path.poses.size() < 2 || goal->path.header.frame_id.empty() ||
        !std::isfinite(goal->distance) || goal->distance <= 0 || goal->distance > 0.5 ||
        !std::isfinite(goal->speed) || goal->speed <= 0 || goal->speed > 0.2 ||
        !std::isfinite(goal->lookahead) || goal->lookahead <= 0 || goal->lookahead > 2.0) {
      return nav2_behaviors::Status::FAILED;
    }
    const auto path_stamp = rclcpp::Time(goal->path.header.stamp, clock_->get_clock_type());
    const double path_age = (clock_->now() - path_stamp).seconds();
    if (path_stamp.nanoseconds() == 0 || path_age < -1.0 || path_age > 10.0) {
      RCLCPP_WARN(logger_, "No current path for translation recovery");
      return nav2_behaviors::Status::FAILED;
    }
    distance_ = goal->distance;
    speed_ = goal->speed;
    if (!nav2_util::getCurrentPose(start_, *tf_, goal->path.header.frame_id,
        robot_base_frame_, transform_tolerance_)) return nav2_behaviors::Status::FAILED;
    std::vector<Point> path;
    path.reserve(goal->path.poses.size());
    for (const auto & pose : goal->path.poses) {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != goal->path.header.frame_id)
        return nav2_behaviors::Status::FAILED;
      path.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    try {
      auto target = lookaheadPoint(path,
        {start_.pose.position.x, start_.pose.position.y}, goal->lookahead, 1.0);
      direction_ = subtract(target, {start_.pose.position.x, start_.pose.position.y});
      const double norm = length(direction_);
      if (norm < 0.05) return nav2_behaviors::Status::FAILED;
      direction_.x /= norm;
      direction_.y /= norm;
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "Path translation rejected: %s", e.what());
      return nav2_behaviors::Status::FAILED;
    }
    deadline_ = clock_->now() + rclcpp::Duration(goal->time_allowance);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  nav2_behaviors::Status onCycleUpdate() override {
    if (clock_->now() >= deadline_) {stopRobot(); return nav2_behaviors::Status::FAILED;}
    geometry_msgs::msg::PoseStamped pose;
    if (!nav2_util::getCurrentPose(pose, *tf_, start_.header.frame_id,
        robot_base_frame_, transform_tolerance_)) {
      stopRobot(); return nav2_behaviors::Status::FAILED;
    }
    const Point moved = subtract({pose.pose.position.x, pose.pose.position.y},
      {start_.pose.position.x, start_.pose.position.y});
    const double progress = moved.x*direction_.x + moved.y*direction_.y;
    feedback_->distance_traveled = std::max(0.0, progress);
    action_server_->publish_feedback(feedback_);
    if (progress >= distance_) {stopRobot(); return nav2_behaviors::Status::SUCCEEDED;}
    if (std::abs(moved.x*direction_.y - moved.y*direction_.x) > 0.15 || progress < -0.1) {
      stopRobot(); return nav2_behaviors::Status::FAILED;
    }
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const Point body = bodyVelocity(direction_, speed_, yaw);
    // Sweep the actual footprint over the commanded path at 5 cm intervals.
    const double check_distance = std::min(distance_ - progress + 0.05, speed_ + 0.05);
    bool fetch = true;
    for (double step=0.0; step<=check_distance; step+=0.05) {
      geometry_msgs::msg::Pose2D trial;
      trial.x = pose.pose.position.x + direction_.x*step;
      trial.y = pose.pose.position.y + direction_.y*step;
      trial.theta = yaw;
      if (!collision_checker_->isCollisionFree(trial, fetch)) {
        stopRobot(); return nav2_behaviors::Status::FAILED;
      }
      fetch = false;
    }
    auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
    cmd->linear.x = body.x;
    cmd->linear.y = body.y;
    vel_pub_->publish(std::move(cmd));
    return nav2_behaviors::Status::RUNNING;
  }
};
}  // namespace rm_path_recovery
PLUGINLIB_EXPORT_CLASS(rm_path_recovery::PathTranslate, nav2_core::Behavior)
