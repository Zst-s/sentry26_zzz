#pragma once

#include <cmath>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <teb_local_planner/pose_se2.h>

namespace teb_local_planner
{

// Seed a holonomic trajectory on the shortest angular branch. Positions and
// endpoint poses stay unchanged. This is an initial guess, not a hard constraint.
inline std::vector<geometry_msgs::msg::PoseStamped> makeShortestRotationPlan(
  const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  auto result = plan;
  if (plan.size() < 3) return result;

  std::vector<double> distance(plan.size(), 0.0);
  for (size_t i = 1; i < plan.size(); ++i)
  {
    distance[i] = distance[i - 1] + std::hypot(
      plan[i].pose.position.x - plan[i - 1].pose.position.x,
      plan[i].pose.position.y - plan[i - 1].pose.position.y);
  }
  const double start_yaw = PoseSE2(plan.front().pose).theta();
  const double delta_yaw = g2o::normalize_theta(PoseSE2(plan.back().pose).theta() - start_yaw);
  for (size_t i = 1; i + 1 < plan.size(); ++i)
  {
    // In-place rotation has no arc length: distribute it by sample index.
    const double fraction = distance.back() > 1e-9
      ? distance[i] / distance.back() : static_cast<double>(i) / (plan.size() - 1);
    const double yaw = g2o::normalize_theta(start_yaw + fraction * delta_yaw);
    auto& q = result[i].pose.orientation;
    q.x = q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
  }
  return result;
}

} // namespace teb_local_planner
