#ifndef TEB_GOAL_APPROACH_H
#define TEB_GOAL_APPROACH_H

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace teb_local_planner
{
// 返回整体速度缩放系数：保持全向平移方向与 v/omega，不改变运动曲率。
inline double goalApproachScale(double distance, double threshold, double max_speed,
                                double vx, double vy)
{
  if (!std::isfinite(distance) || distance < 0 ||
      !std::isfinite(threshold) || threshold <= 0 ||
      !std::isfinite(max_speed) || max_speed <= 0 ||
      !std::isfinite(vx) || !std::isfinite(vy))
    throw std::invalid_argument("Invalid goal approach distance, speed or configuration");
  const double speed = std::hypot(vx, vy);
  const double limit = max_speed * std::min(1.0, distance / threshold);
  return speed > limit ? limit / speed : 1.0;
}
}
#endif
