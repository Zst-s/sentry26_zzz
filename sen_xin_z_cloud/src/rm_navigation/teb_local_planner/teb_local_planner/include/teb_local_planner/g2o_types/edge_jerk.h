#pragma once

#include <algorithm>
#include <cmath>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <g2o/stuff/misc.h>

namespace teb_local_planner {

// 与全向加速度边使用相同的局部坐标和中心差分。极小时长仅用于避免除零。
inline Eigen::Vector3d calculateHolonomicJerk(const PoseSE2* poses[4], const double times[3])
{
  double dt[3];
  Eigen::Vector3d velocity[3];
  for (int i = 0; i < 3; ++i) {
    dt[i] = std::max(times[i], 1e-4);
    const auto delta = poses[i + 1]->position() - poses[i]->position();
    const double c = std::cos(poses[i]->theta()), s = std::sin(poses[i]->theta());
    velocity[i] = Eigen::Vector3d((c * delta.x() + s * delta.y()) / dt[i],
                                  (-s * delta.x() + c * delta.y()) / dt[i],
                                  g2o::normalize_theta(poses[i + 1]->theta() - poses[i]->theta()) / dt[i]);
  }
  const Eigen::Vector3d a0 = 2.0 * (velocity[1] - velocity[0]) / (dt[0] + dt[1]);
  const Eigen::Vector3d a1 = 2.0 * (velocity[2] - velocity[1]) / (dt[1] + dt[2]);
  return (a1 - a0) / (0.25 * dt[0] + 0.5 * dt[1] + 0.25 * dt[2]);
}

class EdgeHolonomicJerk : public BaseTebMultiEdge<3, double> {
public:
  EdgeHolonomicJerk() { resize(7); }
  void computeError() override {
    const PoseSE2* poses[4];
    double dt[3];
    for (int i = 0; i < 4; ++i)
      poses[i] = &static_cast<const VertexPose*>(_vertices[i])->pose();
    for (int i = 0; i < 3; ++i)
      dt[i] = static_cast<const VertexTimeDiff*>(_vertices[i + 4])->dt();
    const auto jerk = calculateHolonomicJerk(poses, dt);
    const double limits[3] = {cfg_->trajectory.jerk_limit_x, cfg_->trajectory.jerk_limit_y,
                              cfg_->trajectory.jerk_limit_theta};
    for (int i = 0; i < 3; ++i)
      _error[i] = std::max(0.0, std::abs(jerk[i]) - limits[i]);
  }
};
}  // namespace teb_local_planner
