#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

namespace point_lio_frames {
inline tf2::Transform lidarPose(const tf2::Transform &odom_imu,
    const Eigen::Matrix3d &imu_lidar_rotation, const Eigen::Vector3d &imu_lidar_translation) {
  tf2::Transform imu_lidar;
  const auto &r = imu_lidar_rotation;
  imu_lidar.setBasis(tf2::Matrix3x3(r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2),
                                  r(2,0), r(2,1), r(2,2)));
  imu_lidar.setOrigin(tf2::Vector3(imu_lidar_translation.x(), imu_lidar_translation.y(),
                                 imu_lidar_translation.z()));
  return odom_imu * imu_lidar;
}

// v_odom is the velocity of the IMU origin in odom; omega_imu is body angular
// velocity in IMU axes. Shift to the base origin before rotating into base axes.
// Output arguments remain unchanged on invalid input.
inline bool baseTwist(const tf2::Transform &odom_imu, const tf2::Transform &imu_base,
                      const Eigen::Vector3d &v_odom, const Eigen::Vector3d &omega_imu,
                      Eigen::Vector3d &linear_base, Eigen::Vector3d &angular_base) {
  const auto matrix = [](const tf2::Matrix3x3 &r) {
    Eigen::Matrix3d out;
    for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) out(i,j)=r[i][j];
    return out;
  };
  const Eigen::Matrix3d r_oi=matrix(odom_imu.getBasis()), r_ib=matrix(imu_base.getBasis());
  const auto &p=imu_base.getOrigin();
  const Eigen::Vector3d p_ib(p.x(),p.y(),p.z());
  if (!v_odom.allFinite() || !omega_imu.allFinite() || !p_ib.allFinite() ||
      !r_oi.allFinite() || !r_ib.allFinite() ||
      !(r_oi.transpose()*r_oi).isApprox(Eigen::Matrix3d::Identity(),1e-6) ||
      !(r_ib.transpose()*r_ib).isApprox(Eigen::Matrix3d::Identity(),1e-6) ||
      std::abs(r_oi.determinant()-1.)>1e-6 || std::abs(r_ib.determinant()-1.)>1e-6) return false;
  const Eigen::Vector3d linear=r_ib.transpose()*(r_oi.transpose()*v_odom+omega_imu.cross(p_ib));
  const Eigen::Vector3d angular=r_ib.transpose()*omega_imu;
  if (!linear.allFinite() || !angular.allFinite()) return false;
  linear_base=linear; angular_base=angular;
  return true;
}

// Same lidar origin; horizontal axes follow base yaw in gravity-aligned odom.
inline tf2::Transform levelScanPose(const tf2::Transform &odom_lidar,
                                    const tf2::Transform &odom_base) {
  tf2::Quaternion yaw;
  yaw.setRPY(0., 0., tf2::getYaw(odom_base.getRotation()));
  return tf2::Transform(yaw, odom_lidar.getOrigin());
}
// Planar base-center frame for 2D mapping; keep the physical base pose intact.
inline tf2::Transform planarBasePose(const tf2::Transform &odom_base) {
  auto planar = levelScanPose(odom_base, odom_base);
  planar.getOrigin().setZ(0.);
  return planar;
}
}  // namespace point_lio_frames
