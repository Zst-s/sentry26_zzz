#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "point_lio_frames.hpp"

TEST(PointLIOFrames, UsesTheSameExtrinsicsAsRegisteredClouds) {
  const double pi = std::acos(-1.);
  const Eigen::Matrix3d base_lidar_rotation =
      (Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(1.3090, Eigen::Vector3d::UnitX())).toRotationMatrix();
  tf2::Transform base_lidar;
  const auto &r = base_lidar_rotation;
  base_lidar.setBasis(tf2::Matrix3x3(r(0,0),r(0,1),r(0,2),r(1,0),r(1,1),r(1,2),r(2,0),r(2,1),r(2,2)));
  base_lidar.setOrigin(tf2::Vector3(0., -.32, .176));
  const Eigen::Matrix3d imu_lidar_rotation =
      Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d imu_lidar_translation(-.011, -.02329, .04412);
  tf2::Transform identity;
  identity.setIdentity();
  const auto imu_lidar = point_lio_frames::lidarPose(identity, imu_lidar_rotation, imu_lidar_translation);
  tf2::Quaternion heading;
  heading.setRPY(.1, -.15, .4);
  const tf2::Transform expected_base(heading, tf2::Vector3(1., 2., 3.));
  const auto odom_imu = expected_base * base_lidar * imu_lidar.inverse();
  const auto odom_lidar = point_lio_frames::lidarPose(odom_imu, imu_lidar_rotation, imu_lidar_translation);
  const auto actual_base = odom_lidar * base_lidar.inverse();
  EXPECT_NEAR((actual_base.getOrigin()-expected_base.getOrigin()).length(), 0., 1e-10);
  EXPECT_NEAR(actual_base.getRotation().angleShortestPath(expected_base.getRotation()), 0., 1e-7);
  const auto scan = point_lio_frames::levelScanPose(odom_lidar, actual_base);
  EXPECT_NEAR((scan.getOrigin()-odom_lidar.getOrigin()).length(), 0., 1e-10);
  EXPECT_NEAR(scan.getBasis().getColumn(2).z(), 1., 1e-10);
  EXPECT_NEAR(tf2::getYaw(scan.getRotation()), .4, 1e-10);
}

#include "dense_terrain_registration.hpp"
#include <limits>

TEST(DenseTerrain, RegistersEachAcquisitionTimeDuringTranslationAndRotation) {
  point_lio_frames::DenseTerrainTrajectory trajectory;
  tf2::Quaternion end_rotation;
  end_rotation.setRPY(.2, -.1, .4);
  const tf2::Transform begin(tf2::Quaternion::getIdentity(), tf2::Vector3(1.,2.,3.));
  const tf2::Transform end(end_rotation, tf2::Vector3(1.1,2.2,3.3));
  ASSERT_TRUE(trajectory.append(10., begin));
  ASSERT_TRUE(trajectory.append(10.01, end));
  const tf2::Vector3 target(4.,5.,6.);
  for (int i=0; i<=10; ++i) {
    const double alpha = i / 10.;
    tf2::Transform expected(begin.getRotation().slerp(end.getRotation(), alpha),
                            begin.getOrigin().lerp(end.getOrigin(), alpha));
    const auto observed = expected.inverse() * target;
    tf2::Transform actual;
    ASSERT_TRUE(trajectory.poseAt(10. + alpha * .01, actual));
    EXPECT_NEAR((actual * observed - target).length(), 0., 1e-10);
  }
  tf2::Transform unavailable;
  EXPECT_FALSE(trajectory.poseAt(9.999, unavailable));
  EXPECT_FALSE(trajectory.poseAt(10.011, unavailable));
}

TEST(DenseTerrain, RejectsGapsAndClearsOnResetInvalidStateOrTimeRollback) {
  point_lio_frames::DenseTerrainTrajectory trajectory;
  tf2::Transform pose; pose.setIdentity();
  ASSERT_TRUE(trajectory.append(1., pose));
  ASSERT_TRUE(trajectory.append(1.1, pose));
  EXPECT_FALSE(trajectory.poseAt(1.05, pose));
  trajectory.clear();
  EXPECT_FALSE(trajectory.poseAt(1., pose));
  ASSERT_TRUE(trajectory.append(2., pose));
  ASSERT_TRUE(trajectory.append(1., pose));
  EXPECT_FALSE(trajectory.poseAt(2., pose));
  EXPECT_FALSE(trajectory.append(std::numeric_limits<double>::quiet_NaN(), pose));
  EXPECT_FALSE(trajectory.poseAt(1., pose));
  pose.setOrigin(tf2::Vector3(std::numeric_limits<double>::infinity(),0.,0.));
  EXPECT_FALSE(trajectory.append(3., pose));
}

TEST(NavigationTwist, StationaryAndRotatedTranslation) {
  tf2::Quaternion q; q.setRPY(.3,-.2,1.2);
  tf2::Transform oi(q,tf2::Vector3(5,6,7)), ib;ib.setIdentity();
  Eigen::Vector3d v,w;
  ASSERT_TRUE(point_lio_frames::baseTwist(oi,ib,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),v,w));
  EXPECT_DOUBLE_EQ(v.norm()+w.norm(),0.);
  const Eigen::Vector3d world(1.2,-.7,.4);
  ASSERT_TRUE(point_lio_frames::baseTwist(oi,ib,world,Eigen::Vector3d::Zero(),v,w));
  auto expected=oi.getBasis().transpose()*tf2::Vector3(world.x(),world.y(),world.z());
  EXPECT_NEAR((v-Eigen::Vector3d(expected.x(),expected.y(),expected.z())).norm(),0.,1e-12);
}

TEST(NavigationTwist, ShiftOriginBeforeRotatingAxes) {
  tf2::Transform oi;oi.setIdentity();tf2::Quaternion q;q.setRPY(0,0,std::acos(-1.)/2);
  tf2::Transform ib(q,tf2::Vector3(.32,0.,0.));Eigen::Vector3d v,w;
  ASSERT_TRUE(point_lio_frames::baseTwist(oi,ib,Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,2),v,w));
  EXPECT_NEAR(v.x(),.64,1e-12);EXPECT_NEAR(v.y(),0.,1e-12);EXPECT_NEAR(w.z(),2.,1e-12);
  // Rotating about the base origin: cancel the IMU lever-arm velocity.
  ASSERT_TRUE(point_lio_frames::baseTwist(oi,ib,Eigen::Vector3d(0,-.64,0),Eigen::Vector3d(0,0,2),v,w));
  EXPECT_NEAR(v.norm(),0.,1e-12);
}

TEST(NavigationTwist, ActualTiltedLidarAndInternalExtrinsicsMatchFiniteDifference) {
  const double pi=std::acos(-1.);
  for(int i=0;i<200;++i) {
    tf2::Quaternion qi,qb;qi.setRPY(.01*i,-.003*i,.02*i);qb.setRPY(-35*pi/180.,0.,pi/2);
    tf2::Transform oi(qi,tf2::Vector3(1,2,3)), bl(qb,tf2::Vector3(-.32,0,.066));
    const auto ril=Eigen::AngleAxisd(.13,Eigen::Vector3d::UnitY()).toRotationMatrix();
    tf2::Transform identity;identity.setIdentity();
    auto il=point_lio_frames::lidarPose(identity,ril,Eigen::Vector3d(-.011,-.02329,.04412));
    auto ib=il*bl.inverse();Eigen::Vector3d speed(.4+.001*i,-.2,.1),omega(.1,-.15,.3),v,w;
    ASSERT_TRUE(point_lio_frames::baseTwist(oi,ib,speed,omega,v,w));
    const double dt=1e-6;tf2::Quaternion dq(tf2::Vector3(omega.x(),omega.y(),omega.z()).normalized(),omega.norm()*dt);
    tf2::Transform moved(qi*dq,oi.getOrigin()+tf2::Vector3(speed.x(),speed.y(),speed.z())*dt);
    const auto ob=oi*ib, next=moved*ib;
    const auto delta=ob.getBasis().transpose()*((next.getOrigin()-ob.getOrigin())/dt);
    EXPECT_NEAR((v-Eigen::Vector3d(delta.x(),delta.y(),delta.z())).norm(),0.,1e-7);
    const auto dr=ob.getRotation().inverse()*next.getRotation();
    const Eigen::Vector3d vector_part(dr.x(),dr.y(),dr.z());
    const double sine=vector_part.norm();
    const Eigen::Vector3d wd=vector_part*(2*std::atan2(sine,dr.w())/(sine*dt));
    EXPECT_NEAR((w-wd).norm(),0.,1e-8);
  }
}

TEST(NavigationTwist, InvalidInputDoesNotOverwriteOutputs) {
  tf2::Transform oi,ib;oi.setIdentity();ib.setIdentity();Eigen::Vector3d v(7,8,9),w(1,2,3),z=Eigen::Vector3d::Zero();
  auto bad=z;bad.x()=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(point_lio_frames::baseTwist(oi,ib,bad,z,v,w));
  EXPECT_FALSE(point_lio_frames::baseTwist(oi,ib,z,bad,v,w));
  ib.setOrigin(tf2::Vector3(std::numeric_limits<double>::infinity(),0,0));
  EXPECT_FALSE(point_lio_frames::baseTwist(oi,ib,z,z,v,w));ib.setIdentity();ib.getBasis()[0][0]=2.;
  EXPECT_FALSE(point_lio_frames::baseTwist(oi,ib,z,z,v,w));
  EXPECT_EQ(v,Eigen::Vector3d(7,8,9));EXPECT_EQ(w,Eigen::Vector3d(1,2,3));
}

TEST(PointLIOFrames, PlanarMappingBasePreservesXYHeadingAndPhysicalPose) {
  for (double roll : {-.45, -.15, 0., .15, .45}) {
    for (double pitch : {-.4, -.1, 0., .1, .4}) {
      for (double yaw : {-3.1, -1.5, 0., 1.5, 3.1}) {
        tf2::Quaternion q;q.setRPY(roll,pitch,yaw);
        const tf2::Transform physical(q,tf2::Vector3(1.2,-.7,.43));
        const auto planar=point_lio_frames::planarBasePose(physical);
        EXPECT_DOUBLE_EQ(planar.getOrigin().x(),1.2);
        EXPECT_DOUBLE_EQ(planar.getOrigin().y(),-.7);
        EXPECT_DOUBLE_EQ(planar.getOrigin().z(),0.);
        EXPECT_NEAR(planar.getBasis().getColumn(2).z(),1.,1e-12);
        EXPECT_NEAR(tf2::getYaw(planar.getRotation()),yaw,1e-12);
        EXPECT_DOUBLE_EQ(physical.getOrigin().z(),.43);
        EXPECT_NEAR(physical.getRotation().angleShortestPath(q),0.,1e-7);
      }
    }
  }
}

TEST(PointLIOFrames, SlopesCannotTiltMapCorrectionThroughPlanarBase) {
  tf2::Quaternion correction_q;correction_q.setRPY(0.,0.,.04);
  const tf2::Transform correction(correction_q,tf2::Vector3(.05,-.03,0.));
  for(int i=0;i<100;++i) {
    tf2::Quaternion q;q.setRPY(.3*std::sin(i*.1),.4*std::cos(i*.13),i*.05);
    const tf2::Transform physical(q,tf2::Vector3(i*.02,std::sin(i*.1),.5*std::sin(i*.11)));
    const auto planar=point_lio_frames::planarBasePose(physical);
    const auto corrected_base=correction*planar;
    // Same transform composition used by SLAM Toolbox, using its planar base.
    const auto map_odom=corrected_base*planar.inverse();
    EXPECT_NEAR((map_odom.getOrigin()-correction.getOrigin()).length(),0.,1e-12);
    EXPECT_NEAR(map_odom.getRotation().angleShortestPath(correction_q),0.,1e-7);
    const auto actual_map_base=map_odom*physical;
    EXPECT_NEAR((actual_map_base.getOrigin()-(correction*physical).getOrigin()).length(),0.,1e-12);
    EXPECT_NEAR(actual_map_base.getRotation().angleShortestPath((correction*physical).getRotation()),0.,1e-7);
  }
}
