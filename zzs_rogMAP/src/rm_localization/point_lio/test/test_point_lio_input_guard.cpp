#include <gtest/gtest.h>
#include <limits>
#include "point_lio_input_guard.hpp"
TEST(PointLIOInputGuard, NormalAndRecoverableFrameLoss) {
  point_lio::InputGuard g;g.validate();
  for (int n=0;n<100;++n) {
    const double t=n*.005;ASSERT_TRUE(g.imu(100+t,t));
    if(n%20==0){ASSERT_TRUE(g.lidar(100+t,t));ASSERT_TRUE(g.frame(true,100+t,t));}
    ASSERT_TRUE(g.poll(t,100+t,true,true));ASSERT_NO_THROW(g.prediction(.005));
  }
  ASSERT_FALSE(g.frame(false,100.5,.5));EXPECT_STREQ(g.state(),"DEGRADED");
  ASSERT_TRUE(g.imu(100.5,.5));ASSERT_TRUE(g.frame(true,100.6,.6));EXPECT_TRUE(g.healthy());
  EXPECT_NO_THROW(g.prediction(-.00026));
}
TEST(PointLIOInputGuard, StartupInterruptRecollectsRatherThanLatches) {
  point_lio::InputGuard g;ASSERT_TRUE(g.imu(100.,0.));
  EXPECT_FALSE(g.imu(99.5,.01));EXPECT_FALSE(g.failed());EXPECT_TRUE(g.take_startup_restart());
  EXPECT_TRUE(g.imu(99.5,.02));EXPECT_STREQ(g.state(),"INITIALIZING");
  ASSERT_TRUE(g.poll(.02,99.5,true,true));EXPECT_FALSE(g.poll(.4,99.88,true,true));
  EXPECT_FALSE(g.failed());EXPECT_TRUE(g.take_startup_restart());
}
TEST(PointLIOInputGuard, TrackingGapAndRollbackCannotResume) {
  for(bool gap:{false,true}){
    point_lio::InputGuard g;ASSERT_TRUE(g.imu(100.,0.));ASSERT_TRUE(g.frame(true,100.,0.));
    EXPECT_FALSE(g.imu(gap?102.435723:99.5,.01));EXPECT_TRUE(g.failed());
    EXPECT_FALSE(g.frame(true,103.,.02));EXPECT_FALSE(g.take_startup_restart());
  }
}
TEST(PointLIOInputGuard, BadPacketIsQuarantinedButDoesNotRenewConstraintAge) {
  point_lio::InputGuard g;ASSERT_TRUE(g.frame(true,100.,0.));
  EXPECT_FALSE(g.scan(4.29496484375));EXPECT_FALSE(g.failed());EXPECT_FALSE(g.healthy());
  EXPECT_TRUE(g.frame(true,100.1,.1));
  EXPECT_FALSE(g.frame(false,100.41,.41));EXPECT_TRUE(g.failed());
}
TEST(PointLIOInputGuard, DuplicateNonfiniteAndSimPause) {
  point_lio::InputGuard g;ASSERT_TRUE(g.imu(100.,0.));ASSERT_TRUE(g.lidar(100.,0.));
  ASSERT_TRUE(g.frame(true,100.,0.));EXPECT_FALSE(g.imu(100.,.2));
  ASSERT_TRUE(g.poll(0.,100.,true,false));EXPECT_TRUE(g.poll(10.,100.,true,false));
  EXPECT_FALSE(g.scan(std::numeric_limits<double>::quiet_NaN()));EXPECT_FALSE(g.failed());
  EXPECT_THROW(g.prediction(.90937),std::runtime_error);EXPECT_TRUE(g.failed());
}
TEST(PointLIOInputGuard, WallTimeoutDegradesButClockJumpRetainsTerminalSession) {
  point_lio::InputGuard g;ASSERT_TRUE(g.imu(100.,0.));ASSERT_TRUE(g.lidar(100.,0.));
  ASSERT_TRUE(g.frame(true,100.,0.));ASSERT_TRUE(g.poll(0.,100.,true,true));
  EXPECT_TRUE(g.poll(.31,100.31,true,true));EXPECT_FALSE(g.failed());EXPECT_FALSE(g.healthy());
  point_lio::InputGuard clock;ASSERT_TRUE(clock.frame(true,100.,0.));
  ASSERT_TRUE(clock.poll(0.,100.,true,true));EXPECT_FALSE(clock.poll(.01,99.51,true,true));
  EXPECT_TRUE(clock.failed());
}

TEST(PointLIOInputGuard, ProcessingBacklogWithContinuousSamplesDoesNotResetSession) {
  point_lio::InputGuard g;
  ASSERT_TRUE(g.imu(100.,0.));ASSERT_TRUE(g.lidar(100.,0.));
  ASSERT_TRUE(g.frame(true,100.,0.));ASSERT_TRUE(g.poll(0.,100.,true,true));
  ASSERT_TRUE(g.imu(100.005,1.));ASSERT_TRUE(g.lidar(100.1,1.));
  EXPECT_TRUE(g.poll(1.,101.,true,true));EXPECT_FALSE(g.failed());EXPECT_FALSE(g.healthy());
  EXPECT_TRUE(g.frame(true,100.1,1.));EXPECT_TRUE(g.healthy());
}

TEST(PointLIOInputGuard, ContinuousBufferedSamplesRecoverOnlyWithAValidFrame) {
  point_lio::InputGuard g;g.imu(100.,0.);g.lidar(100.,0.);g.frame(true,100.,0.);g.poll(0.,100.,true,true);
  EXPECT_TRUE(g.poll(.65,100.65,true,true));EXPECT_STREQ(g.state(),"DEGRADED");EXPECT_FALSE(g.failed());EXPECT_EQ(g.generation(),1u);
  // Buffered samples may be old in wall time, but are consecutive in sensor time.
  ASSERT_TRUE(g.imu(100.005,.651));ASSERT_TRUE(g.lidar(100.1,.651));EXPECT_FALSE(g.healthy());
  EXPECT_EQ(g.last_valid_stamp(),100.);EXPECT_TRUE(g.frame(true,100.1,.652));EXPECT_TRUE(g.healthy());EXPECT_EQ(g.generation(),1u);
}
TEST(PointLIOInputGuard, TrueSensorGapsAfterWallTimeoutStillLatch) {
  for(bool imu_gap:{false,true}) {
    point_lio::InputGuard g;g.imu(100.,0.);g.lidar(100.,0.);g.frame(true,100.,0.);g.poll(0.,100.,true,true);
    ASSERT_TRUE(g.poll(.65,100.65,true,true));ASSERT_FALSE(g.failed());
    EXPECT_FALSE(imu_gap?g.imu(100.65,.651):g.lidar(100.65,.651));EXPECT_TRUE(g.failed());
    EXPECT_FALSE(g.frame(true,100.1,.652));EXPECT_FALSE(g.take_startup_restart());
  }
}
TEST(PointLIOInputGuard, SilenceStaysBlockedAndDoesNotRenewLastValidStamp) {
  point_lio::InputGuard g;g.imu(100.,0.);g.lidar(100.,0.);g.frame(true,100.,0.);g.poll(0.,100.,true,true);
  for(int k=1;k<=200;++k){EXPECT_TRUE(g.poll(k*.05,100.+k*.05,true,true));if(k>6)EXPECT_FALSE(g.healthy());}
  EXPECT_EQ(g.last_valid_stamp(),100.);EXPECT_EQ(g.generation(),1u);EXPECT_FALSE(g.failed());
}
TEST(PointLIOInputGuard, LidarProcessingDelayWithFreshImuDegrades) {
  point_lio::InputGuard g;g.imu(100.,0.);g.lidar(100.,0.);g.frame(true,100.,0.);g.poll(0.,100.,true,true);
  ASSERT_TRUE(g.imu(100.005,.65));EXPECT_TRUE(g.poll(.65,100.65,true,true));EXPECT_FALSE(g.failed());EXPECT_FALSE(g.healthy());
}
TEST(PointLIOInputGuard, ImuCallbackDelayWithFreshLidarDegrades) {
  point_lio::InputGuard g;g.imu(100.,0.);g.lidar(100.,0.);g.frame(true,100.,0.);g.poll(0.,100.,true,true);
  ASSERT_TRUE(g.lidar(100.1,.65));EXPECT_TRUE(g.poll(.65,100.65,true,true));EXPECT_FALSE(g.failed());EXPECT_FALSE(g.healthy());
}
TEST(PointLIOInputGuard, StartupSilenceStillRecollectsAndClearsTimestamps) {
  point_lio::InputGuard g;g.imu(100.,0.);g.lidar(100.,0.);g.poll(0.,100.,true,true);
  EXPECT_FALSE(g.poll(.65,100.65,true,true));EXPECT_FALSE(g.failed());EXPECT_TRUE(g.take_startup_restart());
  EXPECT_TRUE(g.imu(99.,.66));EXPECT_TRUE(g.lidar(99.,.66));EXPECT_STREQ(g.state(),"INITIALIZING");
}
TEST(PointLIOInputGuard, TimestampRollbackAndConstraintAgeRemainTerminalAfterTimeout) {
  for(bool rollback:{true,false}){
    point_lio::InputGuard g;g.imu(100.,0.);g.lidar(100.,0.);g.frame(true,100.,0.);g.poll(0.,100.,true,true);
    ASSERT_TRUE(g.poll(.65,100.65,true,true));
    if(rollback)EXPECT_FALSE(g.imu(99.,.66));else EXPECT_FALSE(g.frame(true,100.31,.66));
    EXPECT_TRUE(g.failed());EXPECT_FALSE(g.frame(true,100.1,.67));
  }
}

TEST(PointLIOInputGuard, GapEvidenceIsExactAndFirstFailureRemainsLatched) {
  for (bool imu : {false,true}) {
    point_lio::InputGuard g;
    const double first=1793777458.1234567, next=first+(imu?.25:.75);
    ASSERT_TRUE(g.lidar(first,0.));ASSERT_TRUE(g.imu(first,0.));
    ASSERT_TRUE(g.frame(true,first,0.));
    ASSERT_FALSE(imu?g.imu(next,1.):g.lidar(next,1.));
    const auto evidence=g.timestamp_gap();
    EXPECT_EQ(evidence.sensor,imu?"IMU":"LiDAR");
    EXPECT_DOUBLE_EQ(evidence.previous,first);EXPECT_DOUBLE_EQ(evidence.current,next);
    EXPECT_DOUBLE_EQ(evidence.limit,imu?g.max_imu_gap:g.max_lidar_gap);
    EXPECT_FALSE(g.lidar(next+10.,2.));EXPECT_FALSE(g.imu(next+10.,2.));
    EXPECT_EQ(g.timestamp_gap().sensor,evidence.sensor);
    EXPECT_DOUBLE_EQ(g.timestamp_gap().current,next);
    EXPECT_FALSE(g.take_startup_restart());EXPECT_TRUE(g.failed());
  }
}
TEST(PointLIOInputGuard, StartupRecollectionClearsOnlyOldGapEvidence) {
  point_lio::InputGuard g;ASSERT_TRUE(g.lidar(100.,0.));
  ASSERT_FALSE(g.lidar(101.,1.));ASSERT_EQ(g.timestamp_gap().sensor,"LiDAR");
  ASSERT_TRUE(g.take_startup_restart());EXPECT_TRUE(g.timestamp_gap().sensor.empty());
  ASSERT_TRUE(g.lidar(101.,1.1));ASSERT_TRUE(g.frame(true,101.,1.1));
  EXPECT_TRUE(g.healthy());EXPECT_TRUE(g.timestamp_gap().sensor.empty());
}
TEST(PointLIOInputGuard, NonGapFailuresDoNotInventGapEvidence) {
  for (bool rollback : {false,true}) {
    point_lio::InputGuard g;ASSERT_TRUE(g.lidar(100.,0.));ASSERT_TRUE(g.frame(true,100.,0.));
    ASSERT_FALSE(g.lidar(rollback?99.:std::numeric_limits<double>::quiet_NaN(),1.));
    EXPECT_TRUE(g.failed());EXPECT_TRUE(g.timestamp_gap().sensor.empty());
  }
}
