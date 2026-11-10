#include <gtest/gtest.h>
#include "cruise_logic.hpp"
using cruise::Logic;
using cruise::pi;
TEST(Cruise, CircleAndStopLatch) {
  Logic l;
  EXPECT_DOUBLE_EQ(l.update(0.), l.speed);
  l.found(false);
  EXPECT_DOUBLE_EQ(l.update(1.), l.speed);
  l.found(true);
  l.enemy(2.);
  EXPECT_DOUBLE_EQ(l.update(0.), 0.);
  l.lost(false);
  EXPECT_EQ(l.state, Logic::State::Found);
  l.lost(true);
  EXPECT_EQ(l.state, Logic::State::Circle);
  EXPECT_DOUBLE_EQ(l.update(0.), l.speed);
}
TEST(Cruise, MinimumTurningSpeedPreservesStop) {
  Logic l;
  l.speed = 0.7;
  l.min_speed = 0.2;
  l.gain = 3.0;
  l.enemy(0.);
  EXPECT_DOUBLE_EQ(l.update(-0.03), 0.2);
  EXPECT_DOUBLE_EQ(l.update(0.03), -0.2);
  EXPECT_DOUBLE_EQ(l.update(-1.0), 0.7);
  l.found(true);
  EXPECT_DOUBLE_EQ(l.update(-0.03), 0.0);
}
TEST(Cruise, SeekShortestAcrossPiAndSweepEndpoints) {
  Logic l;
  const double center = -179. * pi / 180.;
  l.enemy(center);
  EXPECT_GT(l.update(179. * pi / 180.), 0.);
  EXPECT_GT(l.update(center), 0.);
  EXPECT_EQ(l.state, Logic::State::Sweep);
  EXPECT_LT(l.update(cruise::wrap(center + pi/4)), 0.);
  EXPECT_GT(l.update(cruise::wrap(center - pi/4)), 0.);
  l.enemy(0.);
  EXPECT_EQ(l.state, Logic::State::SeekEnemy);
  EXPECT_LT(l.update(1.), 0.);
}
TEST(Cruise, SimulatedFeedbackStaysWithinSector) {
  Logic l;
  l.enemy(0.);
  double yaw = 0.;
  bool positive = false, negative = false;
  for (int i=0; i<6000; ++i) {
    const double speed = l.update(yaw);
    positive |= speed > 0.; negative |= speed < 0.;
    EXPECT_LE(std::abs(speed), l.speed);
    yaw += speed * 0.01;
    EXPECT_LE(std::abs(yaw), pi/4 + 0.005);
  }
  EXPECT_TRUE(positive && negative);
}
