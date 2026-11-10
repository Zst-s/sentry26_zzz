#include <gtest/gtest.h>
#include <teb_local_planner/shortest_rotation.h>
#include <teb_local_planner/timed_elastic_band.h>

namespace teb_local_planner
{
namespace
{
geometry_msgs::msg::PoseStamped pose(double x, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

double totalRotation(const TimedElasticBand& teb)
{
  double result = 0.0;
  for (int i = 1; i < teb.sizePoses(); ++i)
    result += std::abs(g2o::normalize_theta(teb.Pose(i).theta() - teb.Pose(i - 1).theta()));
  return result;
}

TEST(ShortestRotation, Logged304DegreeBranchBecomes56Degrees)
{
  // Reproduce cycle 200: intermediate reference yaws are zero.
  const std::vector<geometry_msgs::msg::PoseStamped> plan = {
    pose(0.0, 2.29348621), pose(0.1, 0.0), pose(0.2, 0.0),
    pose(0.3, 0.0), pose(0.4, -3.01187491)};
  TimedElasticBand legacy, shortest;
  ASSERT_TRUE(legacy.initTrajectoryToGoal(plan, 2.5, 1.5));
  const auto seed = makeShortestRotationPlan(plan);
  ASSERT_TRUE(shortest.initTrajectoryToGoal(seed, 2.5, 1.5));
  EXPECT_NEAR(totalRotation(legacy), 5.30536112, 1e-8);
  const double expected = 2 * M_PI - 5.30536112;
  EXPECT_NEAR(totalRotation(shortest), expected, 1e-8);
  for (size_t i = 0; i < plan.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(seed[i].pose.position.x, plan[i].pose.position.x);
    EXPECT_DOUBLE_EQ(seed[i].pose.position.y, plan[i].pose.position.y);
    EXPECT_EQ(seed[i].header.frame_id, plan[i].header.frame_id);
  }
  EXPECT_EQ(seed.front(), plan.front());
  EXPECT_EQ(seed.back(), plan.back());
  // TEB's actual split/merge operation must preserve the short angular branch.
  shortest.autoResize(0.3, 0.06, 3, 500, false);
  EXPECT_NEAR(totalRotation(shortest), expected, 1e-8);
  for (int i = 0; i < shortest.sizeTimeDiffs(); ++i)
    EXPECT_GT(shortest.TimeDiff(i), 0.0);
}

TEST(ShortestRotation, WrapsBothDirectionsAcrossPi)
{
  for (double direction : {-1.0, 1.0})
  {
    const auto seed = makeShortestRotationPlan({
      pose(0.0, direction * 179.0 * M_PI / 180.0), pose(0.5, 0.0),
      pose(1.0, -direction * 179.0 * M_PI / 180.0)});
    TimedElasticBand teb;
    ASSERT_TRUE(teb.initTrajectoryToGoal(seed, 2.5, 1.5));
    EXPECT_NEAR(totalRotation(teb), 2.0 * M_PI / 180.0, 1e-8);
    EXPECT_GT(direction * g2o::normalize_theta(teb.Pose(1).theta() - teb.Pose(0).theta()), 0.0);
  }
}

TEST(ShortestRotation, UsesArcLengthAndHandlesDuplicatePositions)
{
  const auto seed = makeShortestRotationPlan({pose(0, 0), pose(0.1, -2), pose(0.1, -2), pose(1, 1)});
  EXPECT_NEAR(PoseSE2(seed[1].pose).theta(), 0.1, 1e-8);
  EXPECT_NEAR(PoseSE2(seed[2].pose).theta(), 0.1, 1e-8);
}

TEST(ShortestRotation, InPlaceRotationAndSmallInputs)
{
  const auto seed = makeShortestRotationPlan({pose(0, 2.9), pose(0, 0), pose(0, -2.9)});
  EXPECT_NEAR(std::abs(PoseSE2(seed[1].pose).theta()), M_PI, 1e-8);
  EXPECT_TRUE(makeShortestRotationPlan({}).empty());
  const std::vector<geometry_msgs::msg::PoseStamped> single{pose(0, 1)};
  EXPECT_EQ(makeShortestRotationPlan(single), single);
  const std::vector<geometry_msgs::msg::PoseStamped> pair{pose(0, 2.9), pose(1, -2.9)};
  EXPECT_EQ(makeShortestRotationPlan(pair), pair);
  TimedElasticBand teb;
  ASSERT_TRUE(teb.initTrajectoryToGoal(pair, 2.5, 1.5));
  EXPECT_NEAR(totalRotation(teb), 2 * M_PI - 5.8, 1e-8);
}
} // namespace
} // namespace teb_local_planner
