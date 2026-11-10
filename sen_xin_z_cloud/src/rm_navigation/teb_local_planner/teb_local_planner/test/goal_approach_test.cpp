#include <gtest/gtest.h>
#include <limits>
#include "teb_local_planner/goal_approach.h"
using teb_local_planner::goalApproachScale;

TEST(GoalApproach, FormulaAndBoundary)
{
  EXPECT_DOUBLE_EQ(goalApproachScale(1, .5, .5, 1, 0), .5);
  EXPECT_DOUBLE_EQ(goalApproachScale(.5, .5, .5, 1, 0), .5);
  EXPECT_DOUBLE_EQ(goalApproachScale(.25, .5, .5, 1, 0), .25);
  EXPECT_DOUBLE_EQ(goalApproachScale(0, .5, .5, 1, 0), 0);
}
TEST(GoalApproach, HolonomicAndReverse)
{
  EXPECT_NEAR(goalApproachScale(.25, .5, .5, .3, .4), .5, 1e-12);
  EXPECT_NEAR(goalApproachScale(.25, .5, .5, -.3, -.4), .5, 1e-12);
}
TEST(GoalApproach, NeverAcceleratesOrBlocksPureRotation)
{
  EXPECT_DOUBLE_EQ(goalApproachScale(.25, .5, .5, .01, .01), 1);
  EXPECT_DOUBLE_EQ(goalApproachScale(0, .5, .5, 0, 0), 1);
}
TEST(GoalApproach, RejectsInvalidInputs)
{
  EXPECT_THROW(goalApproachScale(.1, 0, .5, 1, 0), std::invalid_argument);
  EXPECT_THROW(goalApproachScale(.1, .5, -1, 1, 0), std::invalid_argument);
  EXPECT_THROW(goalApproachScale(-1, .5, 1, 1, 0), std::invalid_argument);
  EXPECT_THROW(goalApproachScale(std::numeric_limits<double>::quiet_NaN(), .5, 1, 1, 0), std::invalid_argument);
}
