#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <iostream>
using std::cout;
using std::endl;
#include "ivox/ivox3d.h"

TEST(IVoxFailedQuery, DoesNotReusePreviousNeighbours) {
  using Map = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, pcl::PointXYZI>;
  Map map{Map::Options{}};
  Map::PointVector points;
  pcl::PointXYZI point;
  for (int i=0; i<5; ++i) {
    point.x=.01*i; point.y=0; point.z=0; points.push_back(point);
  }
  map.AddPoints(points);
  Map::PointVector nearest;
  point.x=0;
  ASSERT_TRUE(map.GetClosestPoint(point,nearest,5));
  ASSERT_EQ(nearest.size(),5U);
  point.x=1000; point.y=1000; point.z=1000;
  EXPECT_FALSE(map.GetClosestPoint(point,nearest,5));
  EXPECT_TRUE(nearest.empty());
}
