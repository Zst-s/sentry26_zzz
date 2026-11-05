#include <gtest/gtest.h>
#include "comm/timestamp_policy.h"

TEST(LivoxTimestampPolicy, RejectsNegativeWrappedAndOutageOffsets) {
  const uint64_t base=1792824638220775800ULL;
  uint32_t offset=42;
  EXPECT_FALSE(livox_ros::relativePointTime(base-410798336, base, 200000000, offset));
  EXPECT_FALSE(livox_ros::relativePointTime(base+2453425152, base, 200000000, offset));
  EXPECT_FALSE(livox_ros::relativePointTime(base+4294967296ULL, base, 5000000000ULL, offset));
  ASSERT_TRUE(livox_ros::relativePointTime(base+100447232, base, 200000000, offset));
  EXPECT_EQ(offset, 100447232U);
  ASSERT_TRUE(livox_ros::relativePointTime(base, base, 200000000, offset));
  EXPECT_EQ(offset, 0U);
}

TEST(LivoxTimestampPolicy, KeepsRosEpochAndMonotonicProgress) {
  livox_ros::MonotonicEpoch clock;
  auto first=clock.now();
  auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  EXPECT_LT(std::abs(static_cast<int64_t>(first)-wall), 100000000);
  for (int i=0; i<100; ++i) {
    auto next=clock.now(); EXPECT_GE(next, first); first=next;
  }
}
