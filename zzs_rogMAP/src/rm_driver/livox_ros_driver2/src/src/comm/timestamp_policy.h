#pragma once
#include <chrono>
#include <cstdint>
#include <limits>

namespace livox_ros {
// Keep ROS epoch timestamps, but do not let wall-clock corrections reorder
// unsynchronised LiDAR/IMU packets. Both streams use this one process anchor.
class MonotonicEpoch {
 public:
  MonotonicEpoch()
      : steady_(std::chrono::steady_clock::now()),
        epoch_(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()) {}
  uint64_t now() const {
    return epoch_ + std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now()-steady_).count();
  }
 private:
  std::chrono::steady_clock::time_point steady_;
  uint64_t epoch_;
};

inline bool relativePointTime(uint64_t point, uint64_t base, uint64_t max_span,
                              uint32_t &offset) {
  if (point < base || point-base > max_span ||
      point-base > std::numeric_limits<uint32_t>::max()) return false;
  offset = static_cast<uint32_t>(point-base);
  return true;
}
}  // namespace livox_ros
