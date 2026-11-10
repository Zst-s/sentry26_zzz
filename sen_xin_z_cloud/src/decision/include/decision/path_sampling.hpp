#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace decision
{
struct PathPoint
{
  double x;
  double y;
  double z;
};

// Sample the entire XY polyline by arc length, not by input point index.
// Sparse segments are interpolated; both endpoints are retained exactly.
inline std::vector<PathPoint> sample_path(
  const std::vector<PathPoint> & input, std::size_t max_points = 50)
{
  if (max_points < 2) {
    throw std::invalid_argument("path sampling requires at least two slots");
  }
  if (input.empty()) return {};
  std::vector<PathPoint> points;
  std::vector<double> lengths;
  for (const auto & point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::invalid_argument("path contains non-finite coordinates");
    }
    if (points.empty()) {
      points.push_back(point);
      lengths.push_back(0.0);
      continue;
    }
    const double distance = std::hypot(point.x - points.back().x, point.y - points.back().y);
    if (distance == 0.0) continue;
    const double total = lengths.back() + distance;
    if (!std::isfinite(total)) throw std::invalid_argument("path length overflow");
    points.push_back(point);
    lengths.push_back(total);
  }
  if (points.size() == 1) return {input.front()};

  std::vector<PathPoint> result;
  result.reserve(max_points);
  result.push_back(input.front());
  std::size_t segment = 1;
  for (std::size_t i = 1; i + 1 < max_points; ++i) {
    const double target = lengths.back() * (static_cast<double>(i) / (max_points - 1));
    while (segment + 1 < lengths.size() && lengths[segment] < target) ++segment;
    const double ratio = (target - lengths[segment - 1]) /
      (lengths[segment] - lengths[segment - 1]);
    const auto & a = points[segment - 1];
    const auto & b = points[segment];
    result.push_back({a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y),
      a.z + ratio * (b.z - a.z)});
  }
  result.push_back(input.back());
  return result;
}

inline bool path_delta_fits(int dx, int dy)
{
  return dx >= -128 && dx <= 127 && dy >= -128 && dy <= 127;
}
}  // namespace decision
