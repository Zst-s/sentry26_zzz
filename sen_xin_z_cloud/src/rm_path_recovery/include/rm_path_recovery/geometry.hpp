#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rm_path_recovery
{
struct Point {double x, y;};
inline double length(Point p) {return std::hypot(p.x, p.y);}
inline Point subtract(Point a, Point b) {return {a.x-b.x, a.y-b.y};}
inline Point bodyVelocity(Point direction, double speed, double yaw)
{
  return {speed*(direction.x*std::cos(yaw)+direction.y*std::sin(yaw)),
    speed*(-direction.x*std::sin(yaw)+direction.y*std::cos(yaw))};
}

// Project onto a segment, then advance by arc length toward the path's end.
inline Point lookaheadPoint(
  const std::vector<Point> & path, Point robot, double ahead, double max_offset)
{
  if (path.size() < 2 || !std::isfinite(ahead) || ahead <= 0) {
    throw std::runtime_error("Empty/short path or invalid lookahead");
  }
  for (auto p : path) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      throw std::runtime_error("Non-finite path");
    }
  }
  double best = std::numeric_limits<double>::infinity();
  size_t segment = 0;
  Point projection{};
  for (size_t i=0; i+1<path.size(); ++i) {
    const auto d = subtract(path[i+1], path[i]);
    const double squared = d.x*d.x+d.y*d.y;
    if (squared < 1e-12) {continue;}
    const auto r = subtract(robot, path[i]);
    const double t = std::clamp((r.x*d.x+r.y*d.y)/squared, 0.0, 1.0);
    const Point p{path[i].x+t*d.x, path[i].y+t*d.y};
    const double distance = length(subtract(p, robot));
    if (distance < best) {best=distance; projection=p; segment=i;}
  }
  if (!std::isfinite(best) || best > max_offset) {
    throw std::runtime_error("Path degenerate or too far from robot");
  }
  for (size_t i=segment+1; i<path.size(); ++i) {
    const auto d = subtract(path[i], projection);
    const double distance = length(d);
    if (distance >= ahead && distance > 1e-9) {
      return {projection.x+d.x*ahead/distance, projection.y+d.y*ahead/distance};
    }
    ahead -= distance;
    projection=path[i];
  }
  return path.back();
}
}  // namespace rm_path_recovery
