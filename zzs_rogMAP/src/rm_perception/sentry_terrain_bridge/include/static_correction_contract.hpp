#pragma once
#include <algorithm>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>
namespace sentry_terrain_bridge {
using Grid = nav_msgs::msg::OccupancyGrid;
inline bool validGrid(const Grid &grid, bool mask) {
    const auto &info = grid.info;
    const auto &position = info.origin.position;
    const auto &orientation = info.origin.orientation;
    const size_t count = static_cast<size_t>(info.width) * info.height;
    return grid.header.frame_id == "map" &&
      grid.header.stamp.sec >= 0 && grid.header.stamp.nanosec < 1000000000U &&
      info.map_load_time.sec >= 0 && info.map_load_time.nanosec < 1000000000U &&
      info.width > 0 && info.height > 0 && count <= 4000000 &&
      grid.data.size() == count && std::isfinite(info.resolution) &&
      info.resolution > 0 && std::isfinite(position.x) &&
      std::isfinite(position.y) && position.z == 0.0 &&
      orientation.x == 0.0 && orientation.y == 0.0 &&
      orientation.z == 0.0 && orientation.w == 1.0 &&
      std::all_of(grid.data.begin(), grid.data.end(), [mask](int8_t value) {
        return mask ? (value == 0 || value == 100) :
                      (value >= -1 && value <= 100);
      });
  }

inline bool sameGeometry(const Grid &a, const Grid &b) {
    return a.header.frame_id == b.header.frame_id &&
      a.info.width == b.info.width && a.info.height == b.info.height &&
      a.info.resolution == b.info.resolution && a.info.origin == b.info.origin;
  }

inline bool clearsStatic(int raw,int mask) { return raw==100 && mask==100; }
}
