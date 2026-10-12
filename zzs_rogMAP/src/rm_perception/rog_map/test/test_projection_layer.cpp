#include <gtest/gtest.h>

#include <rog_map/projection_layer.hpp>

namespace {

rog_map::ColumnStats thinOccupiedColumn(double z)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 3;
  stats.occupied_count = 1;
  stats.occupied_z_index_min = 0;
  stats.occupied_z_index_max = 0;
  stats.occupied_z_min_abs = z;
  stats.occupied_z_max_abs = z;
  return stats;
}

uint8_t centerValue(rog_map::ProjectionLayerConfig config, double z)
{
  config.mask_filter_en = false;
  config.hysteresis_en = false;
  rog_map::ProjectionLayer layer;
  const auto scanner = [z](int x, int y) {
    if (x == 1 && y == 1) {
      return thinOccupiedColumn(z);
    }
    rog_map::ColumnStats free;
    free.observed_count = 2;
    return free;
  };
  layer.updateFull(3, 3, 0.05, Eigen::Vector2d::Zero(), 1.0, config, scanner);
  return layer.values().at(4);
}

}  // namespace

TEST(ProjectionLayer, ElevatedSparseReturnIsObstacleButGroundRemainsPassable)
{
  rog_map::ProjectionLayerConfig config;
  config.surface_height_delta_max = 0.25;
  config.elevated_surface_as_obstacle = true;
  config.elevated_surface_min_z_abs = 0.15;

  EXPECT_EQ(centerValue(config, -0.25), 0U);
  EXPECT_EQ(centerValue(config, 0.10), 0U);
  EXPECT_EQ(centerValue(config, 0.35), 254U);

  config.elevated_surface_as_obstacle = false;
  EXPECT_EQ(centerValue(config, 0.35), 0U);
}
