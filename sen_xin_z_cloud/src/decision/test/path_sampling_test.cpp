#include "decision/path_sampling.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

bool near(double a, double b)
{
  return std::abs(a - b) < 1e-9;
}
}

int main()
{
  using decision::PathPoint;
  using decision::sample_path;
  try {
    require(sample_path({}).empty(), "empty path");
    require(sample_path({{1, 2, 0}}).size() == 1, "single point");
    require(sample_path({{1, 2, 0}, {1, 2, 0}}).size() == 1, "zero length path");

    // Uneven input spacing and duplicates must not bias sampling to the first points.
    const auto line = sample_path({{0, 0, 0}, {0, 0, 0}, {0.001, 0, 0}, {49, 0, 0}});
    require(line.size() == 50, "point limit");
    for (std::size_t i = 0; i < line.size(); ++i) {
      require(near(line[i].x, static_cast<double>(i)), "arc length spacing");
    }

    std::vector<PathPoint> dense;
    for (int i = 0; i <= 1000; ++i) dense.push_back({i * 0.01, 2, 0});
    const auto full = sample_path(dense);
    require(near(full.front().x, 0) && near(full.back().x, 10), "full path endpoints");

    const auto corner = sample_path({{0, 0, 0}, {2, 0, 0}, {2, 2, 0}}, 5);
    require(near(corner[2].x, 2) && near(corner[2].y, 0), "follow polyline not endpoint chord");
    require(near(corner[3].x, 2) && near(corner[3].y, 1), "second leg");
    const auto loop = sample_path({{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 0, 0}});
    require(loop.size() == 50 && loop[10].x > 0, "closed path must retain its length");

    // Sparse global paths must be interpolated, not encoded as an overflowing single delta.
    const auto sparse = sample_path({{0, 9.65, 0}, {17.15, 0, 0}});
    int x = 0;
    int y = 150;
    for (std::size_t i = 1; i < sparse.size(); ++i) {
      const int next_x = static_cast<int>(std::lround(sparse[i].x * (28.0 / 17.15) * 10));
      const int next_y = static_cast<int>(std::lround(sparse[i].y * (15.0 / 9.65) * 10));
      require(decision::path_delta_fits(next_x - x, next_y - y), "scaled dm delta range");
      x += next_x - x;
      y += next_y - y;
    }
    require(x == 280 && y == 0, "decoded endpoint after scaling and quantization");
    require(decision::path_delta_fits(-128, 127), "int8 boundary");
    require(!decision::path_delta_fits(128, 0), "reject positive overflow");
    require(!decision::path_delta_fits(0, -129), "reject negative overflow");
    const auto huge = sample_path({{0, 0, 0}, {1000, 0, 0}});
    require(!decision::path_delta_fits(static_cast<int>(std::lround(huge[1].x * 10)), 0),
      "unrepresentable long path must not silently clip");

    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
      bool rejected = false;
      try { sample_path({{0, 0, 0}, {invalid, 0, 0}}); }
      catch (const std::invalid_argument &) { rejected = true; }
      require(rejected, "reject invalid coordinates");
    }
    bool rejected = false;
    try { sample_path({{0, 0, 0}}, 1); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "reject insufficient slots");
    std::cout << "path_sampling_test: all checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "path_sampling_test: " << error.what() << '\n';
    return 1;
  }
}
