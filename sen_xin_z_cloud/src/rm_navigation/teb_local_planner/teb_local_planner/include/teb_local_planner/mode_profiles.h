#pragma once

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <array>
#include <vector>

namespace teb_local_planner {
// 每个模式都从启动时的完整基线生成，避免 A -> B 时遗留 A 的参数。
class ModeProfiles {
public:
  // Values存速度，footprint存轮廓，profiles存不同模式下的几个Value参数
  using Values = std::map<std::string, double>;
  std::map<std::string, Values> profiles;
  using Footprint = std::vector<std::array<double, 2>>;
  std::map<std::string, Footprint> footprints;

  // 定义函数，把一个 YAML 节点转换成 Footprint
  static Footprint parseFootprint(const YAML::Node &node) {
    // node 必须是列表。列表至少包含三个顶点。因为两个点无法围成有面积的多边形。
    if (!node.IsSequence() || node.size() < 3)
      throw std::runtime_error("footprint requires at least three [x,y] vertices");
    // 创建空的顶点列表，准备接收解析结果
    Footprint polygon;
    // 逐个遍历 node 中的顶点
    for (const auto &p : node) {
      // 每个顶点必须是恰好包含两个元素的列表
      if (!p.IsSequence() || p.size() != 2)
        throw std::runtime_error("footprint vertex must be [x,y]");
      // 把第一个元素作为 x，第二个元素作为 y
      const double x = p[0].as<double>(), y = p[1].as<double>();
      // 两个坐标都必须是有限数，不能是无穷大或 NaN。负坐标是允许的
      if (!std::isfinite(x) || !std::isfinite(y))
        throw std::runtime_error("footprint coordinates must be finite");
      polygon.push_back({x, y});
    }
    // 所有其他顶点及原点必须严格位于每条边的同一侧：拒绝凹形、自交、退化框。
    double sign = 0;
    for (size_t i = 0; i < polygon.size(); ++i) {
      const auto &a = polygon[i], &b = polygon[(i + 1) % polygon.size()];
      const double origin = (b[0]-a[0])*(-a[1]) - (b[1]-a[1])*(-a[0]);
      if (std::abs(origin) < 1e-10 || (sign != 0 && origin * sign <= 0))
        throw std::runtime_error("footprint must strictly contain the robot origin");
      sign = origin > 0 ? 1.0 : -1.0;
      for (size_t j = 0; j < polygon.size(); ++j) {
        if (j == i || j == (i + 1) % polygon.size()) continue;
        const auto &c = polygon[j];
        const double cross = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]);
        if (cross * sign <= 1e-10)
          throw std::runtime_error("footprint must be a nondegenerate convex polygon in boundary order");
      }
    }
    return polygon;
  }

  // root：解析后的 YAML 根节点，可以理解成整个 YAML 文件。
  // baseline：基础参数，作为所有模式的初始值。
  void load(const YAML::Node &root, const Values &baseline, const Footprint &base_footprint = {}) {
    // 创建一个临时容器 result，存放加载结果
    std::map<std::string, Values> result;
    std::map<std::string, Footprint> shapes;
    shapes.emplace("default", base_footprint);
    // 向 result 插入一个模式
    // 模式名：default
    // 参数表：baseline 的副本
    // 先放在临时变量里，是为了避免加载到一半出错时破坏原来的配置
    // 无论 YAML 里写了哪些模式，都先保留一个使用基础参数的 default 模式
    result.emplace("default", baseline);
    // 检查 YAML 的结构是否正确。
    // IsMap() 判断节点是不是“键值对结构”
    if (!root.IsMap() || !root["profiles"].IsMap())
      // 抛出异常后，当前函数立即停止正常执行，由外层代码决定如何处理
      throw std::runtime_error("mode YAML must contain a profiles mapping");
    // 逐个遍历 profiles 下的模式
    for (const auto &mode : root["profiles"]) {
      // 把第一个模式转换为string赋给name
      const auto name = mode.first.as<std::string>();
      // 检查这个模式是否合法，四种情况都会被拒绝
      // name.empty()	模式名为空
      // name == "default"	使用了保留名称 default
      // result.count(name)	这个模式名已经存在，重复定义
      // !mode.second.IsMap()	模式内容不是参数键值对
      if (name.empty() || name == "default" || result.count(name) || !mode.second.IsMap())
        throw std::runtime_error("invalid/duplicate/reserved mode name: " + name);
      // 复制一份完整的基础参数，作为当前模式的参数
      Values values = baseline;
      Footprint shape = base_footprint;
      // 内层循环：遍历当前模式里的每个参数
      for (const auto &entry : mode.second) {
        const auto key = entry.first.as<std::string>();
        if (key == "footprint") {
          shape = parseFootprint(entry.second);
          continue;
        }
        // 检查基础参数表里有没有这个参数。只有 baseline 中已经存在的参数，才允许在模式中修改
        if (!baseline.count(key)) throw std::runtime_error("unsupported mode parameter: " + key);
        // 把 YAML 中的参数值转换成 double
        // 如果写的是不能转换成数字的内容，转换过程就可能抛出异常。
        const double value = entry.second.as<double>();
        // 检查数值是否满足两个基本要求：必须是有限数，不能是无穷大或 NaN。不能小于 0。
        if (!std::isfinite(value) || value < 0.0)
          throw std::runtime_error("mode parameter must be finite and nonnegative: " + key);
        
        if ((key == "max_vel_x" || key == "max_vel_theta" || key.compare(0, 8, "acc_lim_") == 0)
            && value <= 0.0)
          throw std::runtime_error("mode parameter must be positive: " + key);
        // 横移能力是运动学配置；模式只调速度，不切换运动学模型。
        if (key == "max_vel_y" && ((value > 0.0) != (baseline.at(key) > 0.0)))
          throw std::runtime_error("mode cannot enable/disable holonomic motion");
        values[key] = value;
      }
      result.emplace(name, std::move(values));
      shapes.emplace(name, std::move(shape));
    }
    profiles = std::move(result);  // 整个文件验证通过后才提交。
    footprints = std::move(shapes);
  }
};
}  // namespace teb_local_planner
