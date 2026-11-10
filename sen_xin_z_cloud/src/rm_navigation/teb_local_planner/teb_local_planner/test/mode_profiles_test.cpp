#include <teb_local_planner/mode_profiles.h>
#include <iostream>

using teb_local_planner::ModeProfiles;
void check(bool condition) {
  if (!condition) throw std::runtime_error("mode profile test failed");
}
int main() {
  ModeProfiles p;
  ModeProfiles::Values base{{"max_vel_x", 1.0}, {"max_vel_y", 1.0},
      {"acc_lim_x", 0.5}, {"weight_obstacle", 50.0}};
  p.load(YAML::Load("profiles: {slow: {max_vel_x: 0.2}, cautious: {weight_obstacle: 100}}"), base);
  check(p.profiles.at("slow").at("max_vel_x") == 0.2);
  // 切换到只覆盖权重的模式时，速度必须恢复基线而不是继承 slow。
  check(p.profiles.at("cautious").at("max_vel_x") == 1.0);
  check(p.profiles.at("default") == base);
  const ModeProfiles::Footprint original{{{0.3, 0.3}}, {{0.3, -0.3}}, {{-0.3, -0.3}}, {{-0.3, 0.3}}};
  p.load(YAML::Load("profiles: {slow: {max_vel_x: 0.2, footprint: [[0.2,0.2],[0.2,-0.2],[-0.2,-0.2],[-0.2,0.2]]}, cautious: {}}"), base, original);
  check(p.footprints.at("default") == original);
  check(p.footprints.at("cautious") == original);
  check(p.footprints.at("slow")[0][0] == 0.2);
  for (const auto &text : {
      "profiles: {bad: {robot_base_frame: map}}",
      "profiles: {bad: {max_vel_x: -1}}",
      "profiles: {bad: {max_vel_x: .nan}}",
      "profiles: {bad: {acc_lim_x: 0}}",
      "profiles: {bad: {max_vel_y: 0}}",
      "profiles: {default: {max_vel_x: 0.2}}",
      "profiles: {bad: {max_vel_x: 0.2}, bad: {max_vel_x: 0.3}}",
      "profiles: []"}) {
    bool rejected = false;
    try { p.load(YAML::Load(text), base); } catch (const std::exception &) { rejected = true; }
    check(rejected);
    check(p.profiles.at("slow").at("max_vel_x") == 0.2); // 失败不部分提交。
  }
  for (const auto &shape : {
      "[[0,0],[1,0]]", "[[.nan,1],[1,-1],[-1,-1],[-1,1]]",
      "[[1,1],[-1,-1],[1,-1],[-1,1]]", "[[1,1],[2,1],[2,2],[1,2]]",
      "[[1,1],[1,-1],[0,0],[-1,-1],[-1,1]]"}) {
    bool rejected = false;
    try { ModeProfiles::parseFootprint(YAML::Load(shape)); }
    catch (const std::exception &) { rejected = true; }
    check(rejected);
  }
  std::cout << "mode profile validation and baseline restoration passed\n";
}
