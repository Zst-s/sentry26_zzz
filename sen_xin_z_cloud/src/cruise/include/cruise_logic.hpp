#pragma once
#include <algorithm>
#include <cmath>
namespace cruise {
constexpr double pi = 3.14159265358979323846;
// atan2带正负检查的arctan（前/后）
// sina/cosa=tana，再arctan（tana），起到把角度转换到-pi到pi内
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
class Logic {
public:
  // Circle	持续转圈
  // SeekEnemy	转向收到的敌人方向
  // Sweep	在敌人方向两侧来回扫描
  // Found	已确认找到目标，停止输出旋转速度
  enum class State { Circle, SeekEnemy, Sweep, Found };
  // speed	0.4	转圈速度，以及转向控制的速度上限
  // min_speed	0.0	转向时的最小非零角速度
  // gain	1.5	根据角度误差计算速度的比例系数
  // tolerance	0.02	判断是否对准的角度容差，约 1.15°
  // half_width	π/4	扫描中心到单侧边界的角度，即 45°
  double speed{0.4}, min_speed{0.0}, gain{1.5}, tolerance{0.02}, half_width{pi/4};
  // 实际代码没有限制它只能取这两个值；如果设为 2，后面的转圈速度也会乘以 2
  int circle_direction{1};
  // 初始状态设置为持续转圈。
  State state{State::Circle};
  // 定义函数，接收敌人的方向
  // 收到的为敌人位置在云台坐标系下计算得到的yaw
  void enemy(double yaw) {
    // 状态为found或yaw不为有限数，直接返回
    if (state == State::Found || !std::isfinite(yaw)) return;
    // 敌人相对与云台原点的角度归一化后设为巡航中心
    center_ = wrap(yaw); 
    // 切换状态机
    state = State::SeekEnemy; 
    // 设置后续扫描先去正侧边界
    side_ = 1;
  }
  // 传入true切换状态,传入false没有操作
  // 代码本身没有判断敌人是否真的被发现或丢失,通过接收外部通知间接
  void found(bool value) { if (value) state = State::Found; }
  void lost(bool value) { if (value) state = State::Circle; }

  // 传入机器人当前朝向，返回当前应该使用的旋转速度
  double update(double yaw) {
    // 如果已经找到目标，或者当前朝向无效，就返回 0.0。
    if (state == State::Found || !std::isfinite(yaw)) return 0.;
    // 转圈状态下，直接返回固定速度
    if (state == State::Circle) return circle_direction * speed;

    if (state == State::SeekEnemy) {
      // 计算目标方向和当前方向之间的最短角度误差
      double error = wrap(center_ - yaw);
      // 如果误差绝对值仍然大于容差，就调用控制函数计算速度，并立即返回
      if (std::abs(error) > tolerance) return control(error);
      // 小于等于容差,切换为sweep,继续执行下面的部分
      state = State::Sweep;
    }
    // 计算当前扫描边界与机器人朝向的差角
    double error = wrap(center_ + side_ * half_width - yaw);
    if (std::abs(error) <= tolerance) {
      // 把扫描目标切到另一侧
      side_ = -side_; 
      // 立即重新计算差角
      error = wrap(center_ + side_ * half_width - yaw);
    }
    // 计算角速度
    return control(error);
  }
private:
  double center_{0.};
  int side_{1};
  // 转向时把非零角速度限制在最小值和最大值之间；停止分支仍返回零。
  double control(double error) const {
    if (error == 0.) return 0.;
    return std::copysign(std::max(min_speed, std::min(speed, std::abs(gain * error))), error);
  }
};
} // namespace cruise
