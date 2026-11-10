#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "cruise_logic.hpp"

class CruiseController : public rclcpp::Node {
public:
  CruiseController() : Node("cruise_controller_node") {
    // 声明并读取参数
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    const auto lidar_frame = declare_parameter<std::string>("lidar_frame", "livox_frame");
    gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "gimbal");
    const auto xyz = declare_parameter<std::vector<double>>("lidar_to_gimbal_xyz", {0.,0.,0.});
    const auto rpy = declare_parameter<std::vector<double>>("lidar_to_gimbal_rpy", {0.,0.,0.});
    logic_.speed = declare_parameter<double>("cruise_speed", 0.4);
    logic_.min_speed = declare_parameter<double>("min_speed", 0.0);
    logic_.gain = declare_parameter<double>("yaw_gain", 1.5);
    logic_.tolerance = declare_parameter<double>("yaw_tolerance", 0.02);
    logic_.circle_direction = declare_parameter<int>("circle_direction", 1);
    const double frequency = declare_parameter<double>("control_frequency", 100.0);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.5);
    const auto enemy_topic = declare_parameter<std::string>("enemy_pose_topic", "/enemy_pose");
    const auto find_topic = declare_parameter<std::string>("find_topic", "/find_already");
    const auto lose_topic = declare_parameter<std::string>("lose_topic", "/lose_already");
    const auto output_topic = declare_parameter<std::string>("speed_topic", "/cruise_speed");
    // 判断数值是否为“有限的正数”
    auto positive = [](double x) { return std::isfinite(x) && x > 0.; };
    // 任何检查失败就抛出异常，停止构造节点
    if (!positive(logic_.speed) || !std::isfinite(logic_.min_speed) || logic_.min_speed < 0. ||
        logic_.min_speed > logic_.speed || !positive(logic_.gain) || !positive(logic_.tolerance) ||
        logic_.tolerance >= logic_.half_width || !positive(frequency) || frequency > 1000. ||
        !positive(tf_timeout_) || std::abs(logic_.circle_direction) != 1 ||
        xyz.size() != 3 || rpy.size() != 3 || map_frame_.empty() || lidar_frame.empty() ||
        gimbal_frame_.empty() || lidar_frame == gimbal_frame_ || map_frame_ == gimbal_frame_)
      throw std::invalid_argument("Invalid cruise settings, frames or extrinsics");
    // 额外判断雷达到云台的相对xyzrpy是不是有限的
    for (double v : xyz) if (!std::isfinite(v)) throw std::invalid_argument("Invalid xyz");
    for (double v : rpy) if (!std::isfinite(v)) throw std::invalid_argument("Invalid rpy");

    // 创建速度发布者，使用深度为 1 的历史队列，保留最新的一条消息
    speed_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(1));
    found_state_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/cruise/found_state", rclcpp::QoS(1).reliable().transient_local());
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    // 发布云台坐标系在雷达坐标系中的位置和姿态
    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now(); t.header.frame_id = lidar_frame; t.child_frame_id = gimbal_frame_;
    t.transform.translation.x = xyz[0]; t.transform.translation.y = xyz[1];
    t.transform.translation.z = xyz[2];
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    t.transform.rotation.x=q.x(); t.transform.rotation.y=q.y();
    t.transform.rotation.z=q.z(); t.transform.rotation.w=q.w();
    static_tf_->sendTransform(t);

    enemy_sub_ = create_subscription<geometry_msgs::msg::Pose>(enemy_topic, rclcpp::QoS(1),
      // 回调：把消息中的姿态四元数转换成 yaw。yaw 通过引用参数被函数写入；返回 bool，说明转换是否成功。 
      [this](geometry_msgs::msg::Pose::ConstSharedPtr msg) {
        double yaw;
        if (!quaternionYaw(msg->orientation, yaw)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid enemy quaternion");
          return;
        }
        // Pose 没有 header，本接口约定 orientation 已在 map 系。
        // 消息中的 orientation 必须已经表示 map 坐标系中的目标朝向
        logic_.enemy(yaw);
      });

    find_sub_ = create_subscription<std_msgs::msg::Bool>(find_topic, rclcpp::QoS(10),
      // find_topic收到true传入函数，切换状态为found；收到false不执行任何操作
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        logic_.found(msg->data);
        if (msg->data) {
          publishSpeed(0.); // 立即停止，不等定时器。
          publishFoundState();
          RCLCPP_INFO(get_logger(), "Received find_already=true; cruise stopped");
        }
      });
    lose_sub_ = create_subscription<std_msgs::msg::Bool>(lose_topic, rclcpp::QoS(10),
      // 收到 true 后切换回 Circle 状态，下次定时更新时恢复转圈
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        logic_.lost(msg->data);
        if (msg->data) {
          publishFoundState();
          RCLCPP_INFO(get_logger(), "Received lose_already=true; cruise resumed");
        }
      });
    publishFoundState();
    // 创建定时器，周期性调用 tick()
    // 频率 -> 周期 -> 秒 -> 纳秒
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1. / frequency)), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Cruise ready: %s -> %s, %.3f rad/s, output=%s",
        lidar_frame.c_str(), gimbal_frame_.c_str(), logic_.speed, output_topic.c_str());
  }
  // 提供公开停止接口，发布一次零速度。
  // 注意它只发布消息，没有修改状态或取消定时器；如果节点还在继续执行定时回调，后续仍可能发布运动速度。
  void stop() { publishSpeed(0.); }

private:
  // 四元数转 yaw 的辅助函数,返回是否成功
  static bool quaternionYaw(const geometry_msgs::msg::Quaternion &q, double &yaw) {

    const double norm = std::hypot(std::hypot(q.x,q.y), std::hypot(q.z,q.w));
    // 如果长度不是有限数，或者小到接近零，就认为四元数无效
    if (!std::isfinite(norm) || norm < 1e-9) return false;
    const double x=q.x/norm, y=q.y/norm, z=q.z/norm, w=q.w/norm;
    yaw = std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
    return std::isfinite(yaw);
  }
  void publishSpeed(double omega) {
    geometry_msgs::msg::Twist msg;
    msg.angular.z = omega; // 相对于 gimbal 的 +Z 转轴，rad/s；其他分量为零。
    speed_pub_->publish(msg);
  }
  void publishFoundState() {
    std_msgs::msg::Bool msg;
    msg.data = logic_.state == cruise::Logic::State::Found;
    found_state_pub_->publish(msg);
  }
  void tick() {
    // 如果已经找到目标，就持续发布零速度，然后结束本次更新
    if (logic_.state == cruise::Logic::State::Found) { publishSpeed(0.); return; }
    // 接下来查询 TF，查询失败可能抛出异常，因此放在 try 中
    try {
      // 查询 gimbal 在 map 中的位置和姿态。
      const auto tf = buffer_->lookupTransform(map_frame_, gimbal_frame_, tf2::TimePointZero);
      // 计算 TF 数据年龄
      const double age = (now() - rclcpp::Time(tf.header.stamp, get_clock()->get_clock_type())).seconds();
      double yaw;
      // 出现 age > tf_timeout_ 或 TF 时间比当前时间超前超过 0.1 秒 或 四元数转换失败 就停止
      if (age > tf_timeout_ || age < -0.1 || !quaternionYaw(tf.transform.rotation, yaw)) {
        publishSpeed(0.);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Gimbal TF invalid/stale: %.3fs; stopped", age);
        return;
      }
      // TF 正常时把当前姿态的yaw传入update并发布速度
      publishSpeed(logic_.update(yaw));
    } catch (const tf2::TransformException &e) {
      publishSpeed(0.);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for gimbal TF: %s", e.what());
    }
  }
  cruise::Logic logic_;
  std::string map_frame_, gimbal_frame_;
  double tf_timeout_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr enemy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr find_sub_, lose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr found_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CruiseController>();
    rclcpp::spin(node);
    // spin() 返回后，如果 ROS2 上下文仍有效，就发布一次零速度。
    if (rclcpp::ok()) node->stop();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("cruise"), "%s", e.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  // 正常结束时，关闭仍有效的 ROS2 上下文，并返回 0
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
    
