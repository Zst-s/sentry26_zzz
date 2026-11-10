#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PathVisionBroadcaster : public rclcpp::Node
{
public:
  PathVisionBroadcaster() : Node("path_vision_broadcaster")
  {
    // 创建静态变换广播器
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTF();
  }

private:
  void publishStaticTF()
  {
    geometry_msgs::msg::TransformStamped t;

    // 时间戳和父子坐标系
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";   // 父坐标系
    // 此坐标系的 (0, 0) 就是裁判小地图左下角。
    t.child_frame_id = "referee_map";

    // 平移 (单位：米)
    t.transform.translation.x = -10.90;
    t.transform.translation.y = -5.49;
    t.transform.translation.z = 0.0;

    // 旋转：使用四元数。这里用欧拉角转四元数，绕 X 轴旋转 180 度
    tf2::Quaternion q;
    // constexpr double PI = 3.14159265358979323846;
    q.setRPY(0.0, 0.0, 0.0);  // roll, pitch, yaw (弧度)

    t.transform.rotation = tf2::toMsg(q);

    // 发布静态变换
    static_broadcaster_->sendTransform(t);

    // RCLCPP_INFO(this->get_logger(),
    //             "Published static transform: %s -> %s",
    //             t.header.frame_id.c_str(),
    //             t.child_frame_id.c_str());
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathVisionBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
