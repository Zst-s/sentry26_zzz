#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <chrono>
#include <cstdlib>

using namespace std;
using std::placeholders::_1;

// 全局变量：检测到 "Creating bond timer" 的次数
uint8_t bond_timer_count = 0;

class RosoutMonitor : public rclcpp::Node
{
public:
    RosoutMonitor() : Node("rosout_node"), start_time_(std::chrono::steady_clock::now())
    {
        // 发布话题，当 bond_timer_count == 2 时发布 1
        nav_ready_pub = this->create_publisher<std_msgs::msg::UInt8>("/nav_ready", 10);

        // 使用 volatile QoS 订阅 /rosout，避免接收历史缓存消息
        auto qos = rclcpp::QoS(100);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
        rosout_sub = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", qos, std::bind(&RosoutMonitor::rosout_callback, this, _1));

        // 创建定时器，每秒检查一次超时
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&RosoutMonitor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "rosout_monitor_node 已启动，等待检测 'Creating bond timer'...");
    }

private:
    // rosout 话题的回调函数，监控终端输出
    void rosout_callback(const rcl_interfaces::msg::Log::SharedPtr msg)
    {
        // 启动后延迟一小段时间再开始计数，避免启动阶段误触发
        const auto now = std::chrono::steady_clock::now();
        // if (now - start_time_ < std::chrono::seconds(1))
        // {
        //     return;
        // }

        // 更严格过滤：仅统计 lifecycle 相关 logger 的精确日志
        const bool is_bond_log = (msg->msg.find("Creating bond timer...") != std::string::npos);
        if (is_bond_log  && bond_timer_count < 2)
        {
            const auto debounce_window = std::chrono::milliseconds(500);
            if (msg->name == last_logger_name_ && (now - last_bond_time_) < debounce_window)
            {
                return;  // 去抖：避免同一 logger 在短时间内重复计数
            }
            last_logger_name_ = msg->name;
            last_bond_time_ = now;

            bond_timer_count++;
            RCLCPP_INFO(this->get_logger(), "检测到 'Creating bond timer'，当前计数: %d", bond_timer_count);

            // 当计数等于 2 时，发布 1
            if (bond_timer_count == 2)
            {
                std_msgs::msg::UInt8 ready_msg;
                ready_msg.data = 1;

                std::this_thread::sleep_for(std::chrono::seconds(2));
                
                nav_ready_pub->publish(ready_msg);
                RCLCPP_INFO(this->get_logger(), "导航就绪！已发布 /nav_ready = 1");
                nav_ready_published = true;
            }
        }
    }

    // 定时器回调函数，检查超时
    void timer_callback()
    {
        if (nav_ready_published)
        {
            rclcpp::shutdown();  // 已经成功，不再检查
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        double elapsed_sec = std::chrono::duration<double>(elapsed).count();

        RCLCPP_INFO(this->get_logger(), "已等待 %.0f 秒，bond_timer_count = %d", elapsed_sec, bond_timer_count);

        // 25 秒超时检查
        if (elapsed_sec >= 25.0 && bond_timer_count < 2)
        {
            RCLCPP_ERROR(this->get_logger(), "25秒超时！bond_timer_count = %d，准备重启...", bond_timer_count);
            
            // 关闭所有终端并重新运行 nav.sh
            // 使用 pkill 关闭所有 gnome-terminal 启动的进程
            std::system("pkill -f 'ros2 launch rm_nav_bringup'");
            std::system("pkill -f 'ros2 run decision trans_node'");
            std::system("pkill -f 'ros2 run rm_bt'");
            std::system("pkill -f 'ros2 run py_tools publish_initial_pose'");
            
            // 等待进程关闭
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // 重新运行 nav.sh（在后台）
            std::system("cd /home/doublegun/sen_xin && bash runn.sh &");
            
            RCLCPP_INFO(this->get_logger(), "已重新启动 runn.sh");
            
            // 关闭当前节点
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr nav_ready_pub;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_bond_time_{std::chrono::steady_clock::time_point::min()};
    std::string last_logger_name_;
    bool nav_ready_published = false;
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosoutMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
