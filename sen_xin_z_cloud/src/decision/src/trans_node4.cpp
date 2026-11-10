#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include <iostream>
#include <rclcpp_action/rclcpp_action.hpp>
#include "base_interfaces_demo/msg/robot.hpp"
#include "base_interfaces_demo/msg/control.hpp"
#include "rm_decision_interfaces/msg/both_camp_hp.hpp"
#include "rm_decision_interfaces/msg/enemy_detected.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "decision/crc8.hpp"
#include "decision/path_sampling.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
// #include "rcl_interfaces/msg/log.hpp"

using namespace std;
using std::placeholders::_1;

#define sBUFFERSIZE 147                  // send buffer size 串口发送缓存长度
#define rBUFFERSIZE 30                  // receive buffer size 串口接收缓存长度

unsigned char r_buffer[rBUFFERSIZE];     // 接收缓存
unsigned char after_buffer[rBUFFERSIZE]; // 处理后的数组

// 联合体,用于浮点数与16进制的快速转换
typedef union
{
    unsigned char cvalue[4];
    float fvalue;
} float_union;

// 创建一个serial类
serial::Serial ser;

// 保护串口操作的互斥量，防止并发读写导致异常未捕获
static std::mutex ser_mutex;

// 全局串口状态，供主循环与异常处理共享
static std::atomic<bool> serial_open(false);

// 流式接收缓存，用于粘包/拆包
// (moved recv buffer into the SerialChassis instance for thread-safety)

int arrival;
uint8_t target_stance;
uint8_t if_guodong = 1;
int bt_game_status = 0;
uint16_t bt_remain_time = 0;

int nav_ready = 0;
int super_ele = 0;
int if_qifu = 0;
int intensify = 0;

class SerialChassis : public rclcpp::Node
{
public:
    SerialChassis() : Node("serial_chassis_node")
    {
        serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        if (serial_port_.empty()) {
            throw std::invalid_argument("serial_port must not be empty");
        }
        RCLCPP_INFO(this->get_logger(), "串口设备：%s", serial_port_.c_str());
        // 从 YAML/启动参数读取；修改后重启节点即可，无需重新编译。
        rcl_interfaces::msg::ParameterDescriptor sender_descriptor;
        sender_descriptor.description = "0x0307 sender ID: red sentry=7, blue sentry=107; must match referee robot_id";
        sender_descriptor.read_only = true;  // 本参数启动时读取，不支持运行中改值。
        const int configured_sender_id = this->declare_parameter<int>("sender_id", 7, sender_descriptor);
        if (configured_sender_id < 1 || configured_sender_id > UINT16_MAX) {
            throw std::invalid_argument("sender_id must be an integer in [1, 65535]");
        }
        sender_id_ = static_cast<uint16_t>(configured_sender_id);
        RCLCPP_INFO(this->get_logger(), "0x0307 sender_id=%u (restart node after changing YAML)",
                    static_cast<unsigned int>(sender_id_));
        target_stance_sub = this->create_subscription<std_msgs::msg::UInt8>("/target_stance", 10, std::bind(&SerialChassis::target_stance_callback, this, _1));
        super_ele_sub = this->create_subscription<std_msgs::msg::UInt8>("/super_ele", 10, std::bind(&SerialChassis::super_ele_callback, this, _1));
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&SerialChassis::cmd_vel_callback, this, _1));
        cruise_speed_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cruise_speed", 10, std::bind(&SerialChassis::cruise_speed_callback, this, _1));
        if_qifu_sub = this->create_subscription<std_msgs::msg::UInt8>("/if_qifu", 10, std::bind(&SerialChassis::if_qifu_callback, this, _1));
        intensify_sub = this->create_subscription<std_msgs::msg::UInt8>("/intensify", 10, std::bind(&SerialChassis::intensify_callback, this, _1));

        RobotStatus_pub = this->create_publisher<rm_decision_interfaces::msg::RobotStatus>("/robot_status", 10);
        enemy_detected_pub = this->create_publisher<std_msgs::msg::UInt8>("/enemy_detected", 10);
        game_status_pub = this->create_publisher<std_msgs::msg::UInt8>("/game_status", 10);
        tree_num_pub = this->create_publisher<std_msgs::msg::UInt8>("/tree_num", 10);
        outpost_if_pub = this->create_publisher<std_msgs::msg::UInt8>("/outpost_if", 10);
        fort_if_pub = this->create_publisher<std_msgs::msg::UInt8>("/fort_if", 10);
        midhigh_if_pub = this->create_publisher<std_msgs::msg::UInt8>("/midhigh_if", 10);
        remain_time_pub = this->create_publisher<std_msgs::msg::UInt16>("/remain_time", 10);

        if_guodong_sub = this->create_subscription<std_msgs::msg::UInt8>("/if_guodong", rclcpp::QoS(1).reliable().transient_local(), std::bind(&SerialChassis::if_guodong_callback, this, _1));
        nav_ready_sub = this->create_subscription<std_msgs::msg::UInt8>("/nav_ready", 10, std::bind(&SerialChassis::nav_ready_callback, this, _1));
        bt_status_sub = this->create_subscription<rm_decision_interfaces::msg::RobotStatus>("bt_status", 10, std::bind(&SerialChassis::bt_status_callback, this, _1));
        bt_game_status_sub = this->create_subscription<std_msgs::msg::UInt8>("bt_game_status", 10, std::bind(&SerialChassis::bt_game_status_callback, this, _1));
        bt_remain_time_sub = this->create_subscription<std_msgs::msg::UInt16>("bt_remain_time", 10, std::bind(&SerialChassis::bt_remain_time_callback, this, _1));
        path_vision_sub = this->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&SerialChassis::path_vision_callback, this, _1));
        goal_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "navigation_goals", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(),
            std::bind(&SerialChassis::goal_pose_callback, this, _1));
        navigation_route_active_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/navigation_route_active", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                navigation_route_active_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "0x0307 route source: %s",
                    navigation_route_active_ ? "cached patrol route" : "Nav2 /plan");
            });

        // referee_map 的原点由静态 TF 定义为裁判小地图左下角。
        referee_frame_ = this->declare_parameter<std::string>("referee_frame", "referee_map");
        // 用于排查裁判路径包。默认开启，每秒打印一次，不影响实际发送内容。
        // 注意：这里才是 ROS 参数的默认值；只改成员变量初值会被此处覆盖。
        debug_referee_0307_ = this->declare_parameter<bool>("debug_referee_0307", true);
        debug_referee_0307_period_ms_ = std::max(
            1, static_cast<int>(this->declare_parameter<int>("debug_referee_0307_period_ms", 1000)));
        RCLCPP_INFO(this->get_logger(), "0x0307 terminal debug: %s, period: %d ms",
                    debug_referee_0307_ ? "ON" : "OFF", debug_referee_0307_period_ms_);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        mid_if_pub = this->create_publisher<std_msgs::msg::UInt8>("/mid_if", 10);        //RMUL
        both_camp_hp_pub = this->create_publisher<rm_decision_interfaces::msg::BothCampHp>("/both_camp_hp", 10);

        // 串口打包周期为25毫秒；全局路径采样在路径回调中完成。
        timer_ = this->create_wall_timer(std::chrono::milliseconds(25), std::bind(&SerialChassis::timer_callback, this));

        // 保护代码
        try
        {
            ser.setPort(serial_port_); // 与重连使用同一个设备参数。
            ser.setBaudrate(115200);     // 设置串口通信的波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open(); // 打开串口
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开端口！！！ 将在运行时重试打开串口");
            // Do not shutdown here; allow main loop to attempt reconnects
        }

        // 判断是否初始化成功（如果未打开，不在此处退出，主循环会尝试重连）
        if (ser.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "串行端口已初始化");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "串口未打开，构造时将不退出，主循环会重试打开串口");
        }

        // start background reader thread
        running_.store(true);
        read_thread_ = std::thread(&SerialChassis::read_loop, this);
    }

    ~SerialChassis()
    {
        // stop background thread and clean up
        running_.store(false);
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        try {
            std::lock_guard<std::mutex> lk(ser_mutex);
            if (ser.isOpen()) ser.close();
        } catch (...) {}
    }

    // background reader thread
    void read_loop()
    {
        using namespace std::chrono_literals;
        while (rclcpp::ok() && running_.load()) {
            // ensure serial is open
            if (!ser.isOpen()) {
                try {
                    std::lock_guard<std::mutex> lk(ser_mutex);
                    ser.setPort(serial_port_);
                    ser.setBaudrate(115200);
                    serial::Timeout to = serial::Timeout::simpleTimeout(500);
                    ser.setTimeout(to);
                    ser.open();
                    if (ser.isOpen()) {
                        RCLCPP_INFO(this->get_logger(), "串口已打开（read_loop 恢复连接）");
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "尝试在 read_loop 中打开串口失败: %s", e.what());
                    std::this_thread::sleep_for(200ms);
                    continue;
                }
            }

            // read available bytes (try to read at least 1 byte so we don't rely on available())
            try {
                size_t avail = 0;
                try { avail = ser.available(); } catch (...) { avail = 0; }
                size_t to_read = avail ? std::min(avail, (size_t)256) : 1; // at least read 1
                std::vector<unsigned char> temp(to_read);
                size_t nread = 0;
                {
                    std::lock_guard<std::mutex> lk(ser_mutex);
                    nread = ser.read(temp.data(), to_read);
                }
                if (nread > 0) {
                    std::lock_guard<std::mutex> lk(recv_mutex_);
                    recv_buffer_.insert(recv_buffer_.end(), temp.begin(), temp.begin() + nread);
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "串口读取异常 (read_loop): %s", e.what());
                try { std::lock_guard<std::mutex> lk(ser_mutex); ser.close(); } catch (...) {}
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }

            // process frames from recv_buffer_
            {
                std::lock_guard<std::mutex> lk(recv_mutex_);
                // look for full frames
                while (recv_buffer_.size() >= rBUFFERSIZE) {
                    auto it = std::find(recv_buffer_.begin(), recv_buffer_.end(), 0xA5);
                    if (it == recv_buffer_.end()) {
                        // no header, drop everything
                        recv_buffer_.clear();
                        break;
                    }
                    size_t idx = std::distance(recv_buffer_.begin(), it);
                    if (recv_buffer_.size() - idx < rBUFFERSIZE) {
                        // incomplete frame
                        break;
                    }
                    unsigned char frame[rBUFFERSIZE];
                    for (size_t i = 0; i < rBUFFERSIZE; ++i) frame[i] = recv_buffer_[idx + i];
                    // erase consumed bytes
                    recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + idx + rBUFFERSIZE);
                    // process extracted frame (publish etc.)
                    process_frame(frame);
                }
            }

            std::this_thread::sleep_for(2ms);
        }
    }

    void process_frame(const unsigned char *new_r_buffer)
    {
        // 控制打印频率：只有每 print_every_ 帧打印一次所有字节，避免日志量过大
        int count = frame_counter_.fetch_add(1, std::memory_order_relaxed) + 1;
        bool printed_group = false;
        // 先计算血量，以便在分组打印中先输出血量行
        uint16_t current_hp = (uint16_t)new_r_buffer[3] << 8 | (uint16_t)new_r_buffer[4];
        // if (count % print_every_ == 0) {
        //     printed_group = true;
        //     RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
        //     // 先打印血量行
        //     RCLCPP_INFO(this->get_logger(), "血量发送为: %d", current_hp);
        //     // 再打印每字节内容
        //     for (int i = 0; i < rBUFFERSIZE; ++i) {
        //         RCLCPP_INFO(this->get_logger(), "r%d发送为: %d", i, new_r_buffer[i]);
        //     }
        // }

        // Mirror the processing logic from main() for a single extracted frame
        uint16_t own_sentry_hp = (uint16_t)new_r_buffer[3] << 8 | (uint16_t)new_r_buffer[4];
        uint8_t current_stance = (uint8_t)new_r_buffer[7];
        rm_decision_interfaces::msg::BothCampHp both_camp_hp_msg;
        both_camp_hp_msg.own_sentry_hp = own_sentry_hp;
        both_camp_hp_msg.current_stance = current_stance;
        both_camp_hp_pub->publish(both_camp_hp_msg);

    // 血量信息已在上面计算（用于分组打印）
        rm_decision_interfaces::msg::RobotStatus robotstatus_msg;
        robotstatus_msg.current_hp = current_hp;
        robotstatus_msg.outpost_hp = (uint16_t)new_r_buffer[1] << 8 | (uint16_t)new_r_buffer[2];
        robotstatus_msg.en_outpost_hp = (uint16_t)new_r_buffer[5] << 8 | (uint16_t)new_r_buffer[6];
        robotstatus_msg.current_stance = (uint8_t)new_r_buffer[7];
        robotstatus_msg.bullets_fired = (uint16_t)new_r_buffer[12] << 8 | (uint16_t)new_r_buffer[13];
        robotstatus_msg.remain_gold = (uint16_t)new_r_buffer[14] << 8 | (uint16_t)new_r_buffer[15];
        robotstatus_msg.fort_fired = (uint16_t)new_r_buffer[16] << 8 | (uint16_t)new_r_buffer[17];
        robotstatus_msg.resur_gold = (uint16_t)new_r_buffer[18] << 8 | (uint16_t)new_r_buffer[19];
        robotstatus_msg.ifoutpost = (uint8_t)new_r_buffer[25];    //是否识别敌方前哨站
        robotstatus_msg.in_defense_re_time = (uint8_t)new_r_buffer[27];     //强化防御剩余时间
        RobotStatus_pub->publish(robotstatus_msg);

        std_msgs::msg::UInt8 fort_if_msg;
        fort_if_msg.data = (uint8_t)new_r_buffer[22];
        fort_if_pub->publish(fort_if_msg);

        // std_msgs::msg::UInt8 outpost_if_msg;
        // outpost_if_msg.data = (uint8_t)new_r_buffer[23];
        // outpost_if_pub->publish(outpost_if_msg);

        // std_msgs::msg::UInt8 midhigh_if_msg;
        // midhigh_if_msg.data = (uint8_t)new_r_buffer[24];
        // midhigh_if_pub->publish(midhigh_if_msg);

        std_msgs::msg::UInt16 remain_time_msg;
        remain_time_msg.data = (uint16_t)new_r_buffer[20] << 8 | (uint16_t)new_r_buffer[21];
        remain_time_pub->publish(remain_time_msg);

        uint8_t game_status = (uint8_t)new_r_buffer[9];
        std_msgs::msg::UInt8 game_status_msg;
        game_status_msg.data = game_status;
        game_status_pub->publish(game_status_msg);

        uint8_t enemy_detected_flag = (uint8_t)new_r_buffer[8];
        std_msgs::msg::UInt8 enemy_detected_msg;
        enemy_detected_msg.data = enemy_detected_flag;
        enemy_detected_pub->publish(enemy_detected_msg);

        uint8_t tree_num = (uint8_t)new_r_buffer[10];
        std_msgs::msg::UInt8 tree_num_msg;
        tree_num_msg.data = tree_num;
        tree_num_pub->publish(tree_num_msg);

        uint8_t mid_if = (uint8_t)new_r_buffer[11];
        std_msgs::msg::UInt8 mid_if_msg;
        mid_if_msg.data = mid_if;
        mid_if_pub->publish(mid_if_msg);

        // 非分组帧不打印血量，避免日志噪声；血量将在分组打印时一起输出

        if (game_status == 5) {
            std::system("shutdown now");
        }
    }
    void write_serial(uint8_t *s_buffer,int s)
    {
        // 将打包好的数据发送（带锁和异常保护）
        std::lock_guard<std::mutex> lk(ser_mutex);
        if (!ser.isOpen()) {
            RCLCPP_WARN(this->get_logger(), "串口未打开，跳过发送");
            return;
        }
        try {
            ser.write(s_buffer, s);
            RCLCPP_DEBUG(this->get_logger(), "发送成功");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "串口写入异常: %s", e.what());
            // 尝试关闭串口以触发上层重连逻辑
            try { ser.close(); } catch (...) {}
        }
    }
    
    void pack()
    {
        const uint16_t sender_id = sender_id_;
        // 定义X Y Z（Ang_V）角速度
        float_union Vx, Vy, Vz;
        Vz.fvalue = 0.0f;
        // 将传入的速度话题信息重新赋值
        Vx.fvalue = cmd_vel_buffer.linear.x;
        Vy.fvalue = cmd_vel_buffer.linear.y;
        {
            std::lock_guard<std::mutex> lock(cruise_mutex_);
            // 巡航节点停止发布后，不能一直发送最后一次非零速度。
            const auto age = std::chrono::steady_clock::now() - cruise_received_at_;
            Vz.fvalue = cruise_received_ && age <= std::chrono::milliseconds(500)
                ? cruise_angular_z_ : 0.0f;
        }
        std::cout << Vx.fvalue << " " << Vy.fvalue << " " << Vz.fvalue << std::endl;

        uint8_t s_buffer[sBUFFERSIZE];
        memset(s_buffer, 0, sizeof(s_buffer));      // 清空发送缓存

        s_buffer[0] = 0xA5;      
        // 数据打包
        s_buffer[1] = Vx.cvalue[0];
        s_buffer[2] = Vx.cvalue[1];
        s_buffer[3] = Vx.cvalue[2];
        s_buffer[4] = Vx.cvalue[3];
        // Vy
        s_buffer[5] = Vy.cvalue[0];
        s_buffer[6] = Vy.cvalue[1];
        s_buffer[7] = Vy.cvalue[2];
        s_buffer[8] = Vy.cvalue[3];
        // Vz
        s_buffer[9] = Vz.cvalue[0];
        s_buffer[10] = Vz.cvalue[1];
        s_buffer[11] = Vz.cvalue[2];
        s_buffer[12] = Vz.cvalue[3];

        // s_buffer[14] = arrival;

        s_buffer[13] = target_stance;

        s_buffer[14] = nav_ready;  // 导航就绪标志

        s_buffer[15] = super_ele;  // 是否开超电

        s_buffer[16] = if_guodong;    // 起伏路段标志
        s_buffer[17] = intensify;  // 是否强化模式

        s_buffer[18] = 0xA5;            // SOF
        s_buffer[19] = 4;            // data_length low: 1 byte status data
        s_buffer[20] = 0;            // data_length high
        s_buffer[21] = 0x00;            // seq
        crc8::Append_CRC8_Check_Sum(&s_buffer[18], 5);

        s_buffer[23] = 0x10;            // cmd_id low: 0x0310
        s_buffer[24] = 0x03;            // cmd_id high
        s_buffer[25] = bt_status_buffer.data;
        s_buffer[26] = bt_remain_time & 0xFF;          // low byte
        s_buffer[27] = (bt_remain_time >> 8) & 0xFF; // high byte
        s_buffer[28] = bt_game_status;
        crc8::Append_CRC16_Check_Sum(&s_buffer[18], 13);

        constexpr size_t kPathPointCount = 50;
        constexpr size_t kPathDeltaCount = kPathPointCount - 1;
        constexpr size_t kPathPayloadLen = 105;
        constexpr size_t kPathInnerHeaderLen = 5;
        constexpr size_t kPathCmdIdLen = 2;
        constexpr size_t kPathInnerCrcLen = 2;
        constexpr size_t kPathInnerFrameOffset = 31;
        constexpr size_t kPathCmdIdOffset = kPathInnerFrameOffset + kPathInnerHeaderLen;
        constexpr size_t kPathPayloadOffset = kPathCmdIdOffset + kPathCmdIdLen;
        constexpr size_t kPathInnerFrameLen = kPathInnerHeaderLen + kPathCmdIdLen + kPathPayloadLen + kPathInnerCrcLen;
        constexpr size_t kMergedPacketLen = sBUFFERSIZE;

        s_buffer[kPathInnerFrameOffset] = 0xA5;            // SOF
        s_buffer[kPathInnerFrameOffset + 1] = kPathPayloadLen & 0xFF;            // data_length low
        s_buffer[kPathInnerFrameOffset + 2] = (kPathPayloadLen >> 8) & 0xFF;     // data_length high
        s_buffer[kPathInnerFrameOffset + 3] = 0x00;            // seq
        crc8::Append_CRC8_Check_Sum(&s_buffer[kPathInnerFrameOffset], kPathInnerHeaderLen);
        s_buffer[kPathCmdIdOffset] = 0x07;                     // cmd_id low: 0x0307
        s_buffer[kPathCmdIdOffset + 1] = 0x03;                 // cmd_id high

        auto write_i16_le = [&s_buffer](size_t index, int16_t value) {
            s_buffer[index] = static_cast<uint8_t>(value & 0xFF);
            s_buffer[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        };

        auto meter_to_dm_i16 = [](double meter) {
            // referee_map 的原点是小地图左下角；坐标不能为负。
            // 先限制浮点范围再取整，避免大坐标导致 lround 溢出。
            const double value = std::clamp(meter * 10.0, 0.0, static_cast<double>(INT16_MAX));
            return static_cast<int16_t>(std::lround(value));
        };

        nav_msgs::msg::Path goal_route_path;
        const bool goal_route_selected = navigation_route_active_ && goal_route_points_.size() >= 4;
        if (goal_route_selected) {
            goal_route_path.header.frame_id = goal_route_points_.front().header.frame_id;
            goal_route_path.header.stamp = goal_route_points_.front().header.stamp;
            goal_route_path.poses.assign(goal_route_points_.begin(), goal_route_points_.end());
        }
        const nav_msgs::msg::Path &selected_path = goal_route_selected ?
            goal_route_path : local_path_buffer;
        const size_t selected_input_points = goal_route_selected ?
            goal_route_points_.size() : path_input_points_;

        std::array<int16_t, kPathPointCount> x_dm{};
        std::array<int16_t, kPathPointCount> y_dm{};
        // 将左下角原点的 23.25 m × 15.55 m 场地铺满到 28 m × 15 m 小地图。
        // 输入的 /goal_pose 和 /plan 均为 map 坐标，通过 TF 转到左下角原点。
        constexpr double kPathScaleX = 28.0 / (458*0.05);
        constexpr double kPathScaleY = 15.0 / (227*0.05);
        size_t valid_points = std::min(selected_path.poses.size(), kPathPointCount);
        bool path_valid = valid_points > 0 && !selected_path.header.frame_id.empty();

        std::vector<std::pair<int16_t, int16_t>> transformed_points;
        transformed_points.reserve(kPathPointCount);

        for (size_t i = 0; i < valid_points; ++i) {
            geometry_msgs::msg::PoseStamped source_pose = selected_path.poses[i];
            if (source_pose.header.frame_id.empty()) {
                source_pose.header.frame_id = selected_path.header.frame_id;
                source_pose.header.stamp = selected_path.header.stamp;
            }
            try {
                // 巡航点和普通导航路径都以 map 为输入；TF 负责转换到以
                // 裁判小地图左下角为原点的 referee_map。
                // 巡航点用最新 TF，避免其启动时间戳超出 TF 缓存。
                if (goal_route_selected) {
                    source_pose.header.stamp.sec = 0;
                    source_pose.header.stamp.nanosec = 0;
                }
                const auto referee_pose = tf_buffer_->transform(
                    source_pose, referee_frame_, tf2::durationFromSec(0.05));
                if (!std::isfinite(referee_pose.pose.position.x) ||
                    !std::isfinite(referee_pose.pose.position.y)) {
                    path_valid = false;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "0x0307: 全局路径变换后的坐标无效，本帧不发送有效路径");
                    break;
                }
                const int16_t x = meter_to_dm_i16(referee_pose.pose.position.x * kPathScaleX);
                const int16_t y = meter_to_dm_i16(referee_pose.pose.position.y * kPathScaleY);
                if (goal_route_selected && i > 0) {
                    const auto [prev_x, prev_y] = transformed_points.back();
                    const int dx = static_cast<int>(x) - prev_x;
                    const int dy = static_cast<int>(y) - prev_y;
                    // Preserve the same straight segment while inserting points
                    // where the signed 8-bit referee deltas would overflow.
                    const int steps = std::max(1, static_cast<int>(std::ceil(
                        std::max(std::abs(dx), std::abs(dy)) / 127.0)));
                    for (int step = 1; step <= steps; ++step) {
                        if (transformed_points.size() >= kPathPointCount) {
                            path_valid = false;
                            break;
                        }
                        const double t = static_cast<double>(step) / steps;
                        transformed_points.emplace_back(
                            static_cast<int16_t>(std::lround(prev_x + t * dx)),
                            static_cast<int16_t>(std::lround(prev_y + t * dy)));
                    }
                    if (!path_valid) break;
                } else {
                    transformed_points.emplace_back(x, y);
                }
            } catch (const tf2::TransformException &ex) {
                path_valid = false;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "无法把采样后的全局路径从 '%s' 转到 '%s': %s",
                    source_pose.header.frame_id.c_str(), referee_frame_.c_str(), ex.what());
                break;
            }
        }

        if (path_valid) {
            valid_points = transformed_points.size();
            if (valid_points > kPathPointCount) {
                path_valid = false;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "0x0307: 插入增量安全采样点后超过50点上限，本帧路径无效");
            }
            for (size_t i = 0; i < valid_points; ++i) {
                x_dm[i] = transformed_points[i].first;
                y_dm[i] = transformed_points[i].second;
            }
        }

        // 禁止静默截断增量：截断会使客户端累计还原后的终点偏离目标。
        if (path_valid) {
            for (size_t i = 1; i < valid_points; ++i) {
                const int dx = x_dm[i] - x_dm[i - 1];
                const int dy = y_dm[i] - y_dm[i - 1];
                if (!decision::path_delta_fits(dx, dy)) {
                    path_valid = false;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "0x0307: 路径增量超出int8范围: source=%s segment=%zu dx=%d dy=%d dm；本帧路径无效",
                        goal_route_selected ? "/goal_pose route" : "/plan",
                        i - 1, dx, dy);
                    break;
                }
            }
        }
        if (!path_valid) {
            valid_points = 0;
            x_dm.fill(0);
            y_dm.fill(0);
        }

        // data[0]: 行为类型；data[1..2]: 起点 x；data[3..4]: 起点 y，单位 dm，小端 int16。
        // s_buffer[kPathPayloadOffset] = bt_status_buffer.data;
        s_buffer[kPathPayloadOffset] = path_valid ? 3 : 0;  // 3: 移动到目标点；无有效 TF 时不发布路径
        write_i16_le(kPathPayloadOffset + 1, x_dm[0]);
        write_i16_le(kPathPayloadOffset + 3, y_dm[0]);

        // data[5..53]: 49 个 x 增量；data[54..102]: 49 个 y 增量，单位 dm, int8。
        for (size_t i = 0; i < kPathDeltaCount; ++i) {
            int16_t dx = 0;
            int16_t dy = 0;
            if (i + 1 < valid_points) {
                dx = static_cast<int16_t>(x_dm[i + 1] - x_dm[i]);
                dy = static_cast<int16_t>(y_dm[i + 1] - y_dm[i]);
            }
            s_buffer[kPathPayloadOffset + 5 + i] = static_cast<uint8_t>(static_cast<int8_t>(dx));
            s_buffer[kPathPayloadOffset + 54 + i] = static_cast<uint8_t>(static_cast<int8_t>(dy));
        }


        // data[103..104]: 发送者 ID，小端 uint16。
        s_buffer[kPathPayloadOffset + 103] = static_cast<uint8_t>(sender_id & 0xFF);
        s_buffer[kPathPayloadOffset + 104] = static_cast<uint8_t>((sender_id >> 8) & 0xFF);

        // 内层校验覆盖 A5 帧头、0x0307 命令码、105 字节路径数据，并写到内层末尾。
        crc8::Append_CRC16_Check_Sum(&s_buffer[kPathInnerFrameOffset], kPathInnerFrameLen);

        // 可选诊断：同时输出易读字段和真正发给下位机的完整 0x0307 字节序列。
        // 这样可直接和 Keil 中 judge_tx_info[0..113] 对照。
        if (debug_referee_0307_) {
            std::ostringstream frame_hex;
            std::ostringstream dx_text;
            std::ostringstream dy_text;
            frame_hex << std::hex << std::uppercase << std::setfill('0');
            for (size_t i = 0; i < kPathInnerFrameLen; ++i) {
                if (i != 0) frame_hex << ' ';
                frame_hex << std::setw(2)
                          << static_cast<unsigned int>(s_buffer[kPathInnerFrameOffset + i]);
            }
            for (size_t i = 0; i < kPathDeltaCount; ++i) {
                if (i != 0) {
                    dx_text << ',';
                    dy_text << ',';
                }
                dx_text << static_cast<int>(static_cast<int8_t>(s_buffer[kPathPayloadOffset + 5 + i]));
                dy_text << static_cast<int>(static_cast<int8_t>(s_buffer[kPathPayloadOffset + 54 + i]));
            }
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), debug_referee_0307_period_ms_,
                "0x0307: valid=%d source=%s points=%zu raw_points=%zu intention=%u start=(%d,%d)dm sender_id=%u | "
                "dx=[%s] dy=[%s] | frame[114]=%s",
                path_valid, goal_route_selected ? "/goal_pose route" : "/plan",
                valid_points, selected_input_points,
                static_cast<unsigned int>(s_buffer[kPathPayloadOffset]),
                static_cast<int>(x_dm[0]), static_cast<int>(y_dm[0]), sender_id,
                dx_text.str().c_str(), dy_text.str().c_str(), frame_hex.str().c_str());
        }

        // 外层校验给底盘使用。
        crc8::Append_CRC16_Check_Sum(s_buffer, kMergedPacketLen);
        write_serial(s_buffer,sBUFFERSIZE);
        
    }

    // void bt_pack(const std_msgs::msg::UInt8 &bt_status_buffer)
    // {
    //     uint8_t s_buffer[18];
    //     memset(s_buffer, 0, sizeof(s_buffer));      // 清空发送缓存

    //     s_buffer[0] = 0xAA;     
    //     s_buffer[1] = 0x02;
    //     s_buffer[2] = 13;
        
    //     // 内层包: frame_header(CRC8) + cmd_id + data + frame_tail(CRC16)。
    //     s_buffer[3] = 0xA5;            // SOF
    //     s_buffer[4] = 4;            // data_length low: 1 byte status data
    //     s_buffer[5] = 0;            // data_length high
    //     s_buffer[6] = 0x00;            // seq
    //     crc8::Append_CRC8_Check_Sum(&s_buffer[3], 5);

    //     s_buffer[8] = 0x10;            // cmd_id low: 0x0310
    //     s_buffer[9] = 0x03;            // cmd_id high
    //     s_buffer[10] = bt_status_buffer.data;
    //     s_buffer[11] = bt_remain_time & 0xFF;          // low byte
    //     s_buffer[12] = (bt_remain_time >> 8) & 0xFF; // high byte
    //     s_buffer[13] = bt_game_status;
    //     crc8::Append_CRC16_Check_Sum(&s_buffer[3], 13);

    //     // 外层校验给底盘使用。
    //     crc8::Append_CRC16_Check_Sum(s_buffer, 18);
    //     write_serial(s_buffer,18);
    // }

    // void path_pack(const nav_msgs::msg::Path &path_msg, const std_msgs::msg::UInt8 &bt_status_buffer, uint16_t sender_id = 7)
    // {
    //     constexpr size_t kPathPointCount = 50;
    //     constexpr size_t kPathDeltaCount = kPathPointCount - 1;
    //     constexpr size_t kPathPayloadLen = 104;
    //     constexpr size_t kPathInnerHeaderLen = 5;
    //     constexpr size_t kPathInnerCrcLen = 2;
    //     constexpr size_t kPathDataLen = kPathInnerHeaderLen + kPathPayloadLen + kPathInnerCrcLen;
    //     constexpr size_t kOuterHeaderLen = 3;
    //     constexpr size_t kOuterCrcLen = 2;
    //     constexpr size_t kPathPacketLen = kOuterHeaderLen + kPathDataLen + kOuterCrcLen;
    //     constexpr size_t kPathPayloadOffset = kOuterHeaderLen + kPathInnerHeaderLen;

    //     uint8_t s_buffer[kPathPacketLen];
    //     memset(s_buffer, 0, sizeof(s_buffer));      // 清空发送缓存

    //     s_buffer[0] = 0xAA;     
    //     s_buffer[1] = 0x03;
    //     s_buffer[2] = kPathDataLen;

    //     s_buffer[3] = 0xA5;            // SOF
    //     s_buffer[4] = kPathPayloadLen & 0xFF;            // data_length low
    //     s_buffer[5] = (kPathPayloadLen >> 8) & 0xFF;     // data_length high
    //     s_buffer[6] = 0x00;            // seq
    //     crc8::Append_CRC8_Check_Sum(&s_buffer[3], 5);

    //     (void)bt_status_buffer;

    //     auto write_i16_le = [&s_buffer](size_t index, int16_t value) {
    //         s_buffer[index] = static_cast<uint8_t>(value & 0xFF);
    //         s_buffer[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    //     };

    //     auto meter_to_dm_i16 = [](double meter) {
    //         long value = std::lround(meter * 10.0);
    //         value = std::clamp(value, static_cast<long>(INT16_MIN), static_cast<long>(INT16_MAX));
    //         return static_cast<int16_t>(value);
    //     };

    //     auto clamp_i8 = [](int value) {
    //         value = std::clamp(value, static_cast<int>(INT8_MIN), static_cast<int>(INT8_MAX));
    //         return static_cast<int8_t>(value);
    //     };

    //     std::array<int16_t, kPathPointCount> x_dm{};
    //     std::array<int16_t, kPathPointCount> y_dm{};
    //     const size_t valid_points = std::min(path_msg.poses.size(), kPathPointCount);

    //     for (size_t i = 0; i < valid_points; ++i) {
    //         x_dm[i] = meter_to_dm_i16(path_msg.poses[i].pose.position.x);
    //         y_dm[i] = meter_to_dm_i16(path_msg.poses[i].pose.position.y);
    //     }

    //     // data[0..1]: 起点 x, data[2..3]: 起点 y, 单位 dm, 小端 int16。
    //     write_i16_le(kPathPayloadOffset, x_dm[0]);
    //     write_i16_le(kPathPayloadOffset + 2, y_dm[0]);

    //     // data[4..52]: 49 个 x 增量；data[53..101]: 49 个 y 增量，单位 dm, int8。
    //     for (size_t i = 0; i < kPathDeltaCount; ++i) {
    //         int16_t dx = 0;
    //         int16_t dy = 0;
    //         if (i + 1 < valid_points) {
    //             dx = static_cast<int16_t>(x_dm[i + 1] - x_dm[i]);
    //             dy = static_cast<int16_t>(y_dm[i + 1] - y_dm[i]);
    //         }
    //         s_buffer[kPathPayloadOffset + 4 + i] = static_cast<uint8_t>(clamp_i8(dx));
    //         s_buffer[kPathPayloadOffset + 53 + i] = static_cast<uint8_t>(clamp_i8(dy));
    //     }

    //     // data[102..103]: 发送者 ID，小端 uint16。
    //     s_buffer[kPathPayloadOffset + 102] = static_cast<uint8_t>(sender_id & 0xFF);
    //     s_buffer[kPathPayloadOffset + 103] = static_cast<uint8_t>((sender_id >> 8) & 0xFF);

    //     // 内层校验覆盖 A5 帧头、104 字节路径数据，并写到内层末尾。
    //     crc8::Append_CRC16_Check_Sum(&s_buffer[3], kPathInnerHeaderLen + kPathPayloadLen + kPathInnerCrcLen);

    //     // 外层校验给底盘使用。
    //     crc8::Append_CRC16_Check_Sum(s_buffer, kPathPacketLen);
    //     write_serial(s_buffer, kPathPacketLen);
    // }

    // target_stance话题的回调函数
    void target_stance_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        target_stance = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received target_stance data: %d", target_stance);
        new_target_stance_received = true;
    }

    // super_ele话题的回调函数
    void super_ele_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        super_ele = msg->data;
    }

    void if_qifu_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        if_qifu = msg->data;
    }

    void intensify_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        intensify = msg->data;
    }

    // nav_ready话题的回调函数，监控导航就绪状态
    void nav_ready_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        nav_ready = 0;
        nav_ready = msg->data;
        nav_ready_received = true;
    }

    void if_guodong_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        if (msg->data > 1) {
            RCLCPP_WARN(this->get_logger(), "Ignoring invalid if_guodong: %u", msg->data);
            return;
        }
        if_guodong = msg->data;
    }

    void bt_status_callback(const rm_decision_interfaces::msg::RobotStatus::SharedPtr msg)
    {
        bt_status_buffer.data = msg->current_stance;
        new_bt_status_received = true;
    }

    void bt_remain_time_callback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        bt_remain_time = msg->data;
        new_bt_status_received = true;
    }

    void bt_game_status_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        bt_game_status = msg->data;
        new_bt_status_received = true;
    }

    // 订阅cmd_vel话题的回调函数,用于显示速度以及角速度
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
    {
        // 存储最新的速度指令，并标记新指令已经接收
        cmd_vel_buffer = *cmd_vel;
        new_cmd_vel_received = true;
    }

    void cruise_speed_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cruise_mutex_);
        const float value = static_cast<float>(msg->angular.z);
        cruise_angular_z_ = std::isfinite(value) ? value : 0.0f;
        cruise_received_at_ = std::chrono::steady_clock::now();
        cruise_received_ = true;
    }

    void path_vision_callback(const nav_msgs::msg::Path::SharedPtr local_path)
    {
        // /plan 是完整全局路径。仅收到新消息时采样，不在串口定时器里遍历原始路径。
        path_input_points_ = local_path->poses.size();
        local_path_buffer.header = local_path->header;
        local_path_buffer.poses.clear();
        std::vector<decision::PathPoint> input;
        input.reserve(path_input_points_);
        try {
            if (!local_path->poses.empty() && local_path->header.frame_id.empty()) {
                throw std::invalid_argument("global path frame_id is empty");
            }
            for (const auto & pose : local_path->poses) {
                if (!pose.header.frame_id.empty() &&
                    pose.header.frame_id != local_path->header.frame_id) {
                    throw std::invalid_argument("global path contains mixed coordinate frames");
                }
                const auto & p = pose.pose.position;
                input.push_back({p.x, p.y, p.z});
            }
            const auto sampled = decision::sample_path(input, 50);
            local_path_buffer.poses.reserve(sampled.size());
            for (const auto & p : sampled) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = local_path->header;
                pose.pose.position.x = p.x;
                pose.pose.position.y = p.y;
                pose.pose.position.z = p.z;
                pose.pose.orientation.w = 1.0;  // 0x0307 只使用位置，不发送朝向。
                local_path_buffer.poses.push_back(pose);
            }
        } catch (const std::invalid_argument & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "0x0307: 全局路径采样失败: %s", ex.what());
        }
        new_local_path_received = true;
    }

    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
    {
        if (goal->header.frame_id.empty() || !std::isfinite(goal->pose.position.x) ||
            !std::isfinite(goal->pose.position.y)) {
            RCLCPP_WARN(this->get_logger(), "Ignoring invalid /goal_pose route point");
            return;
        }
        if (!goal_route_points_.empty() &&
            goal_route_points_.back().header.frame_id != goal->header.frame_id) {
            goal_route_points_.clear();
        }
        if (!goal_route_points_.empty()) {
            const auto & last = goal_route_points_.back().pose.position;
            if (std::abs(last.x - goal->pose.position.x) < 1e-4 &&
                std::abs(last.y - goal->pose.position.y) < 1e-4) {
                return;  // Ignore consecutive duplicate publications.
            }
        }
        goal_route_points_.push_back(*goal);
        while (goal_route_points_.size() > 4) goal_route_points_.pop_front();
        RCLCPP_INFO(this->get_logger(), "/goal_pose route point cached: (%0.3f,%0.3f) frame=%s (%zu/4)",
            goal->pose.position.x, goal->pose.position.y, goal->header.frame_id.c_str(),
            goal_route_points_.size());
    }

    // 定时器回调函数
    void timer_callback()
    {
        // 如果有新的速度指令到来，就发送该速度指令
        // if (new_cmd_vel_received || new_target_stance_received || nav_ready_received)
        // {
        //     data_pack(cmd_vel_buffer);
        //     new_cmd_vel_received = false;
        //     new_target_stance_received = false;
        //     nav_ready_received = true;
        //     // rosout_received = false;
        // }
        // if (new_cmd_vel_received)
        // {
         pack();
            // new_cmd_vel_received = false;
        // if (new_bt_status_received)
        // {
        //     bt_pack(bt_status_buffer);
        //     new_bt_status_received = false;
        // }

        // if (new_local_path_received)
        // {
        //     path_pack(local_path_buffer, bt_status_buffer);
        //     new_local_path_received = false;
        // }
    }

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr if_guodong_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_stance_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr super_ele_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cruise_speed_sub;
    std::mutex cruise_mutex_;
    float cruise_angular_z_{0.0f};
    bool cruise_received_{false};
    std::chrono::steady_clock::time_point cruise_received_at_{};
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr if_qifu_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr intensify_sub;

    rclcpp::Publisher<rm_decision_interfaces::msg::RobotStatus>::SharedPtr RobotStatus_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr enemy_detected_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_status_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr tree_num_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr outpost_if_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fort_if_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr midhigh_if_pub;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr remain_time_pub;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mid_if_pub;           //RMUL
    rclcpp::Publisher<rm_decision_interfaces::msg::BothCampHp>::SharedPtr both_camp_hp_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist cmd_vel_buffer;
    std_msgs::msg::UInt8 bt_status_buffer;

    bool new_bt_status_received = false;
    bool new_cmd_vel_received = false;
    bool new_target_stance_received = false;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_ready_sub;       // 订阅 /rosout 监控终端输出
    rclcpp::Subscription<rm_decision_interfaces::msg::RobotStatus>::SharedPtr bt_status_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr bt_game_status_sub;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr bt_remain_time_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_vision_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_route_active_sub;
    nav_msgs::msg::Path local_path_buffer;
    std::deque<geometry_msgs::msg::PoseStamped> goal_route_points_;
    bool navigation_route_active_ = false;
    size_t path_input_points_ = 0;
    bool new_local_path_received = false;
    std::string referee_frame_;
    uint16_t sender_id_ = 7;
    std::string serial_port_;
    bool debug_referee_0307_ = false;
    int debug_referee_0307_period_ms_ = 1000;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool nav_ready_received = true;
    // recv buffer and thread for reading
    std::vector<unsigned char> recv_buffer_;
    std::mutex recv_mutex_;
    std::thread read_thread_;
    std::atomic<bool> running_{false};
    // 控制打印频率，避免过多日志
    std::atomic<int> frame_counter_{0};
    int print_every_ = 5; // 每5帧打印一次（可调整）

};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialChassis>();
    // Let ROS spin; background thread handles serial reads and publishes
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
