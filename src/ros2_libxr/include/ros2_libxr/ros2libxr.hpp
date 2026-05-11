#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

// ROS2
#include <cstdint>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// C++ system
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

// LibXR
#include "app_framework.hpp"
#include "linux_uart.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "uart.hpp"
#include "SharedTopic.hpp"
#include "SharedTopicClient.hpp"

// ROS2自定义消息包
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"
#include "referee_interfaces/msg/robot_status.hpp"
#include "referee_interfaces/msg/game_status.hpp"
#include "referee_interfaces/msg/rfid_status.hpp"

namespace rm_serial_driver {

/*消息包*/

//底盘运动数据结构体
struct move_vec
{
  float vx=0.0;
  float vy=0.0;
  float wz=0.0;
};

//云台欧拉角数据结构体
typedef struct{
  float pitch;
  float yaw;
  float roll;
} gimbal_euler;

//哨兵裁判数据结构体
typedef struct{
  uint8_t robot_id;                  /* 本机器人 ID */
  uint8_t robot_level;               /* 机器人等级 */
  uint16_t remain_hp;                /* 机器人当前血量 */
  uint16_t max_hp;                   /* 机器人血量上限 */
  uint16_t shooter_cooling_value;    /* 机器人射击热量每秒冷却值 */
  uint16_t shooter_heat_limit;       /* 机器人射击热量上限 */
  uint16_t chassis_power_limit;      /* 机器人底盘功率上限 */
  uint8_t power_gimbal_output : 1;   /* gimbal输出，0为无输出，1为24V输出 */
  uint8_t power_chassis_output : 1;  /* chassis输出，0为无输出，1为24V输出*/
  uint8_t power_launcher_output : 1; /* shooter输出，0为无输出，1为24V 输出 */
} SentryData;

//裁判系统机器人状态数据结构体
struct [[gnu::packed]] RobotStatus {
  uint8_t robot_id;                     /* 本机器人 ID */
  uint8_t robot_level;                  /* 机器人等级 */
  uint16_t current_hp;                  /* 机器人当前血量 */
  uint16_t maximum_hp;                  /* 机器人血量上限 */
  uint16_t shooter_barrel_cooling_value;/* 机器人枪口热量每秒冷却值 */
  uint16_t shooter_barrel_heat_limit;   /* 机器人枪口热量上限 */
  // uint16_t shooter_17mm_1_barrel_heat; /* 机器人17mm1枪口热量 */
  uint16_t chassis_power_limit;         /* 机器人底盘功率上限 */
  uint8_t power_gimbal_output : 1;      /* gimbal输出 */
  uint8_t power_chassis_output : 1;     /* chassis输出 */
  uint8_t power_launcher_output : 1;    /* shooter输出 */
};

//裁判系统比赛状态数据结构体
struct [[gnu::packed]] GameStatus {
  uint8_t game_type : 4;                    /* 比赛类型 */
  uint8_t game_progress : 4;                /* 比赛进程 */
  uint16_t stage_remain_time;           /* 当前阶段剩余时间，单位：秒 */
  uint64_t sync_time_stamp;             /* 同步时间戳 */
};

//裁判系统RFID状态数据结构体
struct [[gnu::packed]] RfidStatus {
  uint32_t rfid_status;  /* 各增益点检测状态位图，参照裁判系统协议 0x0209 */
};

//哨兵裁判系统数据包
struct [[gnu::packed]] SentryPack {
  RobotStatus rs;
  GameStatus gs;
  RfidStatus rfid;
};

/*LibXR相关*/

// LibXR应用程序入口函数
static void XRobotMain(LibXR::HardwareContainer &hw) {  
  using namespace LibXR;
  static ApplicationManager appmgr;

  //LibXR共享话题创建,如有话题增加，需要在此处添加
  static SharedTopic SharedTopic(hw, appmgr, "uart_client", 81920, 256, {{"ahrs_quaternion"},{"yawmotor_angle"},{"sentry_ref"}});
  static SharedTopicClient SharedTopicClient(hw, appmgr, "uart_client", 81920, 256, {{"chassis_data"},{"target_euler"},{"fire_notify", "tracker"}});
}

/* RMSerialDriver类定义*/
class RMSerialDriver : public rclcpp::Node {
 public:
  explicit RMSerialDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~RMSerialDriver();

 private:
  uint8_t fire_notify_ = 1;
  double timestamp_offset_{};

  /* 函数声明 */

  // Send消息回调函数
  void SendCallBack(const auto_aim_interfaces::msg::Send::SharedPtr msg);

 
  void convert_quaternion_to_euler(float qx, float qy, float qz, float qw,
                                   float &roll, float &pitch, float &yaw);
  void get_classic(const geometry_msgs::msg::Twist::SharedPtr twi);

 private:
  // LibXR 资源
  std::unique_ptr<LibXR::HardwareContainer> peripherals;
  std::unique_ptr<LibXR::RamFS> ramfs;
  std::unique_ptr<LibXR::LinuxUART> uart_client;
  std::unique_ptr<LibXR::Terminal<1024, 64, 16, 128>> terminal;
  std::unique_ptr<LibXR::Thread> term_thread;

  // LibXR 话题
  LibXR::Topic ahrs_euler_topic_;
  LibXR::Topic move_vec_topic_;
  LibXR::Topic yawmotor_angle_topic_;
  LibXR::Topic sentry_ref_topic_;
  LibXR::Topic ahrs_quaternion_topic_;
  LibXR::Topic bullet_speed_topic_;
  LibXR::Topic target_euler_topic_;
  LibXR::Topic fire_notify_topic_;

  // 底盘运动数据
  move_vec move_;

  //云台相对底盘yaw全局变量
  float yawmotor_angle_data;

  // ROS2 发布者/订阅者
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_vision_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr move_vec_sub;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr send_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr fire_sub_;
  rclcpp::Publisher<referee_interfaces::msg::RobotStatus>::SharedPtr sentry_ref_pub_;
  rclcpp::Publisher<referee_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<referee_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Velocity>::SharedPtr velocity_pub_;
};

} // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
