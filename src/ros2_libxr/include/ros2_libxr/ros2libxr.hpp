#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

// ROS2
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
#include "referee_interfaces/msg/robot_status.hpp"
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
// #include "auto_aim_interfaces/msg/chassis.hpp"
// #include "auto_aim_interfaces/msg/target.hpp"
// #include "auto_aim_interfaces/msg/send.hpp"
// #include "auto_aim_interfaces/msg/velocity.hpp"
// #include "referee_interfaces/msg/rfid.hpp"
// #include "referee_interfaces/msg/buff.hpp"
// #include "referee_interfaces/msg/basic_hp.hpp"
// #include "referee_interfaces/msg/ally_bot.hpp"
// #include "geometry_msgs/msg/twist.hpp"


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

/*LibXR相关*/

// LibXR应用程序入口函数
static void XRobotMain(LibXR::HardwareContainer &hw) {  
  using namespace LibXR;
  static ApplicationManager appmgr;

  //LibXR共享话题创建,如有话题增加，需要在此处添加
  static SharedTopic SharedTopic(hw, appmgr, "uart_client", 81920, 256, {{"ahrs_quaternion"},{"yawmotor_angle"},{"sentry_hp"}});
  static SharedTopicClient SharedTopicClient(hw, appmgr, "uart_client", 81920, 256, {{"chassis_data"}});
}

/* RMSerialDriver类定义*/
class RMSerialDriver : public rclcpp::Node {
 public:
  explicit RMSerialDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~RMSerialDriver();
 
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

  // LibXR 话题（改为成员变量，延长生命周期）
  LibXR::Topic ahrs_euler_topic_;
  LibXR::Topic move_vec_topic_;
  LibXR::Topic yawmotor_angle_topic_;
  LibXR::Topic sentry_hp_topic_;

  // 底盘运动数据
  move_vec move_;

  //云台相对底盘yaw全局变量
  float yawmotor_angle_data;

  // ROS2 发布者/订阅者
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr move_vec_sub;
  rclcpp::Publisher<referee_interfaces::msg::RobotStatus>::SharedPtr sentry_hp_pub_;
};

} // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
