#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

// ROS2
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
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
// #include "auto_aim_interfaces/msg/chassis.hpp"
// #include "auto_aim_interfaces/msg/target.hpp"
// #include "auto_aim_interfaces/msg/send.hpp"
// #include "auto_aim_interfaces/msg/velocity.hpp"
// #include "referee_interfaces/msg/rfid.hpp"
// #include "referee_interfaces/msg/buff.hpp"
// #include "referee_interfaces/msg/basic_hp.hpp"
// #include "referee_interfaces/msg/ally_bot.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "referee_interfaces/msg/game_status.hpp"

namespace rm_serial_driver
{

//底盘运动数据结构体
typedef struct
{
  float vx=0.0;
  float vy=0.0;
  float w=0.0;
} move_vec;

typedef struct{
  float pitch;
  float yaw;
  float roll;
} gimbal_euler;

// LibXR应用程序入口函数
static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  static ApplicationManager appmgr;

  // Auto-generated module instantiations
  static SharedTopic SharedTopic(hw, appmgr, "uart_client", 81920, 256, {{"ahrs_quaternion"}});
  static SharedTopicClient SharedTopicClient(hw, appmgr, "uart_client", 81920, 256, {{"chassis_data"}});
}

// RMSerialDriver类定义
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);
  ~RMSerialDriver() override;

private:

  // /* 函数声明 */
  void convert_quaternion_to_euler(
    float qx, float qy, float qz, float qw,
    float &roll, float &pitch, float &yaw); // 四元数转欧拉角函数

  /* C++11线程接收数据 */
  std::thread receive_thread_;

  /* ROS2发布者 */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_; //云台关节状态发布者

  /* LibXR相关成员变量 */
  std::unique_ptr<LibXR::RamFS> ramfs;
  std::unique_ptr<LibXR::LinuxUART> uart_client;
  std::unique_ptr<LibXR::Terminal<1024, 64, 16, 128>> terminal;
  std::unique_ptr<LibXR::Thread> term_thread;
  std::unique_ptr<LibXR::HardwareContainer> peripherals;

  // rclcpp::TimerBase::SharedPtr timer_get_euler;
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};
} 

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
