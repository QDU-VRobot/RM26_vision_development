#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

// ROS2
#include <tf2/LinearMath/Matrix3x3.h>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// LibXR
#include "SharedTopic.hpp"
#include "SharedTopicClient.hpp"
#include "app_framework.hpp"
#include "linux_uart.hpp"
#include "message.hpp"
#include "thread.hpp"

// ROS2自定义消息包
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"

namespace rm_serial_driver
{

/*消息包*/
// 云台欧拉角数据结构体
typedef struct
{
  float pitch;
  float yaw;
  float roll;
} gimbal_euler;

/*LibXR相关*/

// LibXR应用程序入口函数
static void XRobotMain(LibXR::HardwareContainer& hw)
{
  using namespace LibXR;
  static ApplicationManager appmgr;

  // LibXR共享话题创建
  // 从下位机接收: ahrs_quaternion (云台姿态), bullet_speed (弹速)
  static SharedTopic shared_topic(hw, appmgr, "uart_client", 81920, 256,
                                  {{"ahrs_quaternion"}, {"bullet_speed", "referee"}});

  // 向下位机发送: target_eulr (目标欧拉角), fire_notify (开火通知)
  static SharedTopicClient shared_topic_client(
      hw, appmgr, "uart_client", 81920, 256,
      {{"target_eulr", "tracker"}, {"fire_notify", "tracker"}});
}

/* RMSerialDriver类定义*/
class RMSerialDriver : public rclcpp::Node
{
 public:
  explicit RMSerialDriver(const rclcpp::NodeOptions& options);
  ~RMSerialDriver() override;

 private:
  /* 函数声明 */

  // 四元数转欧拉角函数
  void ConvertQuaternionToEuler(float qx, float qy, float qz, float qw, float& roll,
                                float& pitch, float& yaw);

  // Send消息回调函数
  void SendCallBack(const auto_aim_interfaces::msg::Send::SharedPtr msg);

  /* ROS2发布者 */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_pub_;  // 云台关节状态发布者
  rclcpp::Publisher<auto_aim_interfaces::msg::Velocity>::SharedPtr
      velocity_pub_;  // 弹速发布者

  /* ROS2订阅者 */
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr
      send_sub_;  // Send消息订阅者

  /* LibXR Topic (用于发送到下位机) */
  LibXR::Topic target_eulr_topic_;
  LibXR::Topic fire_notify_topic_;

  /* LibXR初始化相关成员变量 */
  std::unique_ptr<LibXR::RamFS> ramfs_;
  std::unique_ptr<LibXR::LinuxUART> uart_client_;
  std::unique_ptr<LibXR::Terminal<1024, 64, 16, 128>> terminal_;
  std::unique_ptr<LibXR::Thread> term_thread_;
  std::unique_ptr<LibXR::HardwareContainer> peripherals_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
