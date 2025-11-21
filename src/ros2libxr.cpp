#include "ros2_libxr/ros2libxr.hpp"
// ROS
#include <cstdio>
#include <iterator>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

// TF2
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// C++ system
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"

// LibXR
#include "crc.hpp"
#include "libxr_rw.hpp"
#include "libxr_type.hpp"
#include "linux_uart.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "transform.hpp"

using namespace std::chrono_literals; //定时器


namespace rm_serial_driver {
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions &options)
    : Node("rm_serial_driver", options) {

  /*Libxr串口初始化*/
  LibXR::PlatformInit(); // 初始化 LibXR
  peripherals = std::make_unique<LibXR::HardwareContainer>();
  ramfs = std::make_unique<LibXR::RamFS>();
  uart_client = std::make_unique<LibXR::LinuxUART>("0483", "5740", 115200, LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);
  terminal = std::make_unique<LibXR::Terminal<1024, 64, 16, 128>>(*ramfs);
  term_thread = std::make_unique<LibXR::Thread>();
  term_thread->Create(terminal.get(),
                      LibXR::Terminal<1024, 64, 16, 128>::ThreadFun, "terminal",
                      81900, LibXR::Thread::Priority::MEDIUM);
  static LibXR::HardwareContainer peripherals{
      LibXR::Entry<LibXR::RamFS>({*ramfs, {"ramfs"}}),
      LibXR::Entry<LibXR::UART>({*uart_client, {"uart_client"}}),
  };



  /*LibXR话题创建*/
  auto ahrs_euler_topic = LibXR::Topic::CreateTopic<LibXR::Quaternion<float>>("ahrs_quaternion"); //LibXR读取云台欧拉角话题
  auto chassis_data_topic = LibXR::Topic::CreateTopic<rm_serial_driver::move_vec>("chassis_data");       //LibXR底盘数据话题

  /* ROS2发布者初始化 */
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "serial/gimbal_joint_state", rclcpp::QoS(rclcpp::KeepLast(1))); // 云台关节状态发布者

  XRobotMain(peripherals); // 调用LibXR应用程序入口函数

  /* 云台位姿回调函数 */
  void (*ahrs_euler_cb_fun)(bool, RMSerialDriver *self, LibXR::RawData &data) =
      [](bool, RMSerialDriver *self, LibXR::RawData &data) {
        auto quat = reinterpret_cast<LibXR::Quaternion<float> *>(data.addr_);
        // XR_LOG_INFO("Serial got quat:%f,%f,%f", quat->Yaw(),quat->Pitch(), quat->Roll()); //调试打印
        // XR_LOG_INFO("Serial got quat:%f,%f,%f,%f", quat->w(),quat->x(), quat->y(), quat->z()); //调试打印

        rm_serial_driver::gimbal_euler gimbal_;
        self->convert_quaternion_to_euler(
          quat->x(), quat->y(), quat->z(), quat->w(),
          gimbal_.roll, gimbal_.pitch, gimbal_.yaw);
        // ROS2发布云台关节状态
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = self->now();
        joint_state.name.push_back("gimbal_pitch_joint");
        joint_state.name.push_back("gimbal_yaw_joint");
        joint_state.position.push_back(gimbal_.pitch);
        joint_state.position.push_back(gimbal_.yaw);
        self->joint_state_pub_->publish(joint_state);
      };
  auto ahrs_euler_cb = LibXR::Topic::Callback::Create(ahrs_euler_cb_fun, this);
  ahrs_euler_topic.RegisterCallback(ahrs_euler_cb);

    while (1)
  {
    LibXR::Thread::Sleep(1000);
  }
  
}

RMSerialDriver::~RMSerialDriver() {
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}


void RMSerialDriver::convert_quaternion_to_euler(
    float qx, float qy, float qz, float qw,
    float &roll, float &pitch, float &yaw)
{
    // TF2 的 Quaternion 构造函数接受 double，因此需要进行类型转换
    // 1. 将 float 输入转换为 double，加载到 TF2 的四元数对象中
    tf2::Quaternion q((double)qx, (double)qy, (double)qz, (double)qw);

    // 2. 转换为 3x3 旋转矩阵
    tf2::Matrix3x3 m(q);

    // 3. 提取欧拉角 (TF2 的 getRPY() 函数使用 double 类型)
    double d_roll, d_pitch, d_yaw;
    m.getRPY(d_roll, d_pitch, d_yaw);

    // 4. 将高精度结果转回 float 输出
    roll = (float)d_roll;
    pitch = (float)d_pitch;
    yaw = (float)d_yaw;
}

} // namespace rm_serial_driver




#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
