#include "ros2_libxr/ros2libxr.hpp"

// ROS2库
#include <cstdio>
#include <iterator>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

// TF2
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// C++
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

//ROS2消息包
#include "geometry_msgs/msg/twist.hpp"
#include "referee_interfaces/msg/robot_status.hpp"

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

  /*LibXR串口初始化*/
  LibXR::PlatformInit();
  peripherals = std::make_unique<LibXR::HardwareContainer>();
  ramfs = std::make_unique<LibXR::RamFS>();
  uart_client = std::make_unique<LibXR::LinuxUART>("16d0", "1492", 115200, 
                                                     LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);
  terminal = std::make_unique<LibXR::Terminal<1024, 64, 16, 128>>(*ramfs);
  term_thread = std::make_unique<LibXR::Thread>();
  term_thread->Create(terminal.get(),
                      LibXR::Terminal<1024, 64, 16, 128>::ThreadFun, "terminal",
                      81900, LibXR::Thread::Priority::MEDIUM);

  /*创建硬件容器（删除重复声明）*/
  LibXR::HardwareContainer hw_container{
      LibXR::Entry<LibXR::RamFS>({*ramfs, {"ramfs"}}),
      LibXR::Entry<LibXR::UART>({*uart_client, {"uart_client"}}),
  };

  /*LibXR话题创建 - 直接赋值给成员变量*/
  ahrs_euler_topic_ = LibXR::Topic::CreateTopic<LibXR::Quaternion<float>>("ahrs_quaternion");
  move_vec_topic_ = LibXR::Topic::CreateTopic<move_vec>("chassis_data");
  yawmotor_angle_topic_= LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  sentry_hp_topic_ = LibXR::Topic::CreateTopic<SentryData>("sentry_hp");
  
  /* ROS2发布者或订阅者 */
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "serial/gimbal_joint_state", rclcpp::QoS(rclcpp::KeepLast(1)));

  move_vec_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS(), 
      std::bind(&RMSerialDriver::get_classic, this, std::placeholders::_1));

  sentry_hp_pub_ = this->create_publisher<referee_interfaces::msg::RobotStatus>(
      "referee/robot_status", rclcpp::QoS(rclcpp::KeepLast(1)));

  /* LibXR应用程序入口函数 */
  XRobotMain(hw_container);

  /* 云台位姿回调函数 */
  void (*ahrs_euler_cb_fun)(bool, RMSerialDriver *self, LibXR::RawData &data) =
      [](bool, RMSerialDriver *self, LibXR::RawData &data) {
        auto quat = reinterpret_cast<LibXR::Quaternion<float> *>(data.addr_);

        // std::cout<<"Serial got quat:"<<quat->w()<<","<<quat->x()<<","<< quat->y()<<","<< quat->z()<<std::endl;

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
        joint_state.position.push_back(self->yawmotor_angle_data);

        self->joint_state_pub_->publish(joint_state);
      };
  auto ahrs_euler_cb = LibXR::Topic::Callback::Create(ahrs_euler_cb_fun, this);
  ahrs_euler_topic_.RegisterCallback(ahrs_euler_cb);

  /*云台相对底盘yaw回调函数*/

    void (*yawmotor_angle_cb_fun)(bool, RMSerialDriver *self, LibXR::RawData &data) =
      [](bool, RMSerialDriver *self, LibXR::RawData &data) {
        auto quat = reinterpret_cast<float*>(data.addr_);
        self->yawmotor_angle_data=static_cast<float>(*quat);
      };
  auto yawmotor_angle_cb = LibXR::Topic::Callback::Create(yawmotor_angle_cb_fun, this);
  yawmotor_angle_topic_.RegisterCallback(yawmotor_angle_cb);


  /*哨兵血量回调函数*/
  void (*sentry_hp_cb_fun)(bool, RMSerialDriver *self, LibXR::RawData &data) =
      [](bool, RMSerialDriver *self, LibXR::RawData &data) {
        auto sentry_data = reinterpret_cast<SentryData *>(data.addr_);
          referee_interfaces::msg::RobotStatus msg;
          msg.current_hp = sentry_data->remain_hp;
          self->sentry_hp_pub_->publish(msg);
      };
  auto sentry_hp_cb = LibXR::Topic::Callback::Create(sentry_hp_cb_fun, this);
  sentry_hp_topic_.RegisterCallback(sentry_hp_cb);
}


/*析构函数*/

RMSerialDriver::~RMSerialDriver() {}

/*四元数转欧拉角*/
void RMSerialDriver::convert_quaternion_to_euler(
    float qx, float qy, float qz, float qw,
    float &roll, float &pitch, float &yaw) {
    tf2::Quaternion q((double)qx, (double)qy, (double)qz, (double)qw);
    tf2::Matrix3x3 m(q);
    double d_roll, d_pitch, d_yaw;
    m.getRPY(d_roll, d_pitch, d_yaw);
    roll = (float)d_roll;
    pitch = (float)d_pitch;
    yaw = (float)d_yaw;
}

/*底盘运动数据回调函数*/
void RMSerialDriver::get_classic(const geometry_msgs::msg::Twist::SharedPtr twi) {
    move_.vx = -twi->linear.y;
    move_.vy = twi->linear.x;
    move_.wz = twi->angular.z;
    std::cout << "Received cmd_vel: vx=" << move_.vx 
              << ", vy=" << move_.vy 
              << ", wz=" << move_.wz << std::endl;
    move_vec_topic_.Publish(move_);
}

} // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
