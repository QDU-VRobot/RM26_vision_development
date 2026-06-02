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
#include "referee_interfaces/msg/game_status.hpp"
#include "referee_interfaces/msg/rfid_status.hpp"
#include "std_msgs/msg/int32.hpp"

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
  uart_client = std::make_unique<LibXR::LinuxUART>("16d0", "1492","navigation", 115200,
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
  sentry_ref_topic_ =
      LibXR::Topic::CreateTopic<RobotGameRefereePack>("sentry_ref");
  sentry_state_topic_ = LibXR::Topic::CreateTopic<uint8_t>("sentry_state");
  
  /* ROS2发布者或订阅者 */
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "serial/gimbal_joint_state", rclcpp::QoS(rclcpp::KeepLast(1)));

  // move_vec_sub = this->create_subscription<geometry_msgs::msg::Twist>(
  //     "/fake_cmd_vel", rclcpp::SensorDataQoS(), 
  //     std::bind(&RMSerialDriver::get_classic, this, std::placeholders::_1));

    move_vec_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS(),
      std::bind(&RMSerialDriver::get_classic, this, std::placeholders::_1));

  sentry_ref_pub_ = this->create_publisher<referee_interfaces::msg::RobotStatus>(
      "referee/robot_status", rclcpp::QoS(rclcpp::KeepLast(1)));

  game_status_pub_ = this->create_publisher<referee_interfaces::msg::GameStatus>(
      "referee/game_status", rclcpp::QoS(rclcpp::KeepLast(1)));

  rfid_status_pub_ = this->create_publisher<referee_interfaces::msg::RfidStatus>(
      "referee/rfid_status", rclcpp::QoS(rclcpp::KeepLast(1)));

  sentry_state_pub_ = this->create_publisher<referee_interfaces::msg::SentryState>(
      "referee/sentry_state", rclcpp::QoS(rclcpp::KeepLast(1)));

  our_outpost_hp_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/our_outpost_hp", rclcpp::QoS(rclcpp::KeepLast(1)));

  set_pose_sub_ = this->create_subscription<referee_interfaces::msg::SetPose>(
      "/referee/set_pose", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const referee_interfaces::msg::SetPose::SharedPtr msg) {
        uint8_t pose = msg->pose;
        RCLCPP_INFO(this->get_logger(), "SetPose received: pose=%d, forwarding to lower machine", pose);
        sentry_state_topic_.Publish(pose);
      });
  


  // game_status_fallback_timer_ = this->create_wall_timer(
  //   1min, [this]() {
  //     RCLCPP_WARN(this->get_logger(),
  //       "No game_progress==4 received in 5min, publishing fallback game_status=4");
  //     referee_interfaces::msg::GameStatus fallback_msg;
  //     fallback_msg.game_type = 0;
  //     fallback_msg.game_progress = 4;
  //     fallback_msg.stage_remain_time = 420;
  //     fallback_msg.sync_time_stamp = 0;
  //     game_status_pub_->publish(fallback_msg);
  //     game_status_fallback_timer_->cancel();
  //   });

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

        sensor_msgs::msg::JointState joint_vision_state;
        joint_vision_state.header.stamp = self->now();
        joint_vision_state.name.push_back("pitch_joint");
        joint_vision_state.name.push_back("yaw_joint");
        joint_vision_state.position.push_back(gimbal_.pitch);
        joint_vision_state.position.push_back(gimbal_.yaw);

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


  /*哨兵裁判数据回调函数*/
  void (*sentry_ref_cb_fun)(bool, RMSerialDriver *self, LibXR::RawData &data) =
      [](bool, RMSerialDriver *self, LibXR::RawData &data) {
        auto sentry_data = reinterpret_cast<RobotGameRefereePack *>(data.addr_);

        referee_interfaces::msg::RobotStatus rs_msg;
        rs_msg.robot_id = sentry_data->robot_status.robot_id;
        rs_msg.robot_level = sentry_data->robot_status.robot_level;
        rs_msg.current_hp = sentry_data->robot_status.current_hp;
        rs_msg.maximum_hp = sentry_data->robot_status.maximum_hp;
        rs_msg.shooter_barrel_cooling_value = sentry_data->robot_status.shooter_barrel_cooling_value;
        rs_msg.shooter_barrel_heat_limit = sentry_data->robot_status.shooter_barrel_heat_limit;
        rs_msg.chassis_power_limit = sentry_data->robot_status.chassis_power_limit;
        rs_msg.power_gimbal_output = sentry_data->robot_status.power_gimbal_output;
        rs_msg.power_chassis_output = sentry_data->robot_status.power_chassis_output;
        rs_msg.power_launcher_output = sentry_data->robot_status.power_launcher_output;
        rs_msg.projectile_allowance_17mm = sentry_data->bullet_17_remain;

        self->sentry_ref_pub_->publish(rs_msg);

        referee_interfaces::msg::GameStatus gs_msg;
        gs_msg.game_type = sentry_data->game_status.game_type;
        gs_msg.game_progress = sentry_data->game_status.game_progress;
        gs_msg.stage_remain_time = sentry_data->game_status.stage_remain_time;
        gs_msg.sync_time_stamp = sentry_data->game_status.sync_time_stamp;
        self->game_status_pub_->publish(gs_msg);

        // if (gs_msg.game_progress == 4 && self->game_status_fallback_timer_) {
        //   self->game_status_fallback_timer_->cancel();
        // }

        // 发布哨兵姿态状态，直接对齐下位机值 (0=进攻, 1=防御, 2=移动)
        referee_interfaces::msg::SentryState ss_msg;
        ss_msg.current_state = sentry_data->sentry_info.current_state;
        self->sentry_state_pub_->publish(ss_msg);

        // RFID 状态解析和发布
        referee_interfaces::msg::RfidStatus rfid_msg;
        uint32_t rfid_bits = sentry_data->rfid.own_base |
          (sentry_data->rfid.own_highland_center << 1) |
          (sentry_data->rfid.enemy_highland_center << 2) |
          (sentry_data->rfid.own_trapezium << 3) |
          (sentry_data->rfid.enemy_trapezium << 4) |
          (sentry_data->rfid.own_slope_before_R1B1 << 5) |
          (sentry_data->rfid.own_slope_after_R1B1 << 6) |
          (sentry_data->rfid.enemy_slope_before_R4B4 << 7) |
          (sentry_data->rfid.enemy_slope_after_R4B4 << 8) |
          (sentry_data->rfid.own_terrain_crossing_up_R2B2 << 9) |
          (sentry_data->rfid.own_terrain_crossing_down_R2B2 << 10) |
          (sentry_data->rfid.enemy_terrain_corrssing_up_R2B2 << 11) |
          (sentry_data->rfid.enemy_terrain_corrssing_down_R2B2 << 12) |
          (sentry_data->rfid.own_terrain_crossing_up_R3B3 << 13) |
          (sentry_data->rfid.own_terrain_crossing_down_R3B3 << 14) |
          (sentry_data->rfid.enemy_terrain_corrssing_up_R3B5 << 15) |
          (sentry_data->rfid.enemy_terrain_corrssing_down_R3B3 << 16) |
          (sentry_data->rfid.own_fortress << 17) |
          (sentry_data->rfid.own_outpost << 18) |
          (sentry_data->rfid.own_blood_supply_unoverlapping << 19) |
          (sentry_data->rfid.own_blood_supply_overlapping << 20) |
          (sentry_data->rfid.own_assemble << 21) |
          (sentry_data->rfid.enemy_assemble << 22) |
          (sentry_data->rfid.center_resource_RMUL << 23);
      
        rfid_msg.base_gain_point = (rfid_bits & (1 << 0)) != 0;
        rfid_msg.central_highland_gain_point = (rfid_bits & (1 << 1)) != 0;
        rfid_msg.enemy_central_highland_gain_point = (rfid_bits & (1 << 2)) != 0;
        rfid_msg.friendly_trapezoidal_highland_gain_point = (rfid_bits & (1 << 3)) != 0;
        rfid_msg.enemy_trapezoidal_highland_gain_point = (rfid_bits & (1 << 4)) != 0;
        rfid_msg.friendly_fly_ramp_front_gain_point = (rfid_bits & (1 << 5)) != 0;
        rfid_msg.friendly_fly_ramp_back_gain_point = (rfid_bits & (1 << 6)) != 0;
        rfid_msg.enemy_fly_ramp_front_gain_point = (rfid_bits & (1 << 7)) != 0;
        rfid_msg.enemy_fly_ramp_back_gain_point = (rfid_bits & (1 << 8)) != 0;
        rfid_msg.friendly_central_highland_lower_gain_point = (rfid_bits & (1 << 9)) != 0;
        rfid_msg.friendly_central_highland_upper_gain_point = (rfid_bits & (1 << 10)) != 0;
        rfid_msg.enemy_central_highland_lower_gain_point = (rfid_bits & (1 << 11)) != 0;
        rfid_msg.enemy_central_highland_upper_gain_point = (rfid_bits & (1 << 12)) != 0;
        rfid_msg.friendly_highway_lower_gain_point = (rfid_bits & (1 << 13)) != 0;
        rfid_msg.friendly_highway_upper_gain_point = (rfid_bits & (1 << 14)) != 0;
        rfid_msg.enemy_highway_lower_gain_point = (rfid_bits & (1 << 15)) != 0;
        rfid_msg.enemy_highway_upper_gain_point = (rfid_bits & (1 << 16)) != 0;
        rfid_msg.friendly_fortress_gain_point = (rfid_bits & (1 << 17)) != 0;
        rfid_msg.friendly_outpost_gain_point = (rfid_bits & (1 << 18)) != 0;
        rfid_msg.friendly_supply_zone_non_exchange = (rfid_bits & (1 << 19)) != 0;
        rfid_msg.friendly_supply_zone_exchange = (rfid_bits & (1 << 20)) != 0;
        rfid_msg.friendly_big_resource_island = (rfid_bits & (1 << 21)) != 0;
        rfid_msg.enemy_big_resource_island = (rfid_bits & (1 << 22)) != 0;
        rfid_msg.center_gain_point = (rfid_bits & (1 << 23)) != 0;
        self->rfid_status_pub_->publish(rfid_msg);

        std_msgs::msg::Int32 outpost_hp_msg;
        outpost_hp_msg.data = static_cast<int32_t>(sentry_data->our_outpose);
        self->our_outpost_hp_pub_->publish(outpost_hp_msg);
      };
  auto sentry_ref_cb = LibXR::Topic::Callback::Create(sentry_ref_cb_fun, this);
  sentry_ref_topic_.RegisterCallback(sentry_ref_cb);

  
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
