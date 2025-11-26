#include "rm_serial_driver/ros2libxr.hpp"

using namespace std::chrono_literals;

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions& options)
    : Node("rm_serial_driver", options)
{
  /*LibXR串口初始化*/
  LibXR::PlatformInit();
  peripherals_ = std::make_unique<LibXR::HardwareContainer>();
  ramfs_ = std::make_unique<LibXR::RamFS>();
  uart_client_ = std::make_unique<LibXR::LinuxUART>(
      "0483", "5740", 115200, LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);
  terminal_ = std::make_unique<LibXR::Terminal<1024, 64, 16, 128>>(*ramfs_);
  term_thread_ = std::make_unique<LibXR::Thread>();
  term_thread_->Create(terminal_.get(), LibXR::Terminal<1024, 64, 16, 128>::ThreadFun,
                       "terminal", 81900, LibXR::Thread::Priority::MEDIUM);
  static LibXR::HardwareContainer peripherals{
      LibXR::Entry<LibXR::RamFS>({*ramfs_, {"ramfs"}}),
      LibXR::Entry<LibXR::UART>({*uart_client_, {"uart_client"}}),
  };

  /*===================== LibXR话题创建 =====================*/

  // 从下位机接收的话题
  auto ahrs_euler_topic = LibXR::Topic::CreateTopic<LibXR::Quaternion<float>>(
      "ahrs_quaternion");  // 云台四元数

  // referee domain - bullet_speed
  LibXR::Topic::Domain referee_domain = LibXR::Topic::Domain("referee");
  bullet_speed_topic_ = LibXR::Topic::CreateTopic<float>("bullet_speed", &referee_domain);

  // tracker domain - 发送到下位机的话题
  LibXR::Topic::Domain tracker_domain = LibXR::Topic::Domain("tracker");
  target_eulr_topic_ =
      LibXR::Topic::CreateTopic<LibXR::EulerAngle<float>>("target_eulr", &tracker_domain);
  fire_notify_topic_ = LibXR::Topic::CreateTopic<uint8_t>("fire_notify", &tracker_domain);

  /*===================== ROS2发布者 =====================*/

  // 云台关节状态发布者
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "serial/gimbal_joint_state", rclcpp::QoS(rclcpp::KeepLast(1)));

  // 弹速发布者
  velocity_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::Velocity>("/current_velocity", 10);

  /*===================== ROS2订阅者 =====================*/

  // 订阅 /tracker/send
  send_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
      "/tracker/send", rclcpp::SensorDataQoS(),
      std::bind(&RMSerialDriver::SendCallBack, this, std::placeholders::_1));

  /*===================== LibXR应用程序入口函数 =====================*/
  XRobotMain(peripherals);

  /*===================== LibXR回调注册 =====================*/

  // 云台姿态回调函数 (保留原有)
  void (*ahrs_euler_cb_fun)(bool, RMSerialDriver* self, LibXR::RawData& data) =
      [](bool, RMSerialDriver* self, LibXR::RawData& data)
  {
    auto quat = reinterpret_cast<LibXR::Quaternion<float>*>(data.addr_);

    // 调试打印
    XR_LOG_INFO("Serial got quat:%f,%f,%f,%f", quat->w(), quat->x(), quat->y(),
                quat->z());

    rm_serial_driver::gimbal_euler gimbal;
    self->ConvertQuaternionToEuler(quat->x(), quat->y(), quat->z(), quat->w(),
                                   gimbal.roll, gimbal.pitch, gimbal.yaw);

    // ROS2发布云台关节状态
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = self->now();
    joint_state.name.push_back("gimbal_pitch_joint");
    joint_state.name.push_back("gimbal_yaw_joint");
    joint_state.position.push_back(gimbal.pitch);
    joint_state.position.push_back(gimbal.yaw);
    self->joint_state_pub_->publish(joint_state);
  };
  auto ahrs_euler_cb = LibXR::Topic::Callback::Create(ahrs_euler_cb_fun, this);
  ahrs_euler_topic.RegisterCallback(ahrs_euler_cb);

  // 弹速回调函数 (新增)
  void (*bullet_speed_cb_fun)(bool, RMSerialDriver* self, LibXR::RawData& data) =
      [](bool, RMSerialDriver* self, LibXR::RawData& data)
  {
    auto bullet_speed = reinterpret_cast<float*>(data.addr_);

    // ROS2发布弹速
    auto_aim_interfaces::msg::Velocity velocity_msg;
    velocity_msg.header.stamp = self->now();
    velocity_msg.velocity = static_cast<double>(*bullet_speed);
    self->velocity_pub_->publish(velocity_msg);
  };
  auto bullet_speed_cb = LibXR::Topic::Callback::Create(bullet_speed_cb_fun, this);
  bullet_speed_topic_.RegisterCallback(bullet_speed_cb);

  while (1)
  {
    LibXR::Thread::Sleep(10);  // 发送延迟，10ms
  }
}

/*析构函数*/
RMSerialDriver::~RMSerialDriver() {}

/*Send消息回调函数 - 接收ROS2消息并通过LibXR发送到下位机*/
void RMSerialDriver::SendCallBack(const auto_aim_interfaces::msg::Send::SharedPtr msg)
{
  // 构建目标欧拉角 (yaw, pitch, roll=0)
  LibXR::EulerAngle<float> target_euler;
  target_euler.Yaw() = static_cast<float>(msg->yaw);
  target_euler.Pitch() = static_cast<float>(msg->pitch);
  target_euler.Roll() = 0.0f;

  // 构建开火通知
  uint8_t fire_notify = msg->is_fire ? 1 : 0;

  // 通过LibXR Topic发布到下位机
  target_eulr_topic_.Publish(target_euler);
  fire_notify_topic_.Publish(fire_notify);
}

/*四元数转欧拉角*/
void RMSerialDriver::ConvertQuaternionToEuler(float qx, float qy, float qz, float qw,
                                              float& roll, float& pitch, float& yaw)
{
  tf2::Quaternion q(static_cast<double>(qx), static_cast<double>(qy),
                    static_cast<double>(qz), static_cast<double>(qw));
  tf2::Matrix3x3 m(q);
  double d_roll = NAN, d_pitch = NAN, d_yaw = NAN;
  m.getRPY(d_roll, d_pitch, d_yaw);
  roll = static_cast<float>(d_roll);
  pitch = static_cast<float>(d_pitch);
  yaw = static_cast<float>(d_yaw);
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
