# ROS2与LibXR融合串口通信

## 当前功能
- 从电控获取弹速`bullet_speed`与云台姿态四元数`ahrs_quaternion`
- `\

- 发送开火指令与云台期望欧拉角至电控



## 概述  
本仓库桥接 ROS2 与 LibXR（上位机 ↔ 本库 ↔ 下位机）。代码位于 src/ros2libxr.cpp、接口声明在 include/rm_serial_driver/ros2libxr.hpp。

### 快速开始
- 构建（在工作区根目录，包含 src/ 的上级目录）：
  ```
  colcon build --packages-select rm_serial_driver
  source install/setup.bash
  ```
- 直接运行可执行文件或使用 ros2 run：
  ```
  ros2 run rm_serial_driver ros2_libxr_node
  ```

### 如何接收下位机消息（下位机 -> 上位机 -> ROS2）
1. 在代码中创建 LibXR 话题，例如：
   ```
   auto ahrs_euler_topic = LibXR::Topic::CreateTopic<LibXR::Quaternion<float>>("ahrs_quaternion");
   ```
2. 创建 ROS2 发布者：
   ```
   joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 1);
   ```
3. 编写回调并注册（回调中使用 self 发布 ROS2）：
   ```cpp
   void (*ahrs_euler_cb_fun)(bool, RMSerialDriver *self, LibXR::RawData &data) =
      [](bool, RMSerialDriver *self, LibXR::RawData &data) {
        auto quat = reinterpret_cast<LibXR::Quaternion<float> *>(data.addr_);

        //调试打印三种方式任选其一
        XR_LOG_INFO("Serial got quat:%f,%f,%f,%f", quat->w(),quat->x(), quat->y(), quat->z()); 
        // RCLCPP_INFO(self->get_logger(),"Serial got quat:%f,%f,%f,%f", quat->w(),quat->x(), quat->y(), quat->z());
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
        joint_state.position.push_back(gimbal_.yaw);
        self->joint_state_pub_->publish(joint_state);
      };
      auto ahrs_euler_cb = LibXR::Topic::Callback::Create(ahrs_euler_cb_fun, this);
      ahrs_euler_topic.RegisterCallback(ahrs_euler_cb);
   ```

### 如何向下位机发送消息（上位机 -> 下位机）
1. 创建发送用的 LibXR 话题（示例）：
   ```
   auto chassis_data_topic = LibXR::Topic::CreateTopic<rm_serial_driver::move_vec>("chassis_data");
   ```
2. 在循环中发布：
   ```cpp
   while (1) {
     chassis_data_topic.Publish(move_data);
     LibXR::Thread::Sleep(10); // ms
   }
   ```

### 添加新话题注意事项
- 若新增从下位机到上位机的话题，请在 SharedTopic（头文件）中加入话题名；若新增上位机到下位机的话题，则在 SharedTopicClient 中加入。
- 修改位置和格式请参考 include/rm_serial_driver/ros2libxr.hpp 中的 SharedTopic/SharedTopicClient 定义。

### 调试要点
- 回调要通过传入的 self（RMSerialDriver*）访问成员（不要在 lambda 中直接使用未捕获的 this）。
- 使用 XR_LOG_INFO、RCLCPP_INFO 或 std::cout 三种方式任选其一调试打印。
- 构建后记得 source install/setup.bash，否则运行时可能找不到 .so。

### 文件位置提示
- 主要实现： src/ros2libxr.cpp  
- 接口与话题定义： include/rm_serial_driver/ros2libxr.hpp
