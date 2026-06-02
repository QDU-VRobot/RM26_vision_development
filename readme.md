# ROS2 与 LibXR 融合串口通信框架

本仓库实现 ROS2（上位机）与下位机（嵌入式控制板）之间的**双向串口通信**。
桥接核心借助 [LibXR](https://github.com/Jiu-xiao/libxr) 的 Topic 机制：把下位机发来的二进制数据解析成 ROS2 消息发布出去，同时把 ROS2 侧的指令打包通过 UART 下发给下位机。

整体数据流：

```
下位机  --UART-->  SharedTopic(接收/解析)  -->  LibXR Topic  -->  ROS2 Publisher  -->  ROS2 网络
ROS2 网络  -->  ROS2 Subscriber  -->  LibXR Topic  -->  SharedTopicClient(打包/发送)  --UART-->  下位机
```

## 仓库结构

这是一个 colcon 工作区，`src/` 下包含三个功能包：

| 包名 | 作用 |
| --- | --- |
| `ros2_libxr` | 核心桥接节点。串口收发、LibXR 话题与 ROS2 话题的转换都在这里 |
| `referee_interfaces` | 裁判系统相关自定义消息（RobotStatus、GameStatus、RfidStatus、SentryState、SetPose 等） |
| `auto_aim_interfaces` | 自瞄相关自定义消息（Send、Velocity、Target 等） |

`ros2_libxr` 通过 git submodule 引入三个外部依赖：

| 子模块 | 路径 | 说明 |
| --- | --- | --- |
| `libxr` | `src/ros2_libxr/libxr` | LibXR 主库，CMake 目标名为 `xr` |
| `SharedTopic` | `src/ros2_libxr/include/SharedTopic` | UART 多话题接收/解析服务端（下位机 → 上位机） |
| `SharedTopicClient` | `src/ros2_libxr/include/SharedTopicClient` | UART 多话题打包/转发客户端（上位机 → 下位机） |

核心代码位置：

- 节点实现：`src/ros2_libxr/src/ros2libxr.cpp`
- 接口、数据结构与话题定义：`src/ros2_libxr/include/ros2_libxr/ros2libxr.hpp`

## 快速开始

### 1. 克隆并初始化子模块

```bash
git clone <repo-url> ros2_libxr
cd ros2_libxr
git submodule update --init --recursive
```

### 2. 构建

在工作区根目录（包含 `src/` 的目录）构建。由于 `ros2_libxr` 依赖两个接口包，首次构建建议整包构建：

```bash
colcon build
source install/setup.bash
```

如只想单独编译桥接节点（接口包已编译过）：

```bash
colcon build --packages-select ros2_libxr
source install/setup.bash
```

### 3. 运行

```bash
# 方式一：直接运行节点
ros2 run ros2_libxr ros2_libxr_node

# 方式二：使用 launch 文件
ros2 launch ros2_libxr ros2_libxr_launch.py
```

> 构建后务必 `source install/setup.bash`，否则运行时可能找不到 `.so`。

## 串口配置

串口在 `RMSerialDriver` 构造函数中通过 `LibXR::LinuxUART` 初始化（`ros2libxr.cpp`）：

```cpp
uart_client = std::make_unique<LibXR::LinuxUART>(
    "16d0", "1492", "navigation", 115200,
    LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);
```

参数依次为：USB VID（`16d0`）、PID（`1492`）、设备别名（`navigation`）、波特率（`115200`）、校验位、数据位、停止位。
若你的下位机串口设备不同，请按实际 VID/PID 或设备号修改这一行。

节点还会启动一个 LibXR 终端线程（`Terminal`），可通过它在运行时查看接收速率等调试信息。

## 话题接口

### 下位机 → ROS2（接收方向）

| LibXR 话题 | 数据结构 | 经过处理后发布的 ROS2 话题 | ROS2 消息类型 |
| --- | --- | --- | --- |
| `ahrs_quaternion` | `LibXR::Quaternion<float>` | `serial/gimbal_joint_state` | `sensor_msgs/JointState` |
| `yawmotor_angle` | `float` | （缓存为 yaw，参与上面的关节状态） | — |
| `sentry_ref` | `RobotGameRefereePack` | `referee/robot_status` | `referee_interfaces/RobotStatus` |
| | | `referee/game_status` | `referee_interfaces/GameStatus` |
| | | `referee/rfid_status` | `referee_interfaces/RfidStatus` |
| | | `referee/sentry_state` | `referee_interfaces/SentryState` |
| | | `/our_outpost_hp` | `std_msgs/Int32` |

说明：

- `ahrs_quaternion` 收到四元数后，先用 `convert_quaternion_to_euler` 转成欧拉角，pitch 来自 AHRS、yaw 取自 `yawmotor_angle` 缓存值，组成云台 `JointState` 发布。
- `sentry_ref`（`RobotGameRefereePack`）是裁判系统数据汇总包，回调里会拆解成机器人状态、比赛状态、RFID 增益点位图、哨兵姿态、前哨站血量等多条 ROS2 话题分别发布。

### ROS2 → 下位机（发送方向）

| 订阅的 ROS2 话题 | ROS2 消息类型 | 转发到的 LibXR 话题 | 数据结构 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | `chassis_data` | `move_vec` |
| `/referee/set_pose` | `referee_interfaces/SetPose` | `sentry_state` | `uint8_t` |

说明：

- `/cmd_vel` 的线速度/角速度被换算为底盘 `move_vec{vx, vy, wz}` 下发（注意代码里做了坐标变换：`vx = -linear.y`、`vy = linear.x`）。
- `/referee/set_pose` 用于把哨兵姿态切换指令（`pose` 字段）下发给下位机。

## 如何新增话题

新增话题需要在「话题注册表」和「收发逻辑」两处同步修改，二者的话题名必须与下位机约定一致。

### 接收方向（下位机 → ROS2）

1. 在 `ros2libxr.hpp` 的 `XRobotMain` 中，把话题名加入 **`SharedTopic`** 的列表：

   ```cpp
   static SharedTopic shared_topic_rx(
       hw, appmgr, "uart_client", 256,
       {{"ahrs_quaternion"}, {"yawmotor_angle"}, {"sentry_ref"}, {"your_new_topic"}});
   ```

2. 在 `RMSerialDriver` 构造函数中创建对应的 LibXR 话题并赋值给成员变量：

   ```cpp
   your_new_topic_ = LibXR::Topic::CreateTopic<YourDataType>("your_new_topic");
   ```

3. 创建 ROS2 发布者，并注册回调，在回调中把数据转换后用 `self` 发布：

   ```cpp
   void (*cb_fun)(bool, RMSerialDriver *self, LibXR::RawData &data) =
       [](bool, RMSerialDriver *self, LibXR::RawData &data) {
         auto payload = reinterpret_cast<YourDataType *>(data.addr_);
         // ... 转换并 self->your_pub_->publish(...)
       };
   auto cb = LibXR::Topic::Callback::Create(cb_fun, this);
   your_new_topic_.RegisterCallback(cb);
   ```

### 发送方向（ROS2 → 下位机）

1. 在 `XRobotMain` 中，把话题名加入 **`SharedTopicClient`** 的列表：

   ```cpp
   static SharedTopicClient shared_topic_tx(
       hw, appmgr, "uart_client", 16,
       {{"chassis_data"}, {"sentry_state"}, {"your_tx_topic"}});
   ```

2. 在构造函数中创建对应 LibXR 话题，并在 ROS2 订阅回调里 `Publish` 给该话题：

   ```cpp
   your_tx_topic_ = LibXR::Topic::CreateTopic<YourTxType>("your_tx_topic");
   // 订阅回调内：
   your_tx_topic_.Publish(payload);
   ```

> 若新增的数据需要对应新的 ROS2 消息类型，记得在 `referee_interfaces` 或 `auto_aim_interfaces` 的 `CMakeLists.txt` 中注册 `.msg` 文件，并在 `ros2_libxr` 中添加依赖。

## 调试要点

- 回调函数必须通过传入的 `self`（`RMSerialDriver*`）访问类成员，不要在 lambda 中直接使用未捕获的 `this`。
- 调试打印三选一：`XR_LOG_INFO(...)`（LibXR）、`RCLCPP_INFO(self->get_logger(), ...)`（ROS2）、`std::cout`。
- LibXR 终端的 `shared_topic:<uart_name>` 命令支持 `monitor` 子命令，可实时查看接收带宽（Mbps），用于排查串口吞吐问题。
- 修改子模块后，注意 `git submodule update --recursive`，避免使用过期的头文件版本。
- 构建后记得 `source install/setup.bash`。

