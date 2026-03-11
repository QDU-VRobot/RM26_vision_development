# pb_rm_interfaces

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Build](https://github.com/SMBU-PolarBear-Robotics-Team/pb_rm_interfaces/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb_rm_interfaces/actions/workflows/ci.yml)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

ROS2 interfaces (.msg, .srv, .action) used in the StandardRobot++ project.

## msg

云台和射击使用自定义消息类型，

* GimbalCmd.msg：云台控制命令，使用绝对位置，单位为弧度
* ShootCmd.msg：射击命令，包含射击子弹数
* 底盘控制命令使用 ROS2 的 `geometry_msgs/msg/Twist`。
* referee

    当前对应串口协议版本：[V1.7.0 (20241225)](https://terra-1-g.djicdn.com/b2a076471c6c4b72b574a977334d3e05/RoboMaster%20%E8%A3%81%E5%88%A4%E7%B3%BB%E7%BB%9F%E4%B8%B2%E5%8F%A3%E5%8D%8F%E8%AE%AE%E9%99%84%E5%BD%95%20V1.7.0%EF%BC%8820241225%EF%BC%89.pdf)



我在运行一个ros2导航与决策树的项目中发现了以下的问题：我写了一个基于hp的导航用来初步测试我的决策树是否能跑的通，主要在gazebo和rviz上面进行测试，鉴于目前没有裁判系统，我就又写了一个模拟裁判系统通信的包来和我的决策树进行通信，目前编译正常，且在没有运行导航和仿真的时候通信正常，但是只要打开仿真加导航，决策树的终端就会卡住，具体情况是：CTRL+C无法退出，会i一直输出ctrl+c的键值，在新的终端用topic list和echo也会出现相同的问题，而且导航那边没有按照我的决策树所写的逻辑进行运动，初步怀疑是命名空间的问题，但是表现出来的不是他的问题，然后就是考虑了是不是导航和仿真让cpu负载过大导致决策树卡住，因为我的4核心都跑到了90以上，再然后就是考虑决策树的逻辑写错了。我该怎么排除这些问题，现在我将把可能相关的文件传给你，如果还需要更多再跟我讲