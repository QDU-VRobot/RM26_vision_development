# -----------------------------------------------------------------------------
# 1. CMake 版本和项目定义
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(rm_serial_driver) 

set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# -----------------------------------------------------------------------------
# 2. 查找依赖项
# -----------------------------------------------------------------------------
find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies() 会自动查找 package.xml 中的依赖

# 查找并包含所有 ROS 2 依赖
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
# -----------------------------------------------------------------------------
# 3. 外部库依赖（LibXR）
# -----------------------------------------------------------------------------
# 假设您的 LibXR 库（或包含它的项目）通过 add_subdirectory 引入
# 请根据实际情况调整路径
add_subdirectory(
    ${CMAKE_SOURCE_DIR}/libxr
)

# -----------------------------------------------------------------------------
# 4. 构建：生成节点共享库和可执行文件
# -----------------------------------------------------------------------------

# 4.1. 编译节点共享库 (libros2_libxr.so)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ros2libxr.cpp
)

# 4.2. 设置头文件路径
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  
  # LibXR 库的头文件路径 (请根据 LibXR 实际路径修改)
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../libxr>
)



# 4.3. 链接 ROS 2 依赖 (解决签名冲突的关键)
# 使用 ament_target_dependencies 链接所有 ROS 2 依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  std_srvs
  visualization_msgs
  std_msgs
  serial_driver
  auto_aim_interfaces
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 4.4. 链接外部依赖 (LibXR)
target_link_libraries(${PROJECT_NAME}
  xr # LibXR 的目标库名为 xr
)

# 4.5. 注册 ROS 2 组件和可执行文件
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_serial_driver::RMSerialDriver
  EXECUTABLE ${PROJECT_NAME}_node

)


# -----------------------------------------------------------------------------
# 5. 安装
# -----------------------------------------------------------------------------

# ament_auto_package 宏会自动处理头文件、库文件和可执行文件的安装
ament_auto_package(
  # 安装其他资源文件（如配置文件、启动文件等）
  INSTALL_TO_SHARE
  # config  
  launch
)


# -----------------------------------------------------------------------------
# 1. CMake 版本和项目定义
# -----------------------------------------------------------------------------
# cmake_minimum_required(VERSION 3.16)
# project(ros2_libxr) 

# set(CMAKE_CXX_STANDARD 17) 
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# # -----------------------------------------------------------------------------
# # ⚙️ 新增：通用编译设置和调试选项
# # -----------------------------------------------------------------------------
# ## 强制启用警告
# add_definitions(-Wall -Werror)

# ## 调试选项
# add_compile_options(-g -O0)

# ## 导出编译命令以便使用 clangd (放在 project() 之后)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# # -----------------------------------------------------------------------------


# # -----------------------------------------------------------------------------
# # 2. 查找依赖项
# # -----------------------------------------------------------------------------
# find_package(ament_cmake_auto REQUIRED)
# # ament_auto_find_build_dependencies() 会自动查找 package.xml 中的依赖

# # 查找并包含所有 ROS 2 依赖
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(std_msgs REQUIRED)

# # -----------------------------------------------------------------------------
# # 3. 外部库依赖（LibXR）
# # -----------------------------------------------------------------------------

# # ⚙️ 新增：LibXR 库的特殊变量设置
# set(XROBOT_MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
# set(LIBXR_LOG_OUTPUT_LEVEL 2)

# # 假设您的 LibXR 库（或包含它的项目）通过 add_subdirectory 引入
# # 请根据实际情况调整路径
# add_subdirectory(
#     ${CMAKE_SOURCE_DIR}/libxr
# )

# # -----------------------------------------------------------------------------
# # 4. 构建：生成节点共享库和可执行文件
# # -----------------------------------------------------------------------------

# # 4.1. 编译节点共享库 (libros2_libxr.so)
# ament_auto_add_library(${PROJECT_NAME} SHARED
#   src/ros2libxr.cpp
# )

# # 4.2. 设置头文件路径
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<INSTALL_INTERFACE:include>
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  
#   # LibXR 库的头文件路径 (请根据 LibXR 实际路径修改)
#   # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../libxr>
# )

# # 4.3. 链接 ROS 2 依赖 (解决签名冲突的关键)
# # 使用 ament_target_dependencies 链接所有 ROS 2 依赖
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   rclcpp_components
#   geometry_msgs
#   sensor_msgs
#   tf2
#   tf2_ros
#   std_srvs
#   visualization_msgs
#   std_msgs
# )

# # 4.4. 链接外部依赖 (LibXR)
# target_link_libraries(${PROJECT_NAME}
#   xr # LibXR 的目标库名为 xr
# ) 

# # 4.5. 注册 ROS 2 组件和可执行文件
# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN rm_serial_driver::RMSerialDriver
#   EXECUTABLE ${PROJECT_NAME}_node

# )


# # -----------------------------------------------------------------------------
# # 5. 构建一个非 ROS 可执行文件
# # -----------------------------------------------------------------------------

# # 添加您的可执行文件
# add_executable(
#     ${PROJECT_NAME}_user_app  # 更改名称以避免与 ROS 2 节点可执行文件 ${PROJECT_NAME}_node 冲突
# )

# # 链接 xr 库
# target_link_libraries(
#     ${PROJECT_NAME}_user_app
#     PUBLIC xr
# )

# # 包含 xr 库的接口头文件
# target_include_directories(
#     ${PROJECT_NAME}_user_app
#     PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
# )

# # 包含 User 目录的头文件
# target_include_directories(
#     ${PROJECT_NAME}_user_app
#     PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/User
# )

# # -----------------------------------------------------------------------------
# # 6. 安装
# # -----------------------------------------------------------------------------

# # ament_auto_package 宏会自动处理头文件、库文件和可执行文件的安装
# ament_auto_package(
#   # 安装其他资源文件（如配置文件、启动文件等）
#   INSTALL_TO_SHARE
#   # config  
#   launch
#   User
# )