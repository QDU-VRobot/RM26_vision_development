#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"
#include "rclcpp/rclcpp.hpp"

#include "armor_executor/TableUser.hpp"

namespace rm_auto_aim {

class SolveTrajectory {
public:
  /// 弹道常量;
  static constexpr float GRAVITY = 9.788f;
  static constexpr int MAX_ARMOR_NUM = 4;

  /// 计算模式
  enum CalculateMode {
    NORMAL = 0,      ///< 正常迭代计算
    TABLE_LOOKUP = 1 ///< 查表法
  };

  enum FireLogicMode { OUTPOST = 0, SPIN = 1, COMMON = 2, BUFF = 3 };

  enum AimingState { AIMING = 0, TURNING = 1 };

  enum SpecialArmor { LOST = -2, CENTER = -1 };

  struct ArmorInfo {
    float x;   ///< world-x [m]
    float y;   ///< world-y [m]
    float z;   ///< world-z [m]
    float yaw; ///< armor yaw in world frame [rad]
  };

  SolveTrajectory(float k, int bias_time, float s_bias, float z_bias,
                  float pitch_bias, CalculateMode calculate_mode,
                  const TableUser::TableConfig &table_config);

  void init(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg);

  void rebuild();

  void
  autoSolveTrajectory(float &pitch, float &yaw, bool &is_fire, float &aim_x,
                      float &aim_y, float &aim_z,
                      const auto_aim_interfaces::msg::Target::SharedPtr msg);

private:
  float monoDirectionalAirResistanceModel(float s, float v, float angle);

  float solvePitch(float x, float y, float z);
  float solveYaw(float x, float y);
  void updateSolveState(int &selected_idx, float &pitch, float &yaw,
                        bool &is_fire, float &aim_x, float &aim_y, float &aim_z,
                        const auto_aim_interfaces::msg::Target::SharedPtr &msg);

  bool canFire(float aim_yaw, float max_yaw_diff,
               const auto_aim_interfaces::msg::Target::SharedPtr &msg);

  void calculateArmorPosition(
      const auto_aim_interfaces::msg::Target::SharedPtr &msg);
  void
  predictArmorPosition(const auto_aim_interfaces::msg::Target::SharedPtr &msg,
                       float time_delay);

  int selectArmor(const auto_aim_interfaces::msg::Target::SharedPtr &msg);

  /// 开火逻辑
  void fireLogicIsTop(float &pitch, float &yaw, bool &is_fire, float &aim_x,
                      float &aim_y, float &aim_z,
                      const auto_aim_interfaces::msg::Target::SharedPtr &msg);
  void fireLogicDefault(float &pitch, float &yaw, bool &is_fire, float &aim_x,
                        float &aim_y, float &aim_z,
                        const auto_aim_interfaces::msg::Target::SharedPtr &msg);

  // 配置参数
  const float k_;
  const int bias_time_;
  const float s_bias_;
  const float z_bias_;
  const float pitch_bias_;

  // 状态变量
  float current_v_{12.0f};
  float fly_time_{0.0f};

  // 模式选择
  CalculateMode calculate_mode_;
  FireLogicMode fire_logic_mode_{FireLogicMode::COMMON};

  // 数据存储
  ArmorInfo tar_position_[MAX_ARMOR_NUM];
  ArmorInfo pre_position_[MAX_ARMOR_NUM];

  // 工具类
  TableUser table_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("solve_trajectory")};

  float pre_x_center_{0.0f};
  float pre_y_center_{0.0f};
  float pre_z_center_{0.0f};
  float pre_yaw_{0.0f};

  // 上次状态记录
  float last_pitch_;
  float last_yaw_;
  int last_selected_idx_{SpecialArmor::LOST};
  float last_x_v_{0.0f};
  float last_y_v_{0.0f};
};

} // namespace rm_auto_aim