#include "armor_executor/SolveTrajectory.hpp"
#include "armor_executor/TableUser.hpp"
#include <algorithm>
#include <cmath>
#include <math.h>
#include <utility>

namespace rm_auto_aim {

SolveTrajectory::SolveTrajectory(float k, int bias_time, float s_bias,
                                 float z_bias, float pitch_bias,
                                 CalculateMode calculate_mode,
                                 const TableUser::TableConfig &table_config)
    : k_(k), bias_time_(bias_time), s_bias_(s_bias), z_bias_(z_bias),
      pitch_bias_(pitch_bias), calculate_mode_(calculate_mode),
      table_(table_config) {
  if (calculate_mode_ == CalculateMode::TABLE_LOOKUP) {
    table_.Init();
    if (table_.IsInit()) {
      RCLCPP_INFO(logger_, "Trajectory table initialized successfully");
    } else {
      calculate_mode_ = CalculateMode::NORMAL;
      RCLCPP_WARN(logger_, "Using normal calculation mode");
    }
  }
}

void SolveTrajectory::init(
    const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg) {
  if (!std::isnan(velocity_msg->velocity)) {
    current_v_ = velocity_msg->velocity;
    RCLCPP_DEBUG(logger_, "Velocity updated: %.2f m/s", current_v_);
  } else {
    RCLCPP_WARN(logger_, "Invalid velocity, using default: 12.0 m/s");
    current_v_ = 12.0f;
  }
}

void SolveTrajectory::rebuild() { last_selected_idx_ = SpecialArmor::LOST; }

//整车建模，计算各装甲板位置
void SolveTrajectory::calculateArmorPosition(
    const auto_aim_interfaces::msg::Target::SharedPtr &msg) {

  for (int i = 0; i < msg->armors_num; i++) {

    float radius = i % 2 ? msg->radius_2 : msg->radius_1;

    float tmp_yaw = msg->yaw + i * 2.0f * M_PI / msg->armors_num;

    tar_position_[i].x = msg->position.x - radius * cos(tmp_yaw);
    tar_position_[i].y = msg->position.y - radius * sin(tmp_yaw);
    tar_position_[i].z = msg->position.z;
    tar_position_[i].yaw = tmp_yaw;
  }
}

// 从图片时间到打到的时间：自瞄处理的时间+电控延迟(从视觉发信号到电机动和发弹延迟)+云台转动时间+飞行时间
// msg消息的频率即我们发送开火指令的频率，这可以作为我们的步长时间
void SolveTrajectory::predictArmorPosition(
    const auto_aim_interfaces::msg::Target::SharedPtr &msg, float time_delay) {

  pre_x_center_ = msg->position.x + msg->velocity.x * time_delay;
  pre_y_center_ = msg->position.y + msg->velocity.y * time_delay;
  pre_z_center_ = msg->position.z;
  pre_yaw_ = msg->yaw + msg->v_yaw * time_delay;

  for (int i = 0; i < msg->armors_num; i++) {

    float radius = i % 2 ? msg->radius_2 : msg->radius_1;

    float tmp_yaw = pre_yaw_ + i * 2.0f * M_PI / msg->armors_num;

    pre_position_[i].x = pre_x_center_ - radius * cos(tmp_yaw);
    pre_position_[i].y = pre_y_center_ - radius * sin(tmp_yaw);
    pre_position_[i].z = msg->position.z;
    pre_position_[i].yaw = std::fmod(tmp_yaw + M_PI, 2.0f * M_PI) - M_PI;
  }
}

// 计算简化单向空气阻力模型下的弹道高度，用于正常模式
float SolveTrajectory::monoDirectionalAirResistanceModel(float s, float angle,
                                                         float v) {
  float cos_angle = cos(angle);
  if (cos_angle <= 0) {
    RCLCPP_WARN(logger_, "Invalid angle: cos(angle) <= 0");
    fly_time_ = 0;
    return 0;
  }

  fly_time_ = (exp(k_ * s) - 1) / (k_ * v * cos_angle);

  if (fly_time_ < 0) {
    RCLCPP_WARN(logger_, "Exceeding maximum range! s: %.2f, v: %.2f", s, v);
    fly_time_ = 0;
    return 0;
  }

  return v * sin(angle) * fly_time_ - GRAVITY * fly_time_ * fly_time_ / 2;
}

// 计算俯仰角(两种模式)
float SolveTrajectory::solvePitch(float x, float y, float z) {
  // 计算水平距离
  float distance = sqrt(x * x + y * y);
  float target_s = distance + s_bias_;
  float target_z = z + z_bias_;

  float pitch = 0.0f;

  if (calculate_mode_ == CalculateMode::TABLE_LOOKUP && table_.IsInit()) {
    // 查表法
    auto res = table_.Check(target_s, target_z);
    fly_time_ = res.t;
    pitch = static_cast<float>(res.pitch) + pitch_bias_;
    RCLCPP_DEBUG(logger_, "Table lookup - s: %.2f, z: %.2f, pitch: %.4f",
                 target_s, target_z, pitch);
  } else {
    // 正常模式下的迭代计算
    float z_temp = target_z;

    for (int i = 0; i < 20; ++i) {
      if (std::isnan(z_temp)) {
        RCLCPP_ERROR(logger_, "z_temp is NaN during iteration");
        return 0.0f;
      }

      pitch = atan2(z_temp, target_s);
      float z_actual =
          monoDirectionalAirResistanceModel(target_s, pitch, current_v_);
      float dz = 0.3f * (target_z - z_actual);
      z_temp += dz;

      if (fabsf(dz) < 1e-5f) {
        RCLCPP_DEBUG(logger_, "Pitch convergence after %d iterations", i + 1);
        break;
      }
    }
    pitch += pitch_bias_;
  }
  return pitch;
}

float SolveTrajectory::solveYaw(float x, float y) { return atan2(y, x); }

float fast_atan(float x, float y) {
  float x_y = y / x;
  float x_y_2 = x_y * x_y;
  return x_y * (0.99997726f + x_y_2 * (-0.33262347f + x_y_2 * 0.19354346f));
}

// 判断是否满足开火条件,保守打击，只打真正在跟踪的装甲板
bool SolveTrajectory::canFire(
    float aim_yaw, float max_yaw_diff,
    const auto_aim_interfaces::msg::Target::SharedPtr &msg) {
  // // auto tolerance = std::sqrt(tools::square(target_x) +
  // // tools::square(target_y)) > judge_distance_ second_tolerance_ :
  // // first_tolerance_;
  // // float max_yaw_diff = distance > 2 ? 0.1 : 0.05; // 根据距离调整阈值

  // float time_rotation = fabsf((yaw - cam_yaw)) / 0.58;
  // if (time_rotation > time_delay) {
  //   RCLCPP_DEBUG(logger_,
  //                "Fire check - yaw rotation time %.3f exceeds delay %.3f",
  //                time_rotation, time_delay);
  //   return false;
  // }
  // float distance = std::sqrt(x * x + y * y);
  // float max_yaw_diff = distance > 2 ? 0.1 : 0.05; // 根据距离调整阈值
  // float yaw_diff = fabsf(yaw - cam_yaw);
  // bool can_fire = yaw_diff < max_yaw_diff &&
  //                 // (v_y + std::sin(v_yaw * r)) / x < 0.58 &&
  //                 cam_to_x < 0.15; // 放宽阈值以提高稳定性

  // // RCLCPP_DEBUG(logger_,
  // //              "Fire check - yaw: %.3f, predicted: %.3f, diff: %.3f,
  // fire:
  // //              %d", tmp_yaw, predicted_yaw, yaw_diff, should_fire);
  // return can_fire;

  return fabs(msg->velocity.x - last_x_v_) < 0.1f &&
         fabs(msg->velocity.y - last_y_v_) < 0.1f &&
         fabsf(aim_yaw - msg->cam_yaw) < max_yaw_diff &&
         fabsf(msg->cam_to_x) < 0.07f;
}

// 选择最优装甲板,使得同样时间里aiming时间占比最长，且尽量连续,尽量以中心展开
int SolveTrajectory::selectArmor(
    const auto_aim_interfaces::msg::Target::SharedPtr &msg) {
  int selected_idx = -1;
  // 当无可开火装甲板时，选择到下一装甲板出现的位置预瞄
  float min_yaw = std::numeric_limits<float>::max();
  for (int i = 0; i < msg->armors_num; i++) {
    float aim_yaw =
        pre_position_[selected_idx].yaw +
        solveYaw(pre_position_[selected_idx].y, pre_position_[selected_idx].x);
    if (aim_yaw < min_yaw) {
      min_yaw = aim_yaw;
      selected_idx = i;
    }
  }
  return selected_idx;
}

// if (selected_idx == -1) {
//   float min_approachest = std::numeric_limits<float>::max();
//   for (int i = 0; i < msg->armors_num; i++) {
//     float approachest = msg->v_yaw * pre_position_[i].yaw;
//     if (approachest < 0 && approachest > min_approachest) {
//       min_approachest = approachest;
//       selected_idx = i;
//     }
//   }
//   RCLCPP_DEBUG(logger_, "Selected armor index: %d", selected_idx);
//   return selected_idx;
// }

// 不择板，判断此时发弹能否打击到目标
void SolveTrajectory::fireLogicIsTop(
    float &pitch, float &yaw, bool &is_fire, float &aim_x, float &aim_y,
    float &aim_z, const auto_aim_interfaces::msg::Target::SharedPtr &msg) {

  float time_delay = bias_time_ + fly_time_ + ;
  pre_x_center_ = msg->position.x + msg->velocity.x * time_delay;
  pre_y_center_ = msg->position.y + msg->velocity.y * time_delay;
  pre_z_center_ = msg->position.z;
}
// 择板，判断此时发弹是否有合适的目标
void SolveTrajectory::fireLogicDefault(
    float &pitch, float &yaw, bool &is_fire, float &aim_x, float &aim_y,
    float &aim_z, const auto_aim_interfaces::msg::Target::SharedPtr &msg) {

  float time_delay = bias_time_ + fly_time_;
  predictArmorPosition(msg, time_delay);

  if (last_selected_idx_ == LOST) {
    int idx = selectArmor(msg);
    float toyaw =
        solveYaw(pre_position_[idx].x, pre_position_[idx].y) - msg->cam_yaw;
    predictArmorPosition(msg, time_delay + toyaw / (0.58f + msg->v_yaw));
    int selected_idx = selectArmor(msg);
    updateSolveState(selected_idx, pitch, yaw, is_fire, aim_x, aim_y, aim_z,
                     msg);
  } else {
    int selected_idx = selectArmor(msg);
    if (selected_idx == last_selected_idx_) {
      updateSolveState(selected_idx, pitch, yaw, is_fire, aim_x, aim_y, aim_z,
                       msg);
    } else if (selected_idx != last_selected_idx_) {
      yaw = solveYaw(pre_position_[selected_idx].y,
                     pre_position_[selected_idx].x);
      last_yaw_ = solveYaw(pre_position_[last_selected_idx_].y,
                           pre_position_[last_selected_idx_].x);
      if (fabsf(yaw - last_yaw_) < 0.08) {
        fire_logic_mode_ = FireLogicMode::SPIN;
        selected_idx = CENTER;
        updateSolveState(selected_idx, pitch, yaw, is_fire, aim_x, aim_y, aim_z,
                         msg);
      }
    }
  }
}
void SolveTrajectory::updateSolveState(
    int &selected_idx, float &pitch, float &yaw, bool &is_fire, float &aim_x,
    float &aim_y, float &aim_z,
    const auto_aim_interfaces::msg::Target::SharedPtr &msg) {
  if (selected_idx == CENTER) {
    aim_x = pre_x_center_;
    aim_y = pre_y_center_;
    aim_z = pre_z_center_;
  } else {
    aim_x = pre_position_[selected_idx].x;
    aim_y = pre_position_[selected_idx].y;
    aim_z = pre_position_[selected_idx].z;
  }
  last_x_v_ = msg->velocity.x;
  last_y_v_ = msg->velocity.y;
  pitch = solvePitch(aim_x, aim_y, aim_z);
  yaw = solveYaw(aim_x, aim_y);
  is_fire = canFire(yaw, 0.05f, msg);
  if (selected_idx != LOST || is_fire) {
    last_yaw_ = yaw;
    last_selected_idx_ = selected_idx;
  }
}

// void SolveTrajectory::updateAimingState(
//     bool is_change, const auto_aim_interfaces::msg::Target::SharedPtr msg)
//     {
//   if (current_state_ == AimingState::TURNING) {
//     turning();
//     if (is_fire) {
//       current_state_ = AimingState::AIMING;
//     }
//     return;

//   } else if (current_state_ == AimingState::AIMING) {
//   }
//   //
//   这里假定当同时跳变时is_jump和is_change的方向总相同，此时事实上追踪目标未改变
//   else if (current_state_ == AimingState::TURNING &&
//            msg->is_jump == is_change) {
//     turning_time_count_ += msg->dt;
//     if (turning_time_count_ >= turning_time_) {
//       current_state_ = AimingState::AIMING;
//       turning_time_count_ = 0.0f;
//     }
//   }

//   //
//   若装甲板跳变而选择未改变则事实上追踪目标为上一装甲板；若装甲板未跳变而选择改变则事实上追踪目标为下一装甲板
//   else if (msg->is_jump != is_change) {
//     current_state_ = AimingState::TURNING;
//     turning_time_ = Calculateturningtime() - msg->dt > 0
//                         ? Calculateturningtime() - msg->dt
//                         : 0;
//     turning_time_count_ = 0.0f;
//   }
// }

// // 一旦msg为空则说明目标丢失,tracker中已做temp处理，这里快速响应即可
// else {
//   current_state_ = AimingState::LOST;
// }

void SolveTrajectory::autoSolveTrajectory(
    float &pitch, float &yaw, bool &is_fire, float &aim_x, float &aim_y,
    float &aim_z, const auto_aim_interfaces::msg::Target::SharedPtr msg) {
  if (!msg) {
    RCLCPP_ERROR(logger_, "Invalid target message");
    return;
  }
  fireLogicDefault(pitch, yaw, is_fire, aim_x, aim_y, aim_z, msg);
  // RCLCPP_DEBUG(logger_, "Auto solving trajectory for target");

  // switch (fire_logic_mode_) {
  // case FireLogicMode::TOP:
  //   fireLogicIsTop(pitch, yaw, aim_x, aim_y, aim_z, msg);
  //   break;
  // case FireLogicMode::DEFAULT:
  //   fireLogicDefault(pitch, yaw, aim_x, aim_y, aim_z, msg);
  //   break;
  // default:
  //   RCLCPP_WARN(logger_, "Unknown fire logic mode, using default");
  //   fireLogicDefault(pitch, yaw, aim_x, aim_y, aim_z, msg);
  //   break;
  // }

  // RCLCPP_DEBUG(logger_,
  //              "Final - pitch: %.3f, yaw: %.3f, aim: (%.3f, %.3f, %.3f)",
  //              pitch, yaw, aim_x, aim_y, aim_z);
}
} // namespace rm_auto_aim
// 没有LOST，预瞄考虑装甲板的位置变化，使用这一时刻与下一时刻的yaw变换计算