#include "shooter.hpp"

#include <numeric>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  decision_spin_ = yaml["decision_spin"].as<double>();
  fire_time_min_ratio_ = yaml["fire_time_min_ratio"].as<double>();
  fire_time_max_ratio_ = yaml["fire_time_max_ratio"].as<double>();
  exit_spinning_threshold_ = yaml["exit_spinning_threshold"].as<double>();
  exit_frame_threshold_ = yaml["exit_frame_threshold"].as<int>();
  exit_frame_count_ = 0;
  was_spinning_ = false;
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos,
  double bullet_speed)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  const auto& target = targets.front();
  double angular_vel = std::abs(target.ekf_x()[7]);

  // 滞回状态判定：进入立即进入，退出需要连续N帧低于阈值
  if (was_spinning_) {
    // 已经在小陀螺模式中
    if (angular_vel < exit_spinning_threshold_) {
      exit_frame_count_++;
      if (exit_frame_count_ >= exit_frame_threshold_) {
        was_spinning_ = false;
        exit_frame_count_ = 0;
      }
    } else {
      // 恢复到阈值之上，重置计数
      exit_frame_count_ = 0;
    }
  } else {
    // 不在小陀螺模式中，检查是否进入
    if (angular_vel > decision_spin_) {
      was_spinning_ = true;
      exit_frame_count_ = 0;
    }
  }

  if (was_spinning_) {
    // ===== 小陀螺模式：时序判定 =====
    // 获取飞行时间
    double fly_time = aimer.GetFlyTime();
    if (fly_time <= 0) {
      last_command_ = command;
      return false;
    }

    // 获取车辆中心 yaw
    double center_yaw = std::atan2(target.ekf_x()[2], target.ekf_x()[0]);

    // 获取装甲板列表，计算 coming 方向装甲板的 delta_angle
    std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
    if (armor_xyza_list.empty()) {
      last_command_ = command;
      return false;
    }

    // 找到 coming 方向的装甲板（角速度方向与 delta_angle 符号相反）
    double ekf_w = target.ekf_x()[7];
    double best_delta_angle = 1e10;
    for (const auto& armor_xyza : armor_xyza_list) {
      double armor_yaw = armor_xyza[3];
      double delta_angle = tools::limit_rad(armor_yaw - center_yaw);

      // coming 方向：角速度 > 0 时，delta_angle < 0；角速度 < 0 时，delta_angle > 0
      bool is_coming = (ekf_w > 0 && delta_angle < 0) || (ekf_w < 0 && delta_angle > 0);
      if (is_coming) {
        // 取绝对值最小的（最接近前方的）
        if (std::abs(delta_angle) < std::abs(best_delta_angle)) {
          best_delta_angle = delta_angle;
        }
      }
    }

    if (std::abs(best_delta_angle) >= 1e9) {
      // 没找到 coming 装甲板
      last_command_ = command;
      return false;
    }

    // 计算装甲板到达"正前方"需要的时间
    // delta_angle 为负时，需要转到 0（绝对值时间）
    // delta_angle 为正时，需要转到 0
    double time_to_front = std::abs(best_delta_angle / ekf_w);

    // 计算 ratio
    double ratio = time_to_front / fly_time;

    // 判定时机
    if (ratio >= fire_time_min_ratio_ && ratio <= fire_time_max_ratio_) {
      if (bullet_speed_window_.empty() || bullet_speed != bullet_speed_window_.back()) {
        bullet_speed_window_.push_back(bullet_speed);
        if (bullet_speed_window_.size() > WINDOW_SIZE) {
          bullet_speed_window_.pop_front();
        }
      }
      last_command_ = command;
      return true;
    }

  } else {
    // ===== 非小陀螺模式：原有容差判定 =====
    auto target_x = target.ekf_x()[0];
    auto target_y = target.ekf_x()[2];
    auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                       ? second_tolerance_
                       : first_tolerance_;

    if (std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&
        std::abs(gimbal_pos[0] - command.yaw) < tolerance &&
        aimer.debug_aim_point.valid) {
      if (bullet_speed_window_.empty() || bullet_speed != bullet_speed_window_.back()) {
        bullet_speed_window_.push_back(bullet_speed);
        if (bullet_speed_window_.size() > WINDOW_SIZE) {
          bullet_speed_window_.pop_front();
        }
      }
      last_command_ = command;
      return true;
    }
  }

  last_command_ = command;
  return false;
}

double Shooter::get_average_bullet_speed() const
{
  if (bullet_speed_window_.empty()) {
    return 0.0;
  }
  double sum = std::accumulate(bullet_speed_window_.begin(), bullet_speed_window_.end(), 0.0);
  return sum / bullet_speed_window_.size();
}

}  // namespace auto_aim