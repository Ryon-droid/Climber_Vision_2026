#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <deque>
#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos,
    double bullet_speed);

  // 获取滑动窗口平均弹速
  double get_average_bullet_speed() const;

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;

  // 小陀螺模式参数
  double decision_spin_;
  double fire_time_min_ratio_;
  double fire_time_max_ratio_;

  // 退出小陀螺模式参数（连续N帧低于阈值才退出）
  double exit_spinning_threshold_;
  int exit_frame_threshold_;
  int exit_frame_count_;
  bool was_spinning_;

  // 弹速滑动窗口
  std::deque<double> bullet_speed_window_;
  static constexpr int WINDOW_SIZE = 3;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP