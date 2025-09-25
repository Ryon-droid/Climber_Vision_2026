#ifndef AUTO_BUFF__AIMER_HPP
#define AUTO_BUFF__AIMER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>


// #include "../auto_aim/planner/planner.hpp"
// #include "buff_target.hpp"
#include "buff_type.hpp"
#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"

namespace auto_buff
{
class Aimer
{
public:
  Aimer(const std::string & config_path);

  // 注释：aim 函数依赖 Target 类（来自 buff_target.hpp），暂屏蔽
  // io::Command aim(
  //   Target & target, std::chrono::steady_clock::time_point & timestamp, double bullet_speed,
  //   bool to_now = true);

  // 注释：mpc_aim 函数依赖 auto_aim::Plan（来自 planner.hpp）和 Target 类，暂屏蔽
  // auto_aim::Plan mpc_aim(
  //   Target & target, std::chrono::steady_clock::time_point & timestamp, io::GimbalState gs,
  //   bool to_now = true);

  double angle;      /// 暂留变量（原依赖 Target 状态，当前无实际意义）
  double t_gap = 0;  /// 暂留变量（原依赖时间差计算，当前无实际意义）

private:
  // 注释：SmallTarget 类来自 buff_target.hpp，相关成员暂屏蔽
  // SmallTarget target_;
  double yaw_offset_;
  double pitch_offset_;

  double fire_gap_time_;
  double predict_time_;

  int mistake_count_ = 0;
  bool switch_fanblade_;

  double last_yaw_ = 0;
  double last_pitch_ = 0;

  // 注释：mpc 相关变量（依赖 planner.hpp），暂屏蔽
  // bool first_in_aimer_ = true;

  std::chrono::steady_clock::time_point last_fire_t_;

  // 注释：get_send_angle 函数依赖 Target 类（来自 buff_target.hpp），暂屏蔽
  // bool get_send_angle(
  //   auto_buff::Target & target, const double predict_time, const double bullet_speed,
  //   const bool to_now, double & yaw, double & pitch);
};
}  // namespace auto_buff
#endif  // AUTO_AIM__AIMER_HPP
