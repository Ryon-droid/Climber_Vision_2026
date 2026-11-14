#include "buff_aimer.hpp"

#include "buff_target.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_buff
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  // 目标预测时间（基于识别结果的未来位置推算）
  predict_time_ = yaml["predict_time"].as<double>();

  // 射击时间戳
  last_fire_t_ = std::chrono::steady_clock::now();
}

io::Command Aimer::aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now)
{
  io::Command command = {false, false, 0, 0};
  // -------------------------- 识别相关核心逻辑 --------------------------
  // 1. 判断目标是否可解（识别模块输出的核心状态：是否成功识别并追踪到目标）
  if (target.is_unsolve()) {
    tools::logger()->debug("[Aimer] 目标不可解（识别失败或追踪发散）");
    return command;
  }

  // 2. 计算识别延迟：从"图像采集时间戳"到"当前处理时间"的间隔（识别数据的时效性判断）
  auto now = std::chrono::steady_clock::now();
  auto detect_now_gap = tools::delta_time(now, timestamp);
  // 3. 确定目标预测时间：基于识别延迟补偿，推算目标未来位置的时间基准（识别结果的后续应用）
  auto future = to_now ? (detect_now_gap + predict_time_) : predict_time_;
  double yaw, pitch;

  // 4. 调用角度计算函数（内部包含基于识别结果的目标预测，核心识别-瞄准衔接）
  // -------------------------- 识别相关核心逻辑结束 --------------------------

  // 瞄准角度计算、云台控制、射击决策（控制/瞄准层）
  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
    // 赋值瞄准角度（控制层）
    command.yaw = yaw;
    command.pitch = -pitch;  //世界坐标系下的pitch向上为负
    
    // 角度变化判断（云台稳定性控制）
    bool angle_changed = std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3;
    
    // 扇叶切换与控制权限判断（控制层）
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;
      mistake_count_ = 0;
      command.control = true;
    } else if (angle_changed) {
      switch_fanblade_ = true;
      mistake_count_++;
      command.control = false;
    } else {
      switch_fanblade_ = false;
      mistake_count_ = 0;
      command.control = true;
    }
    last_yaw_ = yaw;
    last_pitch_ = pitch;
  }

  // 非识别：射击决策（控制层）
  if (switch_fanblade_) {
    command.shoot = false;
    last_fire_t_ = now;
  } else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    command.shoot = true;
    last_fire_t_ = now;
  }

  return command;
}

bool Aimer::get_send_angle(
  auto_buff::Target & target, const double predict_time, const double bullet_speed,
  const bool to_now, double & yaw, double & pitch)
{
  // 检查目标是否可解
  if (target.is_unsolve()) {
    tools::logger()->debug("[Aimer] 目标不可解");
    return false;
  }

  // -------------------------- 识别相关核心逻辑 --------------------------
  // 1. 基于识别到的目标状态，预测未来位置（识别结果的动态推算）
  // （predict依赖Target模块的EKF，而EKF输入是识别到的扇叶数据）
  target.predict(predict_time);
  
  // 检查EKF状态是否有效
  Eigen::VectorXd ekf_state = target.ekf_x();
  for (int i = 0; i < ekf_state.size(); ++i) {
    if (std::isnan(ekf_state[i])) {
      tools::logger()->debug("[Aimer] EKF状态包含NaN值，索引: {}", i);
      return false;
    }
  }
  
  // 2. 获取识别追踪的核心状态：扇叶滚转角（验证识别结果的有效性）
  double fanblade_roll = target.ekf_x()[5];
  tools::logger()->debug("[Aimer] 识别到的扇叶滚转角：{:.2f}°", fanblade_roll * 180 / CV_PI);
  // 3. 基于识别的目标坐标系，获取瞄准点世界坐标（识别数据的空间转换）
  // （point_buff2world依赖Target模块的识别校准参数，输入是识别到的Buff中心）
  Eigen::Vector3d aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  
  // 检查瞄准点坐标是否有效
  if (std::isnan(aim_in_world[0]) || std::isnan(aim_in_world[1]) || std::isnan(aim_in_world[2])) {
    tools::logger()->debug("[Aimer] 瞄准点坐标包含NaN值");
    return false;
  }
  
  tools::logger()->debug(
    "[Aimer] 识别推算的瞄准点坐标（x,y,z）：{:.2f}, {:.2f}, {:.2f}",
    aim_in_world[0], aim_in_world[1], aim_in_world[2]
  );
  // -------------------------- 识别相关核心逻辑结束 --------------------------

  // 弹道解算、角度计算（瞄准层）
  double d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  double h = aim_in_world[2];
  
  // 检查距离和高度是否有效
  if (std::isnan(d) || std::isnan(h)) {
    tools::logger()->debug("[Aimer] 距离或高度为NaN");
    return false;
  }

  // 第一次弹道解算（瞄准层）
  tools::Trajectory trajectory0(bullet_speed, d, h);
  if (trajectory0.unsolvable) {  // 如果弹道无法解算，返回未命中结果
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    return false;
  }

  // 基于弹道时间二次预测（瞄准层的动态补偿）
  target.predict(trajectory0.fly_time);
  fanblade_roll = target.ekf_x()[5];
  aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  h = aim_in_world[2];

  // 第二次弹道解算（瞄准层）
  tools::Trajectory trajectory1(bullet_speed, d, h);
  if (trajectory1.unsolvable) {  // 如果弹道无法解算，返回未命中结果
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    return false;
  }

  // 验证弹道时间稳定性（瞄准层）
  auto time_error = trajectory1.fly_time - trajectory0.fly_time;
  if (std::abs(time_error) > 0.01) {  // 如果时间误差过大，返回未命中结果
    tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
    return false;
  }

  // 计算最终瞄准角度（瞄准层）
  yaw = std::atan2(aim_in_world[1], aim_in_world[0]);
  pitch = trajectory1.pitch;
  return true;
}
}  // namespace auto_buff