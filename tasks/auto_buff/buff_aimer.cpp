#include "buff_aimer.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_buff
{
// 保留构造函数：仅依赖YAML和头文件中未注释的成员变量（predict_time_等），无被屏蔽头文件依赖
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  // 以下为非识别相关：机械偏移（瞄准补偿）、射击间隔（控制层），注释保留结构
  // yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  // pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  // fire_gap_time_ = yaml["fire_gap_time"].as<double>();
  // 以下为识别相关：目标预测时间（基于识别结果的未来位置推算）
  predict_time_ = yaml["predict_time"].as<double>();

  // 非识别相关：射击时间戳（控制层），注释保留结构
  // last_fire_t_ = std::chrono::steady_clock::now();
}

// 注释：aim函数声明已在头文件中屏蔽（依赖Target类，来自buff_target.hpp），故屏蔽实现
// io::Command Aimer::aim(
//   auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp,
//   double bullet_speed, bool to_now)
// {
//   io::Command command = {false, false, 0, 0};
//   // -------------------------- 识别相关核心逻辑 --------------------------
//   // 1. 判断目标是否可解（识别模块输出的核心状态：是否成功识别并追踪到目标）
//   if (target.is_unsolve()) {
//     tools::logger()->debug("[Aimer] 目标不可解（识别失败或追踪发散）");
//     return command;
//   }

//   // 2. 计算识别延迟：从“图像采集时间戳”到“当前处理时间”的间隔（识别数据的时效性判断）
//   auto now = std::chrono::steady_clock::now();
//   auto detect_now_gap = tools::delta_time(now, timestamp);
//   // 3. 确定目标预测时间：基于识别延迟补偿，推算目标未来位置的时间基准（识别结果的后续应用）
//   auto future = to_now ? (detect_now_gap + predict_time_) : predict_time_;
//   double yaw, pitch;

//   // 4. 调用角度计算函数（内部包含基于识别结果的目标预测，核心识别-瞄准衔接）
//   // -------------------------- 识别相关核心逻辑结束 --------------------------

//   // 以下为非识别相关：瞄准角度计算、云台控制、射击决策（控制/瞄准层），注释保留调用结构
//   if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
//     // 非识别：赋值瞄准角度（控制层）
//     // command.yaw = yaw;
//     // command.pitch = -pitch;  //世界坐标系下的pitch向上为负
    
//     // 非识别：角度变化判断（云台稳定性控制）
//     // bool angle_changed = std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3;
    
//     // 非识别：扇叶切换与控制权限判断（控制层）
//     // if (mistake_count_ > 3) {
//     //   switch_fanblade_ = true;
//     //   mistake_count_ = 0;
//     //   command.control = true;
//     // } else if (angle_changed) {
//     //   switch_fanblade_ = true;
//     //   mistake_count_++;
//     //   command.control = false;
//     // } else {
//     //   switch_fanblade_ = false;
//     //   mistake_count_ = 0;
//     //   command.control = true;
//     // }
//     // last_yaw_ = yaw;
//     // last_pitch_ = pitch;
//   }

//   // 非识别：射击决策（控制层）
//   // if (switch_fanblade_) {
//   //   command.shoot = false;
//   //   last_fire_t_ = now;
//   // } else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
//   //   command.shoot = true;
//   //   last_fire_t_ = now;
//   // }

//   // return command;
// // }

// 注释：get_send_angle函数声明已在头文件中屏蔽（依赖Target类，来自buff_target.hpp），故屏蔽实现
// bool Aimer::get_send_angle(
//   auto_buff::Target & target, const double predict_time, const double bullet_speed,
//   const bool to_now, double & yaw, double & pitch)
// {
//   // -------------------------- 识别相关核心逻辑 --------------------------
//   // 1. 基于识别到的目标状态，预测未来位置（识别结果的动态推算）
//   // （predict依赖Target模块的EKF，而EKF输入是识别到的扇叶数据）
//   target.predict(predict_time);
//   // 2. 获取识别追踪的核心状态：扇叶滚转角（验证识别结果的有效性）
//   double fanblade_roll = target.ekf_x()[5];
//   tools::logger()->debug("[Aimer] 识别到的扇叶滚转角：{:.2f}°", fanblade_roll * 180 / CV_PI);

//   // 3. 基于识别的目标坐标系，获取瞄准点世界坐标（识别数据的空间转换）
//   // （point_buff2world依赖Target模块的识别校准参数，输入是识别到的Buff中心）
//   Eigen::Vector3d aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
//   tools::logger()->debug(
//     "[Aimer] 识别推算的瞄准点坐标（x,y,z）：{:.2f}, {:.2f}, {:.2f}",
//     aim_in_world[0], aim_in_world[1], aim_in_world[2]
//   );
//   // -------------------------- 识别相关核心逻辑结束 --------------------------

//   // 以下为非识别相关：弹道解算、角度计算（瞄准层），注释保留结构
//   // double d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
//   // double h = aim_in_world[2];

//   // // 非识别：第一次弹道解算（瞄准层）
//   // tools::Trajectory trajectory0(bullet_speed, d, h);
//   // if (trajectory0.unsolvable) {  // 如果弹道无法解算，返回未命中结果
//   //   tools::logger()->debug(
//   //     "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
//   //   return false;
//   // }

//   // // 非识别：基于弹道时间二次预测（瞄准层的动态补偿）
//   // target.predict(trajectory0.fly_time);
//   // fanblade_roll = target.ekf_x()[5];
//   // aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
//   // d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
//   // h = aim_in_world[2];

//   // // 非识别：第二次弹道解算（瞄准层）
//   // tools::Trajectory trajectory1(bullet_speed, d, h);
//   // if (trajectory1.unsolvable) {  // 如果弹道无法解算，返回未命中结果
//   //   tools::logger()->debug(
//   //     "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
//   //   return false;
//   // }

//   // // 非识别：验证弹道时间稳定性（瞄准层）
//   // auto time_error = trajectory1.fly_time - trajectory0.fly_time;
//   // if (std::abs(time_error) > 0.01) {  // 如果时间误差过大，返回未命中结果
//   //   tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
//   //   return false;
//   // }

//   // // 非识别：计算最终瞄准角度（瞄准层，叠加机械偏移）
//   // yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
//   // pitch = trajectory1.pitch + pitch_offset_;

//   // 仅保留识别相关的返回逻辑（此处返回true表示识别流程正常，非瞄准成功）
//   // return true;
// // }

}  // namespace auto_buff
