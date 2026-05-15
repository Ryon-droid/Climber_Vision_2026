#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <limits>
#include <numeric>
#include <tuple>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
const cv::Point2f IMG_CENTER(1440 / 2, 1080 / 2);

double center_distance(const Armor & armor) { return cv::norm(armor.center - IMG_CENTER); }
}  // namespace

Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth},
  target_switch_strategy_{TargetSwitchStrategy::center},
  target_switch_confirm_frames_{5},
  target_switch_distance_ratio_{0.8},
  switch_candidate_name_{ArmorName::not_armor},
  switch_candidate_type_{ArmorType::small},
  switch_candidate_count_{0},
  robot_type_{"standard"},
  priority_mode_{1}
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
  if (yaml["robot_type"]) {
    robot_type_ = yaml["robot_type"].as<std::string>();
  }
  if (yaml["priority_mode"]) {
    priority_mode_ = yaml["priority_mode"].as<int>();
  }

  const auto switch_strategy =
    yaml["target_switch_strategy"] ? yaml["target_switch_strategy"].as<std::string>() : "center";
  if (switch_strategy == "center") {
    target_switch_strategy_ = TargetSwitchStrategy::center;
  } else if (switch_strategy == "distance") {
    target_switch_strategy_ = TargetSwitchStrategy::distance;
  } else if (switch_strategy == "priority") {
    target_switch_strategy_ = TargetSwitchStrategy::priority;
  } else if (switch_strategy == "disabled") {
    target_switch_strategy_ = TargetSwitchStrategy::disabled;
  } else {
    tools::logger()->warn(
      "[Tracker] Unknown target_switch_strategy '{}', fallback to center", switch_strategy);
  }

  if (yaml["target_switch_confirm_frames"]) {
    target_switch_confirm_frames_ = std::max(1, yaml["target_switch_confirm_frames"].as<int>());
  }
  if (yaml["target_switch_distance_ratio"]) {
    target_switch_distance_ratio_ = yaml["target_switch_distance_ratio"].as<double>();
  }
}

std::string Tracker::state() const { return state_; }

/**
 * @brief 执行每一帧的核心追踪逻辑
 * @param armors 视觉检测模块传入的视野内所见装甲板集合
 * @param t 当前处理这帧画面的时刻，计算卡尔曼运动推演 dt 的关键参数
 * @param use_enemy_color 是否仅追踪敌方颜色的装甲板
 * @return 跟踪的目标列表。实际目前对于单自瞄任务，一旦锁定某个目标，返回值为含有一个该 Target 对象；如果没有任何锁定目标则返回空。
 */
std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  (void)use_enemy_color;

  const auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 1. 如果帧间隔过长（比如画面卡顿超过 0.1s），为防止卡尔曼预测的错误外推飞得太离谱，主动放弃当前目标的跟踪进入 lost（丢失）状态。
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 2. 根据己方配置自动过滤掉我们不关心的己方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });
  set_priority(armors);

  // 3. 对识别到的装甲板按到图像中心的欧氏距离进行升序排序，使优先锁定靠近准星或视图中心的装甲板。
  armors.sort([](const Armor & a, const Armor & b) { return center_distance(a) < center_distance(b); });

  // 4. 按装甲板的语义或兵种优先级排序（例如：英雄 > 步兵 > 哨兵等预设逻辑）。保持稳定的靶标选择。
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found = false;
  // 5. 根据Tracker目前所处的状态机状态决定采取何种策略。
  if (try_switch_target(armors, t)) {
    found = true;
  } else if (state_ == "lost") {
    // 丢失状态下重新建图并捕获新目标，一旦设好 found = true
    found = set_target(armors, t);
  } else {
    // 若当前持有一个跟踪目标，尝试用新一帧数据更新（收敛）该目标
    found = update_target(armors, t);
  }

  // 6. 根据 found 判定结果和丢失帧数等阈值，流转状态机（检测中 <=> 追踪中 <=> 暂丢 <=> 丢失）。
  state_machine(found);

  // 7. 如果在跟踪期间发现 EKF 发散严重（比如推算的半径变为负数或者无穷大失真），强制舍弃当前目标。
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 8. EKF NIS (Normalized Innovation Squared) 诊断卡尔曼滤波收敛退化，失败过拟合占比超过40%时抛弃重建目标
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  return {target_};
}

/**
 * @brief 更新追踪器状态机
 * @param found 是否在当前帧中成功找到或更新目标
 */
void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;
    state_ = "detecting";
    detect_count_ = 1;
    return;
  }

  if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
    return;
  }

  if (state_ == "tracking") {
    if (found) return;
    temp_lost_count_ = 1;
    state_ = "temp_lost";
    return;
  }

  if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
    return;
  }

  if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost) {
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      } else {
        max_temp_lost_count_ = normal_temp_lost_count_;
      }
      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  return set_target(armor, t);
}

bool Tracker::set_target(Armor & armor, std::chrono::steady_clock::time_point t)
{
  solver_.solve(armor);

  const auto is_balance = (armor.type == ArmorType::big) &&
                          (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                           armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  } else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 1, 100, 1, 0.5, 0.5}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  } else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  } else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

bool Tracker::try_switch_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (
    state_ == "lost" || armors.empty() ||
    target_switch_strategy_ == TargetSwitchStrategy::disabled) {
    reset_switch_candidate();
    return false;
  }

  std::optional<Armor> candidate;
  if (!select_switch_candidate(armors, candidate)) {
    reset_switch_candidate();
    return false;
  }

  if (candidate->name == switch_candidate_name_ && candidate->type == switch_candidate_type_) {
    switch_candidate_count_++;
  } else {
    switch_candidate_name_ = candidate->name;
    switch_candidate_type_ = candidate->type;
    switch_candidate_count_ = 1;
  }

  if (switch_candidate_count_ < target_switch_confirm_frames_) return false;

  const auto old_name = target_.name;
  const auto old_type = target_.armor_type;
  if (!set_target(*candidate, t)) {
    reset_switch_candidate();
    return false;
  }

  state_ = "lost";
  detect_count_ = 0;
  temp_lost_count_ = 0;
  reset_switch_candidate();

  tools::logger()->info(
    "[Tracker] Switch target {}({}) -> {}({})", ARMOR_NAMES[old_name], ARMOR_TYPES[old_type],
    ARMOR_NAMES[target_.name], ARMOR_TYPES[target_.armor_type]);
  return true;
}

bool Tracker::select_switch_candidate(std::list<Armor> & armors, std::optional<Armor> & candidate)
{
  if (target_switch_strategy_ == TargetSwitchStrategy::center) {
    return select_center_candidate(armors, candidate);
  }
  if (target_switch_strategy_ == TargetSwitchStrategy::distance) {
    return select_distance_candidate(armors, candidate);
  }
  if (target_switch_strategy_ == TargetSwitchStrategy::priority) {
    return select_priority_candidate(armors, candidate);
  }
  return false;
}

bool Tracker::select_center_candidate(const std::list<Armor> & armors, std::optional<Armor> & candidate)
  const
{
  double current_distance = std::numeric_limits<double>::max();
  double candidate_distance = std::numeric_limits<double>::max();
  bool found_current = false;
  bool found_candidate = false;

  for (const auto & armor : armors) {
    const auto distance = center_distance(armor);
    if (is_current_target(armor)) {
      found_current = true;
      current_distance = std::min(current_distance, distance);
      continue;
    }

    if (distance < candidate_distance) {
      candidate = armor;
      candidate_distance = distance;
      found_candidate = true;
    }
  }

  return found_current && found_candidate &&
         candidate_distance < current_distance * target_switch_distance_ratio_;
}

bool Tracker::select_distance_candidate(std::list<Armor> & armors, std::optional<Armor> & candidate)
{
  double current_distance = std::numeric_limits<double>::max();
  double candidate_distance = std::numeric_limits<double>::max();
  bool found_current = false;
  bool found_candidate = false;

  for (auto & armor : armors) {
    auto solved_armor = armor;
    solver_.solve(solved_armor);
    const auto distance = solved_armor.xyz_in_gimbal.norm();

    if (is_current_target(solved_armor)) {
      found_current = true;
      current_distance = std::min(current_distance, distance);
      continue;
    }

    if (distance < candidate_distance) {
      candidate = solved_armor;
      candidate_distance = distance;
      found_candidate = true;
    }
  }

  return found_current && found_candidate &&
         candidate_distance < current_distance * target_switch_distance_ratio_;
}

bool Tracker::select_priority_candidate(const std::list<Armor> & armors, std::optional<Armor> & candidate)
  const
{
  double candidate_distance = std::numeric_limits<double>::max();
  bool found_candidate = false;

  for (const auto & armor : armors) {
    if (is_current_target(armor) || !(armor.priority < target_.priority)) continue;

    const auto distance = center_distance(armor);
    if (
      !found_candidate || armor.priority < candidate->priority ||
      (armor.priority == candidate->priority && distance < candidate_distance)) {
      candidate = armor;
      candidate_distance = distance;
      found_candidate = true;
    }
  }

  return found_candidate;
}

bool Tracker::is_current_target(const Armor & armor) const
{
  return armor.name == target_.name && armor.type == target_.armor_type;
}

void Tracker::reset_switch_candidate()
{
  switch_candidate_name_ = ArmorName::not_armor;
  switch_candidate_type_ = ArmorType::small;
  switch_candidate_count_ = 0;
}

void Tracker::set_priority(std::list<Armor> & armors) const
{
  for (auto & armor : armors) {
    armor.priority = get_priority(armor.name);
  }
}

ArmorPriority Tracker::get_priority(ArmorName name) const
{
  if (priority_mode_ == 2) {
    switch (name) {
      case ArmorName::two:
        return ArmorPriority::first;
      case ArmorName::one:
        return ArmorPriority::second;
      case ArmorName::three:
      case ArmorName::four:
      case ArmorName::five:
        return ArmorPriority::second;
      case ArmorName::sentry:
      case ArmorName::outpost:
      case ArmorName::base:
      case ArmorName::not_armor:
      default:
        return ArmorPriority::third;
    }
  }

  if (robot_type_ == "hero") {
    switch (name) {
      case ArmorName::three:
      case ArmorName::four:
        return ArmorPriority::first;
      case ArmorName::one:
        return ArmorPriority::second;
      case ArmorName::five:
      case ArmorName::sentry:
        return ArmorPriority::third;
      case ArmorName::two:
        return ArmorPriority::forth;
      case ArmorName::outpost:
      case ArmorName::base:
      case ArmorName::not_armor:
      default:
        return ArmorPriority::fifth;
    }
  }

  switch (name) {
    case ArmorName::one:
      return ArmorPriority::second;
    case ArmorName::two:
      return ArmorPriority::forth;
    case ArmorName::three:
    case ArmorName::four:
      return ArmorPriority::first;
    case ArmorName::five:
    case ArmorName::sentry:
      return ArmorPriority::third;
    case ArmorName::outpost:
    case ArmorName::base:
    case ArmorName::not_armor:
    default:
      return ArmorPriority::fifth;
  }
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  std::vector<Armor> matched_armors;
  matched_armors.reserve(3);

  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    solver_.solve(armor);
    matched_armors.push_back(armor);
    if (target_.name == ArmorName::outpost && matched_armors.size() >= 3) break;
  }

  if (matched_armors.empty()) return false;

  if (target_.name == ArmorName::outpost) {
    target_.update_outpost(matched_armors);
    auto iter = target_.ekf().data.find("outpost_assignment_valid");
    if (iter != target_.ekf().data.end() && iter->second < 0.5) return false;
    return true;
  }

  for (const auto & armor : matched_armors) {
    target_.update(armor);
  }
  return true;
}

}  // namespace auto_aim
