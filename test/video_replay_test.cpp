// 用录制好的视频回放 hero 的自瞄流水线（YOLO 检测 -> 跟踪 -> 瞄准 -> 开火判定），
// 不依赖真实相机/云台串口硬件。四元数用单位阵、弹速用配置里的默认值模拟。
// 仅用于本地回归验证，不接入实机控制回路。
#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? |                    | 输出命令行参数说明}"
  "{config-path c  | configs/hero.yaml  | yaml配置文件的路径}"
  "{video v        |                    | 待回放的视频文件路径（必填）}"
  "{out o          |                    | 标注后输出视频路径（可选）}"
  "{snapshot-dir s |                    | 定期保存标注帧截图的目录（可选）}"
  "{snapshot-interval i | 150            | 每隔多少帧保存一张截图}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser parser(argc, argv, keys);
  if (parser.has("help") || !parser.has("video")) {
    parser.printMessage();
    return parser.has("help") ? 0 : -1;
  }

  auto config_path = parser.get<std::string>("config-path");
  auto video_path = parser.get<std::string>("video");
  auto out_path = parser.has("out") ? parser.get<std::string>("out") : std::string();
  auto snapshot_dir = parser.has("snapshot-dir") ? parser.get<std::string>("snapshot-dir") : std::string();
  auto snapshot_interval = parser.get<int>("snapshot-interval");
  if (snapshot_interval <= 0) snapshot_interval = 150;

  auto yaml = tools::load(config_path);
  double mock_bullet_speed = yaml["bullet_speed"] ? yaml["bullet_speed"].as<double>() : 15.0;

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  // Planner 才是 hero.cpp 里真正发给云台的指令来源（plan_thread），
  // Aimer/Shooter 只驱动调试画面，两条链路分开验证。
  auto_aim::Planner planner(config_path);

  // 没有真实 IMU，用单位四元数（云台系=世界系）模拟，只用来跑通流水线，
  // reprojection/瞄准角度的绝对数值仅供参考，不代表真实拍摄姿态。
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  solver.set_R_gimbal2world(q);
  Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    tools::logger()->error("[VideoReplay] Failed to open video: {}", video_path);
    return -1;
  }
  double fps = cap.get(cv::CAP_PROP_FPS);
  if (fps <= 1.0) fps = 30.0;  // 部分 MJPEG 容器读不到 fps，兜底按 30 处理

  cv::VideoWriter writer;
  const auto t0 = std::chrono::steady_clock::now();

  int frame_count = 0;
  int frames_with_armor = 0;
  int frames_tracking = 0;
  int frames_valid_aim = 0;
  int frames_fire = 0;
  int frames_planner_control = 0;
  int frames_planner_fire = 0;

  cv::Mat img;
  while (cap.read(img)) {
    if (img.empty()) break;

    // 用"回放时间戳"而非真实 wall clock，让 EKF/预测里的 dt 符合视频本身的帧率，
    // 不会因为纯 CPU/GPU 推理比 30fps 快很多而把 dt 算得过小。
    auto t = t0 + std::chrono::microseconds(
                    static_cast<int64_t>(frame_count / fps * 1e6));

    auto armors = yolo.detect(img, frame_count);
    if (!armors.empty()) frames_with_armor++;

    cv::Mat draw_img = img.clone();
    for (const auto & armor : armors) {
      tools::draw_text(
        draw_img,
        fmt::format(
          "ID:{},conf{:.2f},type:{}", auto_aim::ARMOR_NAMES[armor.name], armor.confidence,
          auto_aim::ARMOR_TYPES[armor.type]),
        armor.center);
    }

    auto targets = tracker.track(armors, t);
    bool shooter_fire = false;

    if (!targets.empty()) {
      frames_tracking++;
      auto target = targets.front();
      tools::draw_text(draw_img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(draw_img, image_points, {0, 255, 255});
      }

      auto command = aimer.aim(targets, t, mock_bullet_speed, q);
      shooter_fire = shooter.shoot(command, aimer, targets, gimbal_pos, mock_bullet_speed);
      if (shooter_fire) frames_fire++;

      auto aim_point = aimer.debug_aim_point;
      if (aim_point.valid) {
        frames_valid_aim++;
        auto image_points = solver.reproject_armor(
          aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
        tools::draw_points(
          draw_img, image_points, shooter_fire ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0));
      }

      // 真正会发给云台的指令：和 hero.cpp 的 plan_thread 一样直接调用
      // planner.plan(target, bullet_speed)，不经过 Aimer/Shooter。
      auto plan = planner.plan(target, mock_bullet_speed);
      if (plan.control) {
        frames_planner_control++;
        if (plan.fire) frames_planner_fire++;
        auto plan_points = solver.reproject_armor(
          planner.debug_xyza.head(3), planner.debug_xyza[3], target.armor_type, target.name);
        tools::draw_points(
          draw_img, plan_points, plan.fire ? cv::Scalar(0, 128, 255) : cv::Scalar(255, 255, 0));
      }
    }

    if (!armors.empty()) {
      solver.draw_armor_info(draw_img, armors.front());
    }

    if (!out_path.empty()) {
      if (!writer.isOpened()) {
        writer.open(
          out_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
          {draw_img.cols, draw_img.rows});
      }
      writer.write(draw_img);
    }

    if (!snapshot_dir.empty() && frame_count % snapshot_interval == 0) {
      cv::imwrite(fmt::format("{}/frame_{:06d}.jpg", snapshot_dir, frame_count), draw_img);
    }

    frame_count++;
    if (frame_count % 300 == 0) {
      tools::logger()->info(
        "[VideoReplay] {} frames processed (armor {:.1f}%, tracking {:.1f}%, valid_aim {:.1f}%, "
        "fire {:.1f}%, planner_control {:.1f}%, planner_fire {:.1f}%)",
        frame_count, 100.0 * frames_with_armor / frame_count, 100.0 * frames_tracking / frame_count,
        100.0 * frames_valid_aim / frame_count, 100.0 * frames_fire / frame_count,
        100.0 * frames_planner_control / frame_count, 100.0 * frames_planner_fire / frame_count);
    }
  }

  tools::logger()->info(
    "[VideoReplay] DONE video={} total_frames={} frames_with_armor={} frames_tracking={} "
    "frames_valid_aim={} frames_fire={} frames_planner_control={} frames_planner_fire={}",
    video_path, frame_count, frames_with_armor, frames_tracking, frames_valid_aim, frames_fire,
    frames_planner_control, frames_planner_fire);

  return 0;
}
