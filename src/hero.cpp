#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/lobshot/lobshot.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;
using auto_aim::Plan;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/hero.yaml      | yaml配置文件的路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser parser(argc, argv, keys);
  if (parser.has("help")) {
    parser.printMessage();
    return 0;
  }
  auto config_path = parser.get<std::string>(0);
  auto yaml = tools::load(config_path);
  const auto lobshot_config_path =
    (std::filesystem::path(config_path).parent_path() / "lobshot.yaml").string();

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  std::unique_ptr<io::Camera> aim_camera;
  io::Gimbal gimbal(config_path);
  lobshot::Lobshot lobshot_sender(lobshot_config_path);
  std::unique_ptr<io::Camera> lobshot_camera;

  auto use_tradition = tools::read<bool>(yaml, "use_traditional");
  const double fire_yaw_tolerance =
    (yaml["fire_yaw_tolerance"] ? yaml["fire_yaw_tolerance"].as<double>() : 1.0) / 57.3;
  const double fire_pitch_tolerance =
    (yaml["fire_pitch_tolerance"] ? yaml["fire_pitch_tolerance"].as<double>() : 1.0) / 57.3;
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Detector detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target_opt = target_queue.front();
      auto gs = gimbal.state();
      auto gimbal_mode = gimbal.mode();

      if (gimbal_mode == io::GimbalMode::LOBSHOT) {
        gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
        std::this_thread::sleep_for(10ms);
        continue;
      }

      Plan plan;
      if (target_opt.has_value()) {
        plan = planner.plan(target_opt.value(), gs.bullet_speed);
      } else {
        plan = Plan{};
      }

      const auto fire_yaw_err = std::abs(tools::limit_rad(plan.yaw - gs.yaw));
      const auto fire_pitch_err = std::abs(plan.pitch - gs.pitch);
      const bool fire_closed_loop =
        plan.control && plan.fire && fire_yaw_err < fire_yaw_tolerance &&
        fire_pitch_err < fire_pitch_tolerance;

      gimbal.send(
        plan.control, fire_closed_loop, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
        plan.pitch_vel,
        plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      data["bullet_speed"] = gs.bullet_speed;
      data["gimbal_yaw"] = tools::limit_rad(gs.yaw) * 57.3;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = tools::limit_rad(gs.pitch) * 57.3;
      data["gimbal_pitch_vel"] = gs.pitch_vel;
      data["predict_time"] = plan.predict_time * 1e3;
      data["target_yaw"] = plan.target_yaw * 57.3;
      data["target_pitch"] = plan.target_pitch * 57.3;
      data["plan_yaw"] = plan.yaw * 57.3;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_pitch"] = plan.pitch * 57.3;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["fire"] = fire_closed_loop ? 1 : 0;
      data["fire_request"] = plan.fire ? 1 : 0;
      data["fire_yaw_err"] = fire_yaw_err * 57.3;
      data["fire_pitch_err"] = fire_pitch_err * 57.3;
      data["fired"] = fired ? 1 : 0;

      if (target_opt.has_value()) {
        const auto x = target_opt->ekf_x();
        data["w"] = x[7];
        data["v"] = std::sqrt(x[1] * x[1] + x[3] * x[3]);
        data["distance"] = std::sqrt(x[0] * x[0] + x[2] * x[2] + x[4] * x[4]);
      } else {
        data["w"] = 0.0;
        data["v"] = 0.0;
        data["distance"] = 0.0;
      }

      plotter.plot(data);
      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto fps_window_start = std::chrono::steady_clock::now();
  int fps_frame_count = 0;
  auto report_fps = [&](io::GimbalMode mode) {
    fps_frame_count++;
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = tools::delta_time(now, fps_window_start);
    if (elapsed >= 1.0) {
      tools::logger()->info("[Hero] {} FPS: {:.2f}", gimbal.str(mode), fps_frame_count / elapsed);
      fps_window_start = now;
      fps_frame_count = 0;
    }
  };

  while (!exiter.exit()) {
    const auto gimbal_mode = gimbal.mode();
    if (gimbal_mode == io::GimbalMode::LOBSHOT) {
      if (aim_camera) {
        tools::logger()->info("[Hero] Releasing aim camera after entering lobshot mode.");
        aim_camera.reset();
      }
      if (!lobshot_camera) {
        tools::logger()->info("[Hero] Initializing lobshot camera on demand.");
        lobshot_camera = std::make_unique<io::Camera>(config_path, "lobshot");
      }
      lobshot_camera->read(img, t);
    } else {
      if (lobshot_camera) {
        tools::logger()->info("[Hero] Releasing lobshot camera after exiting lobshot mode.");
        lobshot_camera.reset();
      }
      if (!aim_camera) {
        tools::logger()->info("[Hero] Initializing aim camera on demand.");
        aim_camera = std::make_unique<io::Camera>(config_path, "aim");
      }
      aim_camera->read(img, t);
    }
    if (img.empty()) break;

    auto q = gimbal.q(t);
    recorder.record(img, q, t);

    if (gimbal_mode == io::GimbalMode::LOBSHOT) {
      lobshot_sender.set_enabled(true);
      lobshot_sender.process(img, t);
      target_queue.push(std::nullopt);

      auto draw_img = lobshot_sender.debug_frame();
      if (draw_img.empty()) {
        draw_img = img.clone();
      }
      tools::draw_text(draw_img, "[LOBSHOT]", {10, 30}, {255, 255, 255});
      cv::resize(draw_img, draw_img, {}, 0.5, 0.5);
      cv::imshow("hero_debug", draw_img);
      report_fps(gimbal_mode);

      if (cv::waitKey(1) == 'q') break;
      continue;
    }

    lobshot_sender.set_enabled(false);

    auto last = std::chrono::steady_clock::now();
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = use_tradition ? detector.detect(img) : yolo.detect(img);
    auto draw_img = img.clone();
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
      auto target = targets.front();
      tools::draw_text(
        draw_img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(draw_img, image_points, {0, 255, 255});
      }

      auto gimbal_state = gimbal.state();
      auto command = aimer.aim(targets, t, gimbal_state.bullet_speed, q);
      shooter_fire = shooter.shoot(command, aimer, targets, gimbal_pos, gimbal_state.bullet_speed);

      auto aim_point = aimer.debug_aim_point;
      if (aim_point.valid) {
        auto image_points =
          solver.reproject_armor(aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
        tools::draw_points(draw_img, image_points, shooter_fire ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0));
      }

      auto plan_target = target;
      target_queue.push(plan_target);
    } else {
      target_queue.push(std::nullopt);
    }

    if (!armors.empty()) {
      solver.draw_armor_info(draw_img, armors.front());
    }

    cv::resize(draw_img, draw_img, {}, 0.5, 0.5);
    cv::imshow("hero_debug", draw_img);
    report_fps(gimbal_mode);

    if (cv::waitKey(1) == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  lobshot_sender.set_enabled(false);
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
