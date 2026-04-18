#include "perceptron.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <thread>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  io::USBCamera * usbcam1, io::USBCamera * usbcam2, const std::string & config_path)
: config_path_(config_path), detection_queue_(10), decider_(config_path), stop_flag_(false)
{
  // 读取配置文件，为感知相机创建单独的配置
  auto yaml = YAML::LoadFile(config_path);
  std::string perception_device = "CPU";  // 默认使用CPU
  if (yaml["perception_device"]) {
    perception_device = yaml["perception_device"].as<std::string>();
  }
  
  tools::logger()->info("Perception cameras using device: {}", perception_device);
  
  // 创建临时配置文件，强制感知相机使用指定设备
  std::string temp_config_path = "/tmp/perception_config.yaml";
  yaml["device"] = perception_device;
  std::ofstream temp_config(temp_config_path);
  temp_config << yaml;
  temp_config.close();
  
  // 初始化 YOLO，使用感知相机专用配置
  yolo_ = std::make_shared<auto_aim::YOLO>(temp_config_path, false);

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Sort cameras based on device_name (sharpness)
  std::vector<io::USBCamera *> cameras = {usbcam1, usbcam2};
  io::USBCamera * sorted_cams[4] = {nullptr, nullptr, nullptr, nullptr};
  std::vector<io::USBCamera *> unknown_cams;
  int available_camera_count = 0;

  for (auto cam : cameras) {
    if (cam == nullptr) {
      continue;  // 跳过空指针相机
    }
    available_camera_count++;
    if (cam->device_name.find("camera_back_right") != std::string::npos) sorted_cams[0] = cam;
    else if (cam->device_name.find("camera_back_left") != std::string::npos) sorted_cams[1] = cam;
    // else if (cam->device_name.find("camera_back_right") != std::string::npos) sorted_cams[2] = cam;
    // else if (cam->device_name.find("camera_back_left") != std::string::npos) sorted_cams[3] = cam;
    else unknown_cams.push_back(cam);
  }

  if (available_camera_count == 0) {
    tools::logger()->error("No cameras available! System cannot start.");
    throw std::runtime_error("No cameras available");
  }

  tools::logger()->info("Available cameras: {}/4", available_camera_count);

  // Fill in missing slots with unknown cameras
  int unknown_idx = 0;
  for (int i = 0; i < 4; ++i) {
    if (sorted_cams[i] == nullptr) {
      if (unknown_idx < unknown_cams.size()) {
        sorted_cams[i] = unknown_cams[unknown_idx++];
        tools::logger()->warn("Camera slot {} filled with unknown camera: {}", i + 1, sorted_cams[i]->device_name);
      } else {
        tools::logger()->warn("Camera slot {} is offline/unavailable", i + 1);
      }
    } else {
      tools::logger()->info("Camera slot {} assigned to {}", i + 1, sorted_cams[i]->device_name);
    }
  }

  auto c1 = sorted_cams[0];
  auto c2 = sorted_cams[1];
  // auto c3 = sorted_cams[2];
  // auto c4 = sorted_cams[3];

  // 创建推理线程
  inference_thread_ = std::thread([this, c1, c2] {
    inference_loop(c1, c2);
  });
  
  tools::logger()->info("Perceptron initialized.");
}

Perceptron::~Perceptron()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_flag_ = true;  // 设置退出标志
  }
  condition_.notify_all();  // 唤醒所有等待的线程

  if (inference_thread_.joinable()) {
    inference_thread_.join();
  }
  tools::logger()->info("Perceptron destructed.");
}

std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  std::vector<DetectionResult> result;
  DetectionResult temp;


  while (!detection_queue_.empty()) {
    detection_queue_.pop(temp);
    result.push_back(std::move(temp));
  }

  return result;
}

// [LEGACY] 历史遗留函数：将拼接图的象限坐标映射回单相机坐标系。
// 顺序推理模式已不再使用该函数，保留以备参考。
// void Perceptron::adjust_armor_coordinates(
//   auto_aim::Armor & armor, int offset_x, int offset_y, double scale_x, double scale_y, int width,
//   int height)
// {
//   cv::Point2f offset(offset_x, offset_y);

//   auto adjust_point = [&](cv::Point2f & p) {
//     p -= offset;
//     p.x *= scale_x;
//     p.y *= scale_y;
//   };

//   adjust_point(armor.center);
//   for (auto & p : armor.points) adjust_point(p);

//   armor.box.x -= offset_x;
//   armor.box.y -= offset_y;
//   armor.box.x = static_cast<int>(armor.box.x * scale_x);
//   armor.box.y = static_cast<int>(armor.box.y * scale_y);
//   armor.box.width = static_cast<int>(armor.box.width * scale_x);
//   armor.box.height = static_cast<int>(armor.box.height * scale_y);

//   // Adjust lightbars
//   adjust_point(armor.left.center);
//   adjust_point(armor.left.top);
//   adjust_point(armor.left.bottom);
//   for (auto & p : armor.left.points) adjust_point(p);
//   adjust_point(armor.left.rotated_rect.center);
//   armor.left.rotated_rect.size.width *= scale_x;
//   armor.left.rotated_rect.size.height *= scale_y;

//   adjust_point(armor.right.center);
//   adjust_point(armor.right.top);
//   adjust_point(armor.right.bottom);
//   for (auto & p : armor.right.points) adjust_point(p);
//   adjust_point(armor.right.rotated_rect.center);
//   armor.right.rotated_rect.size.width *= scale_x;
//   armor.right.rotated_rect.size.height *= scale_y;

//   // Recalculate center_norm
//   armor.center_norm.x = armor.center.x / width;
//   armor.center_norm.y = armor.center.y / height;
// }

void Perceptron::inference_loop(
  io::USBCamera * cam1, io::USBCamera * cam2)
{
  // 检查至少有一个相机可用
  if (!cam1 && !cam2) {
    tools::logger()->error("All camera pointers are null! Cannot run inference.");
    return;
  }

  tools::logger()->info("Inference loop started (sequential) with cameras: {} {}",
    cam1 ? "cam1" : "X", cam2 ? "cam2" : "X");

  try {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;
      }

      cv::Mat img1, img2;
      std::chrono::steady_clock::time_point ts1, ts2;

      bool any_valid = false;
      if (cam1) {
        cam1->read(img1, ts1);
        if (!img1.empty()) any_valid = true;
      }
      if (cam2) {
        cam2->read(img2, ts2);
        if (!img2.empty()) any_valid = true;
      }
      if (!any_valid) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      auto process_and_push = [&](std::list<auto_aim::Armor> & cam_armors, io::USBCamera * cam, std::chrono::steady_clock::time_point ts, const cv::Mat & img) {
        if (!cam || img.empty()) return;  // 跳过离线相机

        DetectionResult dr;
        dr.armors = std::move(cam_armors);
        dr.timestamp = ts;
        dr.camera_name = cam->device_name;
        dr.img = img.clone();

        if (!dr.armors.empty()) {
          auto delta_angle = decider_.delta_angle(dr.armors, dr.camera_name);
          dr.delta_yaw = delta_angle[0] / 57.3;
          dr.delta_pitch = delta_angle[1] / 57.3;
        } else {
          dr.delta_yaw = 0;
          dr.delta_pitch = 0;
        }
        detection_queue_.push(dr);
      };

      // 顺序单相机推理
      if (cam1 && !img1.empty()) {
        auto armors1 = yolo_->detect(img1);
        process_and_push(armors1, cam1, ts1, img1);
      }
      if (cam2 && !img2.empty()) {
        auto armors2 = yolo_->detect(img2);
        process_and_push(armors2, cam2, ts2, img2);
      }
    }
  } catch (const std::exception & e) {
    tools::logger()->error("Exception in inference_loop: {}", e.what());
  }
}

}  // namespace omniperception
