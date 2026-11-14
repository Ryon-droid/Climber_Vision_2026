#include <fmt/core.h>

#include <chrono>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto display = cli.has("display");


  std::unique_ptr<io::USBCamera> usbcam_back_left, usbcam_back_right;
  
  try {
    auto temp = std::make_unique<io::USBCamera>("camera_back_left", config_path);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));  // 等待后台初始化
    if (!temp->is_initialized()) {
      tools::logger()->warn("Camera back_left failed to initialize");
    } else {
      tools::logger()->info("Camera back_left initialized successfully");
      usbcam_back_left = std::move(temp);
    }
  } catch (const std::exception& e) {
    tools::logger()->warn("Camera back_left initialization failed: {}", e.what());
  }
  
  try {
    auto temp = std::make_unique<io::USBCamera>("camera_back_right", config_path);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));  // 等待后台初始化
    if (!temp->is_initialized()) {
      tools::logger()->warn("Camera back_right failed to initialize");
    } else {
      tools::logger()->info("Camera back_right initialized successfully");
      usbcam_back_right = std::move(temp);
    }
  } catch (const std::exception& e) {
    tools::logger()->warn("Camera back_right initialization failed: {}", e.what());
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // 检查至少有一个相机可用
  if (!usbcam_back_left && !usbcam_back_right) {
    tools::logger()->error("No rear cameras available! Exiting.");
    return -1;
  }

  // 顺序推理：后左和后右相机
  omniperception::Perceptron perceptron(usbcam_back_left.get(), usbcam_back_right.get(), config_path);

  int frame_count = 0;
  auto last_log_time = std::chrono::steady_clock::now();
  auto last_loop_time = std::chrono::steady_clock::now();
  
  // 缓存每个相机的最新结果，防止“吞相机”现象
  std::map<std::string, omniperception::DetectionResult> latest_results;

  while (!exiter.exit()) {
    auto results = perceptron.get_detection_queue();
    
    // 更新缓存
    for (const auto& res : results) {
      latest_results[res.camera_name] = res;
    }

    // 如果没有新结果，短暂休眠后继续
    if (results.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    frame_count++;
    
    /// 每秒输出一次完整的感知信息
    auto now = std::chrono::steady_clock::now();
    auto time_since_log = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_log_time).count();
    
    if (time_since_log >= 1000) {  // 每秒输出一次
      last_log_time = now;
      
      int total_detections = 0;
      for (const auto& res : results) {
        total_detections += res.armors.size();
      }
      
      tools::logger()->info("=== Frame {}: New detections from queue: {} ===", 
                            frame_count, total_detections);
      
      /// 输出每个相机的最新检测结果（来自缓存）
      if (!latest_results.empty()) {
        tools::logger()->info("  Latest detections by camera (cached):");
        for (const auto& [name, res] : latest_results) {
          tools::logger()->info("    {}: {} detections, Yaw: {:.2f}, Pitch: {:.2f}", 
                                name, res.armors.size(),
                                res.delta_yaw * 57.3, res.delta_pitch * 57.3);
          for (const auto & armor : res.armors) {
             tools::logger()->info("      - Armor: {} {}", 
               auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type]);
          }
        }
      } else {
        tools::logger()->info("  No detections yet");
      }
    }
    
    if (display) {
      // 计算显示FPS
      double fps = 1.0 / tools::delta_time(now, last_loop_time);
      last_loop_time = now;
      std::string fps_text = fmt::format("FPS: {:.1f}", fps);

      // 遍历缓存显示所有相机
      for (const auto& [name, res] : latest_results) {
        if (res.img.empty()) continue;
        
        cv::Mat display_img = res.img.clone();
        
        // 绘制装甲板
        for (const auto& armor : res.armors) {
          tools::draw_points(display_img, armor.points, {0, 0, 255});
          cv::Point center(armor.center_norm.x * display_img.cols, 
                          armor.center_norm.y * display_img.rows);
          cv::circle(display_img, center, 8, cv::Scalar(0, 0, 255), 2);
          
          std::string info = fmt::format("{} {} {}", auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type], auto_aim::COLORS[armor.color]);
          tools::draw_text(display_img, info, armor.center, {0, 255, 255});
        }
        
        tools::draw_text(display_img, fps_text, {10, 30}, {0, 255, 0});
        
        cv::imshow(name, display_img);
      }
      
      if (cv::waitKey(1) == 'q') break;
    }
  }


  return 0;
}