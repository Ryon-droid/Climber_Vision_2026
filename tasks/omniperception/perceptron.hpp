#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <chrono>
#include <list>
#include <memory>

#include "decider.hpp"
#include "detection.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tools/thread_pool.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{

class Perceptron
{
public:
  Perceptron(
    io::USBCamera * usbcma1, io::USBCamera * usbcam2, const std::string & config_path);

  ~Perceptron();

  std::vector<DetectionResult> get_detection_queue();

private:
  void inference_loop(
    io::USBCamera * cam1, io::USBCamera * cam2);  
  // 调整识别到的装甲板坐标到各自相机坐标系
  // [LEGACY] 历史遗留：用于拼接四象限坐标映射，顺序推理已不再使用
  // [[deprecated("Legacy: used for stitched quadrant coordinate mapping; not used in sequential inference")]]
  // void adjust_armor_coordinates(
  //   auto_aim::Armor & armor, int offset_x, int offset_y, double scale_x, double scale_y, int width,
  //   int height);

  std::string config_path_;
  std::thread inference_thread_;
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;

  std::shared_ptr<auto_aim::YOLO> yolo_;

  Decider decider_;
  bool stop_flag_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

}  // namespace omniperception
#endif