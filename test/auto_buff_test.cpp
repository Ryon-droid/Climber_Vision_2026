#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_target.hpp"  
#include "tasks/auto_buff/buff_type.hpp"
#include "tasks/auto_buff/buff_solver.hpp"

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "io/camera.hpp"
// #include "io/gimbal/gimbal.hpp"

// 命令行参数定义
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/test_buff.yaml    | yaml配置文件的路径}"
  "{cam-id i       | 1                      | 摄像头设备号}"
  "{width w        | 640                    | 摄像头分辨率宽度（640/1280）}"
  "{height h       | 480                    | 摄像头分辨率高度（480/720）}"
  "{fps f          |                        | }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  if (!cli.check()) {
    cli.printErrors();
    return -1;
  }

  const std::string config_path = cli.get<std::string>("config-path");
  const int cam_id = cli.get<int>("cam-id");          // 摄像头设备号
  const int cam_width = cli.get<int>("width");        // 摄像头宽度
  const int cam_height = cli.get<int>("height");      // 摄像头高度
  const int cam_fps = cli.get<int>("fps");            // 摄像头帧率

  //初始化工具类
  tools::Plotter plotter;
  tools::Exiter exiter;
  cv::namedWindow("实时能量机关检测", cv::WINDOW_NORMAL);
  cv::resizeWindow("实时能量机关检测", cam_width, cam_height);  // 匹配摄像头分辨率
  
  //摄像头驱动
  io::Camera camera(config_path);
  std::chrono::steady_clock::time_point frame_timestamp;

  //检测器+瞄准器（后续添加）
  auto_buff::Buff_Detector detector(config_path);  // 能量机关实时检测器
  auto_buff::Solver solver(config_path);           // 坐标解算器
  auto_buff::Aimer aimer(config_path);             // 瞄准器
  auto_buff::SmallTarget target;                   // 小能量机关目标跟踪器

  //实时帧循环处理
  cv::Mat frame;
  const auto time_base = std::chrono::steady_clock::now();
  int frame_count = 0;

  tools::logger()->info("实时检测开始！按 'q' 退出，按 ' '（空格）暂停");

  while (!exiter.exit()) {
    camera.read(frame, frame_timestamp);
    if (frame.empty()) {
      tools::logger()->warn("无法读取图像帧，跳过当前循环");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    frame_count++;
    
    const double frame_timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      frame_timestamp - time_base).count();

    std::optional<auto_buff::PowerRune> power_runes = detector.detect(frame);
    
    // 使用solver计算坐标
    solver.solve(power_runes);
    
    nlohmann::json debug_data;

    // 更新目标状态
    target.get_target(power_runes, frame_timestamp);
    
    if (power_runes.has_value()) {
      const auto& buff_data = power_runes.value();

      debug_data["frame_count"] = frame_count;
      debug_data["timestamp_ms"] = frame_timestamp_ms;
      debug_data["buff"]["status"] = "detected";  // 已检测到能量机关
      debug_data["buff"]["R_yaw_deg"] = buff_data.ypd_in_world[0] * 57.3;    // 相对yaw（弧度→角度）
      debug_data["buff"]["R_pitch_deg"] = buff_data.ypd_in_world[1] * 57.3;  // 相对pitch
      debug_data["buff"]["R_dis_m"] = buff_data.ypd_in_world[2];             // 相对距离（米）
      debug_data["buff"]["self_roll_deg"] = buff_data.ypr_in_world[2] * 57.3;// 扇叶角度

      tools::draw_point(frame, buff_data.r_center, cv::Scalar(0, 0, 255), 4);  // 中心（红色，4像素）
      for (const auto& blade : buff_data.fanblades) {
        if (blade.type != auto_buff::FanBlade_type::_unlight) {
          tools::draw_point(frame, blade.center, cv::Scalar(0, 255, 0), 3);  // 扇叶中心（绿色，3像素）
        }
      }

      // 使用Aimer进行瞄准计算
      double bullet_speed = 15.0; // 默认子弹速度，可以根据配置文件读取
      io::Command command = aimer.aim(target, frame_timestamp, bullet_speed);
      
      // 显示瞄准和射击指令
      if (command.control) {
        cv::putText(frame, "CONTROL: YES", cv::Point(20, 120), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
      } else {
        cv::putText(frame, "CONTROL: NO", cv::Point(20, 120), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
      }
      
      if (command.shoot) {
        cv::putText(frame, "SHOOT: YES", cv::Point(20, 150), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
      } else {
        cv::putText(frame, "SHOOT: NO", cv::Point(20, 150), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
      }
      
      // 显示计算出的yaw和pitch角度
      cv::putText(frame, fmt::format("YAW: {:.2f}", command.yaw * 57.3), cv::Point(20, 180), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
      cv::putText(frame, fmt::format("PITCH: {:.2f}", command.pitch * 57.3), cv::Point(20, 210), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);

      if (frame_count % 10 == 0) {
        tools::logger()->info("第{}帧：检测到能量机关，距离：{:.2f}m，扇叶角度：{:.1f}°",
                              frame_count, buff_data.ypd_in_world[2], 
                              buff_data.ypr_in_world[2] * 57.3);
        tools::logger()->info("控制权: {} 射击: {} Yaw: {:.2f} Pitch: {:.2f}", 
                              command.control, command.shoot, command.yaw * 57.3, command.pitch * 57.3);
      }
    } else {
      debug_data["buff"]["status"] = "not_detected";
      cv::putText(frame, "No Buff Detected", cv::Point(20, 40), 
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
      if (frame_count % 20 == 0) {
        tools::logger()->debug("第{}帧：未检测到能量机关，请调整摄像头角度", frame_count);
      }
    }

    const std::string fps_text = "FPS: N/A (HikRobot)";
    cv::putText(frame, fps_text, cv::Point(20, 80), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);  // 绿色帧率文字

    plotter.plot(debug_data);
    cv::imshow("实时能量机关检测", frame);

    //q=退出，空格=暂停
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {
      tools::logger()->info("用户按下退出键，实时检测结束");
      break;
    } else if (key == ' ') {
      tools::logger()->info("实时检测暂停，再次按空格继续");
      cv::putText(frame, "Paused", cv::Point(cam_width/2-50, cam_height/2), 
                  cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255, 0, 0), 3);
      cv::imshow("实时能量机关检测", frame);
      while (true) {
        const int pause_key = cv::waitKey(30);
        if (pause_key == ' ') {
          tools::logger()->info("实时检测继续");
          break;
        } else if (pause_key == 'q' || pause_key == 27) {
          cv::destroyAllWindows();
          tools::logger()->info("实时检测退出");
          return 0;
        }
      }
    }
  }

  cv::destroyAllWindows();
  tools::logger()->info("实时检测正常结束，已释放所有资源");

  return 0;
}