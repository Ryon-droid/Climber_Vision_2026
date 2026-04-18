#include <fmt/core.h>
#include <fmt/format.h>

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[]){
    tools::Exiter exiter;

    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>(0);
    auto display = cli.has("display");

    io::USBCamera usbcam_left("camera_back_left", config_path);
    io::USBCamera usbcam_right("camera_back_right", config_path);
    io::Camera camera(config_path);

    auto_aim::YOLO yolo(config_path, false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);

    omniperception::Decider decider(config_path);
    omniperception::Perceptron perceptron(&usbcam_left, &usbcam_right, config_path);

    omniperception::DetectionResult switch_target;
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    io::Command last_command;

    int frame_count = 0;
    auto last_frame_time = std::chrono::steady_clock::now();


    while (!exiter.exit()) {
        auto loop_start = std::chrono::steady_clock::now();
        camera.read(img, timestamp);
        if (img.empty()) {
            tools::logger()->warn("Empty image from camera");
            continue;
        }

        auto armors = yolo.detect(img);

        decider.get_invincible_armor({});

        decider.armor_filter(armors);

        decider.set_priority(armors);

        auto detection_queue = perceptron.get_detection_queue();

        decider.sort(detection_queue);

        auto targets = tracker.track(armors, timestamp);

        omniperception::DetectionResult switch_target;
        if (!detection_queue.empty()) {
            switch_target = detection_queue.front();
        }

        io::Command command{false, false, 0, 0};

        /// 全向感知逻辑
        if (tracker.state() == "switching") {
            command.control = switch_target.armors.empty() ? false : true;
            command.shoot = false;
            command.pitch = tools::limit_rad(switch_target.delta_pitch);
            command.yaw = tools::limit_rad(switch_target.delta_yaw);
        }

        else if (tracker.state() == "lost") {
            command = decider.decide(detection_queue);
            command.yaw = tools::limit_rad(command.yaw);
        }

        double fps = 1.0 / tools::delta_time(loop_start, last_frame_time);
        last_frame_time = loop_start;
        std::string fps_text = fmt::format("FPS: {:.1f}", fps);

        tools::logger()->info(
            "Frame {}: State: {}, Command: Ctrl:{} Shoot:{} Yaw:{:.2f} Pitch:{:.2f} FPS:{:.1f}",
            frame_count, tracker.state(), command.control ? "ON" : "OFF",
            command.shoot ? "ON" : "OFF", command.yaw * 57.3, command.pitch * 57.3, fps);
        // Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

        // tools::logger()->info(
        //     "State: {}, Target Info: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]",
        //     tracker.state(), target_info[0], target_info[1], target_info[2], target_info[3]);

        if (display) {
            frame_count++;

            // 显示主相机图像
            if (!img.empty()) {
                cv::Mat display_img = img.clone();
                // 绘制装甲板
                for (const auto& armor : armors) {
                    tools::draw_points(display_img, armor.points, {0, 0, 255});
                    cv::Point center(armor.center_norm.x * display_img.cols, 
                                    armor.center_norm.y * display_img.rows);
                    cv::circle(display_img, center, 8, cv::Scalar(0, 0, 255), 2);
                    
                    std::string info = fmt::format("{} {} {}", 
                        auto_aim::ARMOR_NAMES[armor.name], 
                        auto_aim::ARMOR_TYPES[armor.type], 
                        auto_aim::COLORS[armor.color]);
                    tools::draw_text(display_img, info, armor.center, {0, 255, 255});
                }
                
                // 显示追踪状态
                std::string state_text = fmt::format("State: {}", tracker.state());
                tools::draw_text(display_img, state_text, {10, 60}, {255, 255, 0});
                
                // 显示控制命令
                std::string cmd_text = fmt::format("Ctrl: {} Shoot: {} Y:{:.2f} P:{:.2f}",
                    command.control ? "ON" : "OFF",
                    command.shoot ? "ON" : "OFF",
                    command.yaw * 57.3,
                    command.pitch * 57.3);
                tools::draw_text(display_img, cmd_text, {10, 90}, {255, 0, 255});
                
                tools::draw_text(display_img, fps_text, {10, 30}, {0, 255, 0});

                cv::resize(display_img, display_img, {}, 0.5, 0.5); 
                cv::imshow("Main Camera", display_img);
            }

            // 显示后置相机图像 - 直接使用DetectionResult中的图像
            if (!detection_queue.empty()) {
                for (const auto& res : detection_queue) {
                    if (res.img.empty()) continue;
                    
                    cv::Mat display_img = res.img.clone();
                    
                    // 绘制装甲板
                    for (const auto& armor : res.armors) {
                        tools::draw_points(display_img, armor.points, {0, 255, 0});
                        cv::Point center(armor.center_norm.x * display_img.cols, 
                                        armor.center_norm.y * display_img.rows);
                        cv::circle(display_img, center, 8, cv::Scalar(0, 255, 0), 2);
                        
                        std::string info = fmt::format("{} {} {}", 
                            auto_aim::ARMOR_NAMES[armor.name], 
                            auto_aim::ARMOR_TYPES[armor.type], 
                            auto_aim::COLORS[armor.color]);
                        tools::draw_text(display_img, info, armor.center, {0, 255, 255});
                    }
                    
                    std::string delta_text = fmt::format("Delta Y:{:.2f} P:{:.2f}",
                        res.delta_yaw * 57.3, res.delta_pitch * 57.3);
                    tools::draw_text(display_img, delta_text, {10, 60}, {255, 255, 0});
                    tools::draw_text(display_img, fps_text, {10, 30}, {0, 255, 0});
                    
                    cv::resize(display_img, display_img, {}, 0.5, 0.5); 
                    cv::imshow(res.camera_name, display_img);
                }
            }
            
            if (cv::waitKey(1) == 'q') break;
        }
    }

    return 0;
}