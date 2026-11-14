#include <fmt/core.h>

#include <chrono>
#include <fstream>
// #include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
// #include "tools/plotter.hpp"
#include "io/camera.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "io/gimbal/gimbal.hpp"


using namespace std::chrono_literals;  

const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明 }"
    "{config-path c  | configs/sentry.yaml | yaml配置文件的路径}";
    // "{tradition t    | true | 是否使用传统方法识别}";

int main(int argc, char *argv[])
{
    // 1. 初始化与参数读取
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) { cli.printMessage(); return 0; }
    auto config_path = cli.get<std::string>("config-path");
    auto yaml = tools::load(config_path);   
    auto use_tradition = tools::read<bool>(yaml, "use_traditional");
    if (!cli.check()) { cli.printErrors(); return -1; }

    // 2. 任务类初始化
    tools::Exiter exiter;
    auto_aim::YOLO yolo(config_path, false);
    auto_aim::Detector detector(config_path, false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);
    io::USBCamera usbcam_left("camera_back_left", config_path);
    io::USBCamera usbcam_right("camera_back_right", config_path);
    omniperception::Decider decider(config_path);
    omniperception::Perceptron perceptron(&usbcam_left, &usbcam_right, config_path);

    // 3. 数据变量定义
    cv::Mat img;
    std::list<auto_aim::Armor> armors;
    std::list<auto_aim::Target> targets;
    std::chrono::steady_clock::time_point t;
    Eigen::Quaterniond q;
    io::Camera camera(config_path);
    io::Gimbal gimbal(config_path);
    omniperception::DetectionResult switch_target;

    auto last_omni_time = std::chrono::steady_clock::now();
    io::Command last_omni_command{false, false, 0, 0};
    io::Command command{false, false, 0, 0};

    // 4. 主循环
    while (!exiter.exit())
    {
        camera.read(img, t);
        if (img.empty()) break;
        
        q = gimbal.q(t - 1ms);
        auto frame_start = std::chrono::steady_clock::now();

        // 计算位姿与欧拉角 (gimbal_pos 修复点)
        solver.set_R_gimbal2world(q);
        Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

        // 识别逻辑
        if (use_tradition) armors = detector.detect(img);
        else armors = yolo.detect(img);

        // 全向感知预处理
        decider.get_invincible_armor({});
        decider.armor_filter(armors);
        decider.set_priority(armors);
        
        // 获取全向队列 (detection_queue 修复点)
        auto detection_queue = perceptron.get_detection_queue();
        decider.sort(detection_queue);

        // 追踪逻辑
        targets = tracker.track(armors, t);
        // if (!detection_queue.empty()) {
        //     switch_target = detection_queue.front();
        // }

        /// 全向感知逻辑
        if (tracker.state() == "lost")
            command = decider.decide(detection_queue);
        else {
            auto gimbal_state = gimbal.state();
            command = aimer.aim(targets, t, gimbal_state.bullet_speed, q);
            /// 发射逻辑
            command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos, gimbal_state.bullet_speed);
        }

        // // --- 核心控制逻辑决策树 ---
        // if (!targets.empty()) 
        // {
        //     command = aimer.aim(targets, t, cboard.bullet_speed, q);
        //     if (tracker.state() == "switching") {
        //         command.control = true; 
        //         command.shoot = false;
        //         command.yaw = tools::limit_rad(switch_target.delta_yaw);
        //         command.pitch = tools::limit_rad(switch_target.delta_pitch);
        //     } else {
        //         command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
        //     }
        //     // 主相机工作时，重置记忆
        //     last_omni_command = {false, false, 0, 0}; 
        // }
        // else if (!detection_queue.empty()) 
        // {
        //     command = decider.decide(detection_queue);
        //     command.control = true;
        //     last_omni_command = command; // 存入记忆
        //     last_omni_time = std::chrono::steady_clock::now();
        //     tools::logger()->info("Omni-Perception Active: Target detected.");
        // }
        // else 
        // {
        //     // 记忆维持逻辑
        //     auto now = std::chrono::steady_clock::now();
        //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_omni_time).count();
        //     if (duration < 500 && last_omni_command.control) {
        //         command = last_omni_command;
        //     } else {
        //         command = {false, false, 0, 0};
        //     }
        // }

        // 5. 发送指令
        gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

        // 6. 可视化 (可选)
        // cv::imshow("debug", img); 
        if (cv::waitKey(1) == 'q') break;
    }

    return 0;
}