#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/img_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

const std::string keys =
    "{help h usage ? |                        | 输出命令行参数说明 }"
    "{@config-path   | configs/hero.yaml      | yaml配置文件的路径}"
    "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char *argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    auto config_path = parser.get<std::string>(0);
    auto yaml = tools::load(config_path);

    tools::Exiter exiter;
    tools::Plotter plotter;
    tools::Recorder recorder;

    // 初始化相机
    io::Camera camera(config_path);
    
    // 初始化Gimbal
    io::Gimbal gimbal(config_path);
    
    // 初始化自瞄模块
    auto use_tradition = tools::read<bool>(yaml, "use_traditional");
    auto_aim::YOLO yolo(config_path, false);
    auto_aim::Detector detector(config_path, false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);
    auto_aim::Planner planner(config_path);

    cv::Mat img, draw_img;
    std::list<auto_aim::Armor> armors;
    std::list<auto_aim::Target> targets;
    std::chrono::steady_clock::time_point t;
    Eigen::Quaterniond q;
    Eigen::VectorXd x;
    double last_t = -1;
    
    while (!exiter.exit())
    {
        // 读取图像
        camera.read(img, t);
        if (img.empty())
            break;
        
        q = gimbal.q(t - 1ms);

        recorder.record(img, q, t);

        auto last = std::chrono::steady_clock::now();
        
        solver.set_R_gimbal2world(q);
        Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
        
        // 目标检测
        if (use_tradition) armors = detector.detect(img);
        else armors = yolo.detect(img);
        
        draw_img = img.clone();
        for (const auto &armor : armors) {
            tools::draw_text(draw_img, fmt::format("ID:{},conf{:.2f},type:{}",
                auto_aim::ARMOR_NAMES[armor.name],
                armor.confidence, 
                auto_aim::ARMOR_TYPES[armor.type]),
                 armor.center);
        }
        
        // 目标跟踪
        targets = tracker.track(armors, t);
        
        nlohmann::json data;
        
        if (!targets.empty()) {
            auto target = targets.front();
            tools::draw_text(draw_img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});
            x = target.ekf_x();
            data["a"] = x[6] * 57.3;
            data["w"] = x[7];
            
            std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
            for (const Eigen::Vector4d &xyza : armor_xyza_list) {
                auto image_points =
                    solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);          
                tools::draw_points(draw_img, image_points, {0, 255, 255});
            }
            
            auto aim_point = aimer.debug_aim_point;
            Eigen::Vector4d aim_xyza = aim_point.xyza;
            auto image_points =
                solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
            if (aim_point.valid)
                tools::draw_points(draw_img, image_points, {0, 0, 255});
            else
                tools::draw_points(draw_img, image_points, {255, 0, 0});
            
            auto now = std::chrono::steady_clock::now();
            auto dt = tools::delta_time(now, last);
            tools::logger()->info("{:.2f} fps", 1 / dt);
        }
        
        if (!armors.empty()) {
            solver.draw_armor_info(draw_img, armors.front());
        }
        
        cv::resize(draw_img, draw_img, {}, 0.5, 0.5);
        cv::imshow("hero_debug", draw_img);
        
        auto key = cv::waitKey(1);
        if (key == 'q')
            break;
        
        // 弹道解算与瞄准
        if (!targets.empty()) {
            auto target = targets.front();
            auto gimbal_state = gimbal.state();
            
            // 使用Planner规划轨迹
            auto plan = planner.plan(target, gimbal_state.bullet_speed);
            
            data["t"] = tools::delta_time(std::chrono::steady_clock::now(), last);
            data["target_yaw"] = plan.target_yaw * 57.3;
            data["target_pitch"] = plan.target_pitch * 57.3;
            data["yaw"] = gimbal_pos[0] * 57.3;
            data["pitch"] = gimbal_pos[1] * 57.3;
            data["fire"] = plan.fire ? 1 : 0;
            data["err_yaw"] = (gimbal_pos[0] - plan.target_yaw) * 57.3;
            data["err_pitch"] = (gimbal_pos[1] - plan.target_pitch) * 57.3;
            plotter.plot(data);
            
            // 发送控制指令
            gimbal.send(plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel, plan.pitch_acc);
        }
        else {
            // 发送空指令
            gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
        }
    }
    
    return 0;
}