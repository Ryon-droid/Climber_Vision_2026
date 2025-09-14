#include <fmt/core.h>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"   
#include "tasks/auto_aim/tracker.hpp
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
    "{help h usage ? |                        | 输出命令行参数说明 }"
    "{@config-path   | configs/sentry.yaml    | yaml配置文件的路径}"
    "{tradition t    |  false                 | 是否使用传统方法识别}";
int main(int argc, char *argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    auto config_path = parser.get<std::string>(0);

    tools::Exiter exiter;
    tools::Plotter plotter;
    tools::Recorder recorder;

    auto_aim::YOLO detector(config_path, false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);

    cv::Mat img;
    while (!exiter.exit())
    {
        camera.read(img, t);

    }
}