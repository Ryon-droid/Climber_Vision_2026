#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

// #include "tasks/auto_aim/aimer.hpp"
// #include "tasks/auto_aim/solver.hpp"
// #include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "io/camera.hpp"

const std::string keys =
    "{help h usage ? |                   | 输出命令行参数说明 }"
    "{config-path c  | configs/test.yaml | yaml配置文件的路径}";

int main(int argc, char *argv[])
{
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    auto config_path = cli.get<std::string>("config-path");
    if (cli.has("help") || !cli.check())
    {
        cli.printMessage();
        return 0;
    }

    tools::Plotter plotter;
    tools::Exiter exiter;

    auto_aim::YOLO yolo(config_path,false);


    cv::Mat img, drawing;
    std::chrono::steady_clock::time_point t;
    io::Camera camera(config_path);
    double last_t = -1;

    while(!exiter.exit())
    {
        camera.read(img, t);
        if (img.empty())
            break;

        auto armors = yolo.detect(img);
        cv::Mat draw_img = img.clone();
        for(const auto & armor : armors){
            std::vector<cv::Point> point(4);
            for(int i=0;i<4;i++){
                point[i] = cv::Point(static_cast<int>(armor.points[i].x),static_cast<int>(armor.points[i].y));
            }
            cv::polylines(draw_img, std::vector<std::vector<cv::Point>>{point}, true, cv::Scalar(0,255,0),2);
        
            tools::draw_text(draw_img, fmt::format("ID:{} conf{:.2f}",armor.name,armor.confidence), armor.center);
        }

        cv::resize(draw_img, draw_img, {}, 0.5, 0.5); // 显示时缩小图片尺寸
        cv::imshow("reprojection", draw_img);

        auto key = cv::waitKey(30);
        if (key == 'q')
            break;
    }

    return 0;
}