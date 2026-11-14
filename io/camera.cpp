#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
// #include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{

YAML::Node resolve_camera_yaml(const YAML::Node & yaml, const std::string & camera_role)
{
  if (camera_role.empty()) {
    return yaml;
  }

  const std::string role_key = camera_role + "_camera";
  if (!yaml[role_key]) {
    tools::logger()->warn(
      "[Camera] {} not found, fallback to top-level camera config", role_key);
    return yaml;
  }

  return yaml[role_key];
}

template<typename T>
T read_camera_value(const YAML::Node & yaml, const std::string & key)
{
  if (yaml[key]) {
    return yaml[key].as<T>();
  }
  tools::logger()->error("[Camera] {} not found in camera config", key);
  exit(1);
}

template<typename T>
T read_camera_value_or(const YAML::Node & yaml, const std::string & key, const T & default_value)
{
  if (yaml[key]) {
    return yaml[key].as<T>();
  }
  return default_value;
}

}  // namespace

Camera::Camera(const std::string & config_path, const std::string & camera_role)
{
  auto yaml = tools::load(config_path);
  auto camera_yaml = resolve_camera_yaml(yaml, camera_role);
  auto camera_name = read_camera_value<std::string>(camera_yaml, "camera_name");
  auto exposure_ms = read_camera_value<double>(camera_yaml, "exposure_ms");

  // if (camera_name == "mindvision") {
  //   auto gamma = tools::read<double>(yaml, "gamma");
  //   auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
  //   camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  // }

  if (camera_name == "hikrobot") {
    auto gain = read_camera_value<double>(camera_yaml, "gain");
    auto vid_pid = read_camera_value<std::string>(camera_yaml, "vid_pid");
    auto user_id = read_camera_value_or<std::string>(camera_yaml, "user_id", "");
    auto serial_number = read_camera_value_or<std::string>(camera_yaml, "serial_number", "");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, user_id, serial_number);
    return;
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
