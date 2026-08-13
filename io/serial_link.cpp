#include "serial_link.hpp"

#include <unistd.h>

#include <cmath>
#include <thread>

#include "tools/logger.hpp"

namespace io
{
void open_serial_blocking(
  serial::Serial & serial, const std::string & port, uint32_t baudrate, uint32_t timeout_ms)
{
  serial.setPort(port);
  serial.setBaudrate(baudrate);
  serial.setFlowcontrol(serial::flowcontrol_none);
  serial.setParity(serial::parity_none);
  serial.setStopbits(serial::stopbits_one);
  serial.setBytesize(serial::eightbits);
  serial::Timeout time_out = serial::Timeout::simpleTimeout(timeout_ms);
  serial.setTimeout(time_out);
  serial.open();
  usleep(1000000);  // 1s wait，与原有行为一致
}

bool read_exact(serial::Serial & serial, uint8_t * buffer, size_t size)
{
  try {
    return serial.read(buffer, size) == size;
  } catch (const std::exception &) {
    return false;
  }
}

void reconnect_serial(
  serial::Serial & serial, std::atomic<bool> & quit, const char * log_tag,
  const std::function<void()> & on_success, int max_retry)
{
  for (int i = 0; i < max_retry && !quit; ++i) {
    tools::logger()->warn("{} Reconnecting serial, attempt {}/{}...", log_tag, i + 1, max_retry);
    try {
      serial.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial.open();
      on_success();
      tools::logger()->info("{} Reconnected serial successfully.", log_tag);
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("{} Reconnect failed: {}", log_tag, e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

bool euler_angles_sane(float yaw, float pitch, float roll, const char * log_tag)
{
  if (!std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(roll)) {
    tools::logger()->warn("{} Invalid euler: yaw={}, pitch={}, roll={}", log_tag, yaw, pitch, roll);
    return false;
  }

  constexpr double kMaxReasonableAngle = 32.0 * M_PI;
  if (
    std::abs(yaw) > kMaxReasonableAngle || std::abs(pitch) > kMaxReasonableAngle ||
    std::abs(roll) > kMaxReasonableAngle) {
    tools::logger()->warn(
      "{} Suspicious euler magnitude: yaw={:.4f}, pitch={:.4f}, roll={:.4f}", log_tag, yaw, pitch,
      roll);
    return false;
  }

  return true;
}

std::optional<Eigen::Quaterniond> decode_euler_quaternion(
  float yaw, float pitch, float roll, const char * log_tag)
{
  Eigen::Quaterniond q = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                           .normalized();

  auto x = q.x();
  auto y = q.y();
  auto z = q.z();
  auto w = q.w();

  if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(w)) {
    tools::logger()->warn("{} Invalid q: NaN detected - w={}, x={}, y={}, z={}", log_tag, w, x, y, z);
    return std::nullopt;
  }
  if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
    tools::logger()->warn(
      "{} Invalid q: magnitude check failed - w={}, x={}, y={}, z={}", log_tag, w, x, y, z);
    return std::nullopt;
  }

  return q;
}

}  // namespace io
