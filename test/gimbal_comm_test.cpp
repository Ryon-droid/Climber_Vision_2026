#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>
#include <serial/serial.h>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{config-path c  | configs/hero.yaml | yaml配置文件的路径}"
  "{raw r          | false            | 同时打印原始十六进制数据}";

namespace
{
bool read_exact(serial::Serial & serial, uint8_t * buffer, size_t size)
{
  try {
    return serial.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[GimbalCommTest] Read failed: {}", e.what());
    return false;
  }
}

void print_hex_packet(const io::GimbalToVision & packet)
{
  const auto * bytes = reinterpret_cast<const uint8_t *>(&packet);
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < sizeof(io::GimbalToVision); ++i) {
    oss << std::setw(2) << static_cast<int>(bytes[i]);
    if (i + 1 != sizeof(io::GimbalToVision)) {
      oss << ' ';
    }
  }
  tools::logger()->info("[GimbalCommTest] raw: {}", oss.str());
}
}  // namespace

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

  const auto config_path = cli.get<std::string>("config-path");
  const bool print_raw = cli.get<bool>("raw");

  const auto yaml = tools::load(config_path);
  const auto com_port = tools::read<std::string>(yaml, "com_port");
  const auto baudrate = tools::read<int>(yaml, "baudrate");

  tools::Exiter exiter;
  serial::Serial serial_port;

  try {
    serial_port.setPort(com_port);
    serial_port.setBaudrate(baudrate);
    serial_port.setFlowcontrol(serial::flowcontrol_none);
    serial_port.setParity(serial::parity_none);
    serial_port.setStopbits(serial::stopbits_one);
    serial_port.setBytesize(serial::eightbits);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(20);
    serial_port.setTimeout(timeout);
    serial_port.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[GimbalCommTest] Failed to open serial: {}", e.what());
    return 1;
  }

  tools::logger()->info(
    "[GimbalCommTest] Listening on {} @ {} baud, packet size {} bytes",
    com_port, baudrate, sizeof(io::GimbalToVision));

  io::GimbalToVision packet{};
  uint64_t packet_count = 0;
  uint64_t head_miss_count = 0;
  uint64_t tail_miss_count = 0;
  auto last_report = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    if (!read_exact(serial_port, reinterpret_cast<uint8_t *>(&packet.head), sizeof(packet.head))) {
      std::this_thread::sleep_for(2ms);
      continue;
    }

    if (packet.head[0] != 'G' || packet.head[1] != 'V') {
      head_miss_count++;
      continue;
    }

    if (!read_exact(
          serial_port,
          reinterpret_cast<uint8_t *>(&packet) + sizeof(packet.head),
          sizeof(packet) - sizeof(packet.head))) {
      std::this_thread::sleep_for(2ms);
      continue;
    }

    if (packet.tail[0] != 'E' || packet.tail[1] != 'N') {
      tail_miss_count++;
      tools::logger()->warn(
        "[GimbalCommTest] Tail mismatch: got [{:#04x} {:#04x}] expected ['E' 'N']",
        packet.tail[0], packet.tail[1]);
      if (print_raw) {
        print_hex_packet(packet);
      }
      continue;
    }

    packet_count++;
    const auto mode = packet.mode;
    const auto yaw = packet.yaw;
    const auto yaw_vel = packet.yaw_vel;
    const auto pitch = packet.pitch;
    const auto pitch_vel = packet.pitch_vel;
    const auto roll = packet.roll;
    const auto bullet_speed = packet.bullet_speed;
    const auto bullet_count = packet.bullet_count;
    const bool finite_values =
      std::isfinite(yaw) && std::isfinite(yaw_vel) && std::isfinite(pitch) &&
      std::isfinite(pitch_vel) && std::isfinite(roll) && std::isfinite(bullet_speed);

    if (!finite_values) {
      tools::logger()->warn(
        "[GimbalCommTest] Non-finite payload: mode={}, yaw={}, yaw_vel={}, pitch={}, "
        "pitch_vel={}, roll={}, bullet_speed={}, bullet_count={}",
        mode, yaw, yaw_vel, pitch, pitch_vel, roll, bullet_speed, bullet_count);
      if (print_raw) {
        print_hex_packet(packet);
      }
      continue;
    }

    tools::logger()->info(
      "[GimbalCommTest] packet={} mode={} yaw={:.4f} yaw_vel={:.4f} pitch={:.4f} "
      "pitch_vel={:.4f} roll={:.4f} bullet_speed={:.3f} bullet_count={}",
      packet_count, static_cast<int>(mode), yaw, yaw_vel, pitch, pitch_vel, roll, bullet_speed,
      bullet_count);

    if (print_raw) {
      print_hex_packet(packet);
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_report >= 1s) {
      tools::logger()->info(
        "[GimbalCommTest] stats: valid_packets={} head_miss={} tail_miss={}",
        packet_count, head_miss_count, tail_miss_count);
      last_report = now;
    }
  }

  try {
    serial_port.close();
  } catch (...) {
  }

  tools::logger()->info(
    "[GimbalCommTest] exit: valid_packets={} head_miss={} tail_miss={}",
    packet_count, head_miss_count, tail_miss_count);
  return 0;
}
