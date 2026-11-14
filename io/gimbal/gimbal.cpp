#include "gimbal.hpp"

#include "io/serial_link.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
  : solver_(config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  auto baudrate = tools::read<int>(yaml, "baudrate");
  speed_control_ = tools::read<bool>(yaml, "speed_control");
  bullet_speed_config_ = tools::read<double>(yaml, "bullet_speed");
  auto_fire_ = tools::read<bool>(yaml, "auto_fire");

  try {
    io::open_serial_blocking(serial_, com_port, baudrate);
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::LOBSHOT:
      return "LOBSHOT";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  fire = fire && auto_fire_;
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  // tx_data_.crc16 = tools::get_crc16(
  //   reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));
  tx_data_.tail[0] = 'E';
  tx_data_.tail[1] = 'N';

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    const auto mode = tx_data_.mode;
    const auto yaw_sent = tx_data_.yaw;
    const auto yaw_vel_sent = tx_data_.yaw_vel;
    const auto yaw_acc_sent = tx_data_.yaw_acc;
    const auto pitch_sent = tx_data_.pitch;
    const auto pitch_vel_sent = tx_data_.pitch_vel;
    const auto pitch_acc_sent = tx_data_.pitch_acc;
    tools::logger()->debug(
      "[Gimbal] Sent packet - mode: {}, yaw: {:.4f} rad ({:.2f} deg), yaw_vel: {:.4f}, "
      "yaw_acc: {:.4f}, pitch: {:.4f} rad ({:.2f} deg), pitch_vel: {:.4f}, pitch_acc: {:.4f}",
      mode, yaw_sent, yaw_sent * 57.3F, yaw_vel_sent, yaw_acc_sent, pitch_sent, pitch_sent * 57.3F,
      pitch_vel_sent, pitch_acc_sent);
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  return io::read_exact(serial_, buffer, size);
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'G' || rx_data_.head[1] != 'V') continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    // if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
    //   tools::logger()->debug("[Gimbal] CRC16 check failed.");
    //   continue;
    // }
    if (rx_data_.tail[0] != 'E' || rx_data_.tail[1] != 'N') continue;


    error_count = 0;
    auto yaw_ = rx_data_.yaw;
    auto pitch_ = rx_data_.pitch;
    auto roll_ = rx_data_.roll;
    if (!io::euler_angles_sane(yaw_, pitch_, roll_, "[Gimbal]")) {
      continue;
    }

    auto q = io::decode_euler_quaternion(yaw_, pitch_, roll_, "[Gimbal]");
    if (q) {
      queue_.push({{q->w(), q->x(), q->y(), q->z()}, t});
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (speed_control_) {
      const float current_bullet_speed = rx_data_.bullet_speed;
      bullet_speed_filter_.push(current_bullet_speed);
      state_.bullet_speed =
        bullet_speed_filter_.full() ? bullet_speed_filter_.average() : current_bullet_speed;
    } else {
      state_.bullet_speed = bullet_speed_config_;
    }

    state_.yaw = tools::limit_rad(rx_data_.yaw);
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = tools::limit_rad(rx_data_.pitch);
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_count = rx_data_.bullet_count;

    auto new_mode = GimbalMode::IDLE;
    switch (rx_data_.mode) {
      case 0:
        new_mode = GimbalMode::IDLE;
        break;
      case 1:
        new_mode = GimbalMode::AUTO_AIM;
        break;
      case 2:
        new_mode = GimbalMode::LOBSHOT;
        break;
      default:
        new_mode = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
    if (new_mode != mode_) {
      tools::logger()->info("[Gimbal] RX mode: {} -> {}", str(mode_), str(new_mode));
    }
    mode_ = new_mode;
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  io::reconnect_serial(serial_, quit_, "[Gimbal]", [this] {
    queue_.clear();
    bullet_speed_filter_.reset();
  });
}

}  // namespace io
