#include "cboard.hpp"

#include "io/serial_link.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"
#include "tools/crc.hpp"

namespace io
{  

  CBoard::CBoard(const std::string & config_path)
  : mode(Mode::idle),
    // shoot_mode(ShootMode::left_shoot),
    bullet_speed(0)
  {
    auto yaml = tools::load(config_path);
    auto com_port = tools::read<std::string>(yaml, "com_port");
    speed_control = tools::read<bool>(yaml, "speed_control");
    bullet_speed_config = tools::read<double>(yaml, "bullet_speed");
    try{
      io::open_serial_blocking(serial_, com_port, 115200);
    }
    catch (const std::exception &e)
    {
      tools::logger()->error("[Cboard] Failed to open serial: {}", e.what());
      exit(1);
    }
    
    // 启动线程
    thread_ = std::thread(&CBoard::read_thread, this);

    tools::logger()->info("[Cboard] Waiting for q...");
    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);
    tools::logger()->info("[Cboard] Opened.");
  }

  CBoard::~CBoard()
  {
    quit_ = true;
    if (thread_.joinable())
      thread_.join();
    try
    {
      serial_.close();
    }
    catch (...)
    {
    }
  }

  Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
  {
    if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

    while (true) {
      queue_.pop(data_behind_);
      if (data_behind_.timestamp > timestamp) break;
      data_ahead_ = data_behind_;
    }

    Eigen::Quaterniond q_a = data_ahead_.q.normalized();
    Eigen::Quaterniond q_b = data_behind_.q.normalized();
    auto t_a = data_ahead_.timestamp;
    auto t_b = data_behind_.timestamp;
    auto t_c = timestamp;
    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;

    // 四元数插值
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    return q_c;
  }

  void CBoard::send(Command command)
  {
    VisionToBoard tx_data;
    tx_data.head[0] = 'V';
    tx_data.head[1] = 'B';

    // 封装控制和开火位
    tx_data.shoot = 0;
    tx_data.control = 0;
    if (command.control)
    {
      tx_data.control = 1;
    }
    if (command.shoot)
    {
      tx_data.shoot = 1;
    }

    // double yaw =- 0.12;
    // double pitch = -0.12;


    // 转换 double 到 float 并赋值
    tx_data.yaw = (float)(command.yaw);
    tx_data.pitch = (float)(command.pitch);
    // tx_data.yaw = (float_t)(yaw);
    // tx_data.pitch = (float_t)(pitch);
    tx_data.horizon_distance = (float)command.horizon_distance;

    // 计算 CRC16
    // tx_data.crc16 = tools::get_crc16(
    //     reinterpret_cast<uint8_t *>(&tx_data), sizeof(tx_data) - sizeof(tx_data.crc16)- sizeof(tx_data.tail));
    tx_data.tail[0] = 'E';
    tx_data.tail[1] = 'N';
    try
    {
      // 通过串口发送
      serial_.write(reinterpret_cast<uint8_t *>(&tx_data), sizeof(tx_data));
      tools::logger()->debug("[Cboard] Sent command: control={}, shoot={}, yaw={:.4f}, pitch={:.4f}, horizon_distance={:.2f}, bullet_speed={:.2f}, avg_speed={:.2f}",
                            command.control, command.shoot, (float_t)(command.yaw ), (float_t)(command.pitch), (float_t)command.horizon_distance, bullet_speed, get_average_bullet_speed());
      // tools::logger()->debug("[Cboard] Sent command: control={}, shoot={}, yaw={}, pitch={}, horizon_distance={:.2f}",
      //                       command.control, command.shoot, (float)(command.yaw / 3.14159 * 180.0), (float)(command.pitch/ 3.14159 * 180.0), (float_t)command.horizon_distance);
    }
    catch (const std::exception &e)
    {
      tools::logger()->warn("[Cboard] Failed to write serial: {}", e.what());
    }
  }

  bool CBoard::read(uint8_t *buffer, size_t size)
  {
    return io::read_exact(serial_, buffer, size);
  }

  void CBoard::reconnect()
  {
    io::reconnect_serial(serial_, quit_, "[Cboard]", [this] { queue_.clear(); });
  }

  void CBoard::read_thread()
  {
    tools::logger()->info("[Cboard] read_thread started.");
    int error_count = 0;

    while (!quit_)
    {
      if (error_count > 5000)
      {
        error_count = 0;
        tools::logger()->warn("[Cboard] Too many errors, attempting to reconnect...");
        reconnect();
        continue;
      }

      // 1. 读取帧头
      if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head)))
      {
        error_count++;
        continue;
      }

      // 2. 检查帧头
      if (rx_data_.head[0] != 'B' || rx_data_.head[1] != 'V')
        continue;

      auto t = std::chrono::steady_clock::now();

      // 3. 读取数据包剩余部分
      if (!read(
              reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
              sizeof(rx_data_) - sizeof(rx_data_.head)))
      {
        error_count++;
        continue;
      }

      // // 4. 检查 CRC16
      // if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_)))
      // {
      //   tools::logger()->debug("[Cboard] CRC16 check failed.");
      //   continue;
      // }
      if(rx_data_.tail[0]!='E' || rx_data_.tail[1]!='N')
      {
        tools::logger()->debug("[Cboard] Tail check failed.");
        continue;
      }

      // tools::logger()->debug("[Cboard] Received data: mode={}", rx_data_.mode);
      error_count = 0;

      // --- 数据解析：旧版 q[4] 四元数字段已废弃，当前统一使用 yaw/pitch/roll ---
      auto yaw_ = rx_data_.yaw;
      auto pitch_ = rx_data_.pitch;
      auto roll_ = rx_data_.roll;
      // tools::logger()->debug("[Cboard] Received data: yaw={.02f}, pitch={.02f}, roll={.02f}", yaw_, pitch_, roll_);

      if (!io::euler_angles_sane(yaw_, pitch_, roll_, "[Cboard]")) {
        continue;
      }

      // 从欧拉角转换为四元数（ZYX顺序：yaw-pitch-roll），并做有效性检查
      auto q_from_euler = io::decode_euler_quaternion(yaw_, pitch_, roll_, "[Cboard]");
      if (q_from_euler) {
        queue_.push({{q_from_euler->w(), q_from_euler->x(), q_from_euler->y(), q_from_euler->z()}, t});
      }

      if(speed_control)
      {
        // 更新状态变量 (对应原 bullet_speed_canid_ 接收逻辑)
        bullet_speed = rx_data_.bullet_speed;

        // 滑动窗口：只有有效弹速且与前一个值不同时才加入窗口
        if (bullet_speed > 0) {
          bullet_speed_filter_.push(static_cast<float>(bullet_speed));
        }

        mode = Mode(rx_data_.mode);
        // shoot_mode = ShootMode(rx_data_.shoot_mode);
        // ft_angle = rx_data_.ft_angle;

        // 限制日志输出频率为1Hz (保持不变)
        static auto last_log_time = std::chrono::steady_clock::time_point::min();
        auto now = std::chrono::steady_clock::now();

        if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0)
        {
          // tools::logger()->info(
          //     "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
          //     bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
          // last_log_time = now;
          tools::logger()->info(
              "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
              bullet_speed, MODES[mode]);
          last_log_time = now;
        }
      }else{
        bullet_speed = bullet_speed_config;
        mode = Mode(rx_data_.mode);
      }
    }

    tools::logger()->info("[Cboard] read_thread stopped.");
  }

  double CBoard::get_average_bullet_speed() const
  {
    if (bullet_speed_filter_.empty()) {
      return bullet_speed_config;
    }
    return bullet_speed_filter_.average();
  }

}  // namespace io
