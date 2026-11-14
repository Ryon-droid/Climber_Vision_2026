#include "lobshot_comm.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "tools/crc.hpp"
#include "tools/logger.hpp"

namespace io
{
namespace
{

std::string hex_preview(const uint8_t * data, size_t size, size_t max_bytes = 32)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  const size_t preview_size = std::min(size, max_bytes);
  for (size_t i = 0; i < preview_size; ++i) {
    if (i != 0) {
      oss << ' ';
    }
    oss << std::setw(2) << static_cast<unsigned int>(data[i]);
  }
  if (size > preview_size) {
    oss << " ...";
  }
  return oss.str();
}

}  // namespace

LobshotComm::LobshotComm(
  const std::string & com_port, int baudrate, LobshotTxFrameMode tx_frame_mode)
: tx_frame_mode_(tx_frame_mode)
{
  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    auto timeout = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(timeout);
    serial_.open();
    usleep(1000000);
    tools::logger()->info(
      "[LobshotComm] Opened {} @ {} ({})",
      com_port, baudrate,
      tx_frame_mode_ == LobshotTxFrameMode::INNER_PACKET_ONLY ? "inner_only" : "rm_outer");
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to open lobshot serial: ") + e.what());
  }
}

LobshotComm::~LobshotComm()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (serial_.isOpen()) {
    serial_.close();
  }
}

bool LobshotComm::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return serial_.isOpen();
}

void LobshotComm::send(const uint8_t * data, size_t size)
{
  if (data == nullptr || size == 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!serial_.isOpen()) {
    return;
  }

  try {
    if (tx_frame_mode_ == LobshotTxFrameMode::INNER_PACKET_ONLY) {
      const auto bytes_written = serial_.write(data, size);
      if (bytes_written != size) {
        tools::logger()->warn(
          "[LobshotComm] Short inner write: expected={} actual={}", size, bytes_written);
        return;
      }

      if (!logged_first_packet_) {
        tools::logger()->info(
          "[LobshotComm] First inner packet ({}B): {}",
          size, hex_preview(data, size));
        logged_first_packet_ = true;
      }
      tx_packet_count_++;
      return;
    }

    CommFrame frame{};
    frame.header.sof = 0xA5;
    constexpr uint16_t payload_size = sizeof(frame.data);
    frame.header.data_length[0] = payload_size & 0xFF;
    frame.header.data_length[1] = (payload_size >> 8) & 0xFF;
    frame.header.seq = seq_++;
    frame.header.crc8 = tools::get_crc8(reinterpret_cast<const uint8_t *>(&frame.header), 4);

    constexpr uint16_t cmd_id = 0x0310;
    frame.cmd_id[0] = cmd_id & 0xFF;
    frame.cmd_id[1] = (cmd_id >> 8) & 0xFF;

    const size_t copy_size = std::min(size, sizeof(frame.data));
    std::memcpy(frame.data, data, copy_size);

    const auto crc = tools::get_crc16(
      reinterpret_cast<const uint8_t *>(&frame), sizeof(frame) - sizeof(frame.frame_tail));
    frame.frame_tail[0] = crc & 0xFF;
    frame.frame_tail[1] = (crc >> 8) & 0xFF;

    const auto bytes_written = serial_.write(
      reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
    if (bytes_written != sizeof(frame)) {
      tools::logger()->warn(
        "[LobshotComm] Short outer write: expected={} actual={}",
        sizeof(frame), bytes_written);
      return;
    }

    if (!logged_first_packet_) {
      tools::logger()->info(
        "[LobshotComm] First outer packet ({}B): {}",
        sizeof(frame),
        hex_preview(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame)));
      logged_first_packet_ = true;
    }

    tx_packet_count_++;
    if (tx_packet_count_ % 200 == 0) {
      tools::logger()->info(
        "[LobshotComm] TX packets={}, seq={}", tx_packet_count_, frame.header.seq);
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("[LobshotComm] Failed to write serial: {}", e.what());
  }
}

}  // namespace io
