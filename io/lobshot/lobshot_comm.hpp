#ifndef IO__LOBSHOT__LOBSHOT_COMM_HPP
#define IO__LOBSHOT__LOBSHOT_COMM_HPP

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include "serial/serial.h"

namespace io
{

enum class LobshotTxFrameMode
{
  RM_OUTER_FRAME,
  INNER_PACKET_ONLY
};

class LobshotComm
{
public:
  LobshotComm(const std::string & com_port, int baudrate, LobshotTxFrameMode tx_frame_mode);
  ~LobshotComm();

  void send(const uint8_t * data, size_t size = 300);
  bool is_open() const;

private:
  struct __attribute__((packed)) FrameHeader
  {
    uint8_t sof = 0xA5;
    uint8_t data_length[2] = {0};
    uint8_t seq = 0;
    uint8_t crc8 = 0;
  };
  static_assert(sizeof(FrameHeader) == 5);

  struct __attribute__((packed)) CommFrame
  {
    FrameHeader header;
    uint8_t cmd_id[2] = {0};
    uint8_t data[300] = {0};
    uint8_t frame_tail[2] = {0};
  };
  static_assert(sizeof(CommFrame) == 309);

  serial::Serial serial_;
  mutable std::mutex mutex_;
  LobshotTxFrameMode tx_frame_mode_ = LobshotTxFrameMode::RM_OUTER_FRAME;
  uint8_t seq_ = 0;
  uint64_t tx_packet_count_ = 0;
  bool logged_first_packet_ = false;
};

}  // namespace io

#endif  // IO__LOBSHOT__LOBSHOT_COMM_HPP
