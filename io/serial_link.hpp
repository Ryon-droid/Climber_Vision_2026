#ifndef IO__SERIAL_LINK_HPP
#define IO__SERIAL_LINK_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <optional>
#include <string>

#include "serial/serial.h"

namespace io
{
// 按 8N1 + 指定超时打开串口，打开成功后按现有惯例等待 1s 让下位机就绪。
// 打开失败时向上抛出异常，调用方按各自现有方式处理（记录日志 + exit）。
void open_serial_blocking(
  serial::Serial & serial, const std::string & port, uint32_t baudrate, uint32_t timeout_ms = 20);

// 读取 size 字节到 buffer，返回是否成功读满 size 字节；读取异常会被吞掉并返回 false。
bool read_exact(serial::Serial & serial, uint8_t * buffer, size_t size);

// 通用串口重连骨架：关闭 -> 等待1s -> 打开，最多重试 max_retry 次。打开成功后调用
// on_success（各类自己的状态复位，例如清空队列/复位弹速滤波器），然后立即返回。
void reconnect_serial(
  serial::Serial & serial, std::atomic<bool> & quit, const char * log_tag,
  const std::function<void()> & on_success, int max_retry = 10);

// 检查从下位机收到的原始欧拉角是否"合理"（有限 + 幅值在合理范围内）。
// 用于在构造四元数之前就过滤掉明显损坏的帧。
bool euler_angles_sane(float yaw, float pitch, float roll, const char * log_tag);

// 由 yaw/pitch/roll（ZYX 欧拉角顺序）构造四元数，并做 NaN / 归一化幅值检查。
// 检查不通过时返回 std::nullopt 并打印警告日志；调用方应据此跳过入队但仍可
// 继续处理该帧的其它字段（这与原有行为一致）。
std::optional<Eigen::Quaterniond> decode_euler_quaternion(
  float yaw, float pitch, float roll, const char * log_tag);

}  // namespace io

#endif  // IO__SERIAL_LINK_HPP
