#ifndef TASKS__LOBSHOT__LOBSHOT_HPP
#define TASKS__LOBSHOT__LOBSHOT_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>

#include "io/lobshot/lobshot_comm.hpp"

namespace lobshot
{

class Lobshot
{
public:
  explicit Lobshot(const std::string & config_path);
  ~Lobshot();

  void set_enabled(bool enabled);
  bool enabled() const;

  void process(
    const cv::Mat & input,
    std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now());

  cv::Mat debug_frame() const;

private:
  void initialize_gstreamer();
  void shutdown_gstreamer();
  void poll_gstreamer_bus();

  cv::Mat preprocess_image(const cv::Mat & input);
  void draw_overlay(cv::Mat & frame) const;
  void push_frame_to_gstreamer(const cv::Mat & frame);
  void pull_stream_and_packetize();

  void send_loop();
  void send_packet_once();
  void reset_runtime_state();
  void clear_backlog_locked();

  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  GstElement * appsink_ = nullptr;
  GstBus * bus_ = nullptr;
  bool gstreamer_ready_ = false;

  mutable std::mutex comm_mutex_;
  std::shared_ptr<io::LobshotComm> comm_;

  std::thread send_thread_;
  std::atomic<bool> quit_ = false;
  std::atomic<bool> enabled_ = false;

  mutable std::mutex debug_mutex_;
  cv::Mat debug_frame_;

  mutable std::mutex buffer_mutex_;
  std::deque<std::vector<uint8_t>> frame_queue_;
  std::vector<uint8_t> current_send_frame_;
  uint16_t current_frame_no_ = 0;
  uint16_t current_frag_no_ = 0;
  size_t current_send_offset_ = 0;
  uint64_t synthetic_sequence_id_ = 0;
  uint64_t encoded_au_count_ = 0;
  uint64_t dropped_bytes_ = 0;
  uint32_t dropped_events_ = 0;

  cv::Mat background_gray_f32_;
  cv::Mat motion_erode_kernel_;
  cv::Mat motion_dilate_kernel_;
  std::deque<cv::Mat> motion_mask_history_;
  std::deque<cv::Mat> trail_frame_history_;

  bool has_last_encode_time_ = false;
  std::chrono::steady_clock::time_point last_encode_time_{};
  std::chrono::steady_clock::time_point last_telemetry_time_{};
  std::chrono::steady_clock::time_point last_idle_log_time_{};
  std::chrono::steady_clock::time_point last_appsink_empty_log_time_{};

  int crop_size_ = 800;
  int output_size_ = 400;
  int output_fps_ = 40;
  guint target_bitrate_ = 40;
  int motion_threshold_ = 14;
  int motion_erode_px_ = 1;
  int motion_dilate_px_ = 2;
  int motion_trail_frames_ = 8;
  double trail_disable_motion_ratio_ = 0.30;
  double bg_update_alpha_ = 0.01;
  double bg_blur_sigma_ = 1.2;
  int center_clear_size_ = 100;
  bool force_monochrome_ = false;
  bool static_simplify_ = true;
  bool bake_overlay_ = true;
  bool fixed_test_payload_mode_ = false;
  bool send_inner_packet_only_ = false;
  double bandwidth_limit_kbytes_ = 12.0;
  double max_tx_delay_s_ = 1.0;
  int tx_rate_hz_ = 48;
  std::string x265_preset_ = "faster";
  int crosshair_offset_x_ = 0;
  int crosshair_offset_y_ = 0;
  int crosshair_width_ = 1;
  int center_circle_radius_ = 24;
  int center_circle_width_ = 1;
  std::string lobshot_com_port_ = "/dev/ttyACM1";
  int lobshot_baudrate_ = 921600;
};

}  // namespace lobshot

#endif  // TASKS__LOBSHOT__LOBSHOT_HPP
