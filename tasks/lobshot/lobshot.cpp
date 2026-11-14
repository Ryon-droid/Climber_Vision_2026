#include "lobshot.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <sstream>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace lobshot
{
namespace
{

template<typename T>
T read_or(const YAML::Node & yaml, const std::string & key, const T & default_value)
{
  return yaml[key] ? yaml[key].as<T>() : default_value;
}

std::once_flag gst_init_once_flag;

}  // namespace

Lobshot::Lobshot(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  crop_size_ = read_or<int>(yaml, "crop_size", crop_size_);
  output_size_ = read_or<int>(yaml, "output_size", output_size_);
  output_fps_ = std::max(1, read_or<int>(yaml, "output_fps", output_fps_));
  target_bitrate_ = static_cast<guint>(std::max(1, read_or<int>(yaml, "target_bitrate", target_bitrate_)));
  motion_threshold_ = read_or<int>(yaml, "motion_threshold", motion_threshold_);
  motion_erode_px_ = std::clamp(read_or<int>(yaml, "motion_erode_px", motion_erode_px_), 0, 20);
  motion_dilate_px_ = std::clamp(read_or<int>(yaml, "motion_dilate_px", motion_dilate_px_), 0, 20);
  motion_trail_frames_ = std::clamp(read_or<int>(yaml, "motion_trail_frames", motion_trail_frames_), 0, 15);
  trail_disable_motion_ratio_ = std::clamp(
    read_or<double>(yaml, "trail_disable_motion_ratio", trail_disable_motion_ratio_), 0.0, 1.0);
  bg_update_alpha_ = std::clamp(read_or<double>(yaml, "bg_update_alpha", bg_update_alpha_), 0.001, 0.2);
  bg_blur_sigma_ = std::max(0.0, read_or<double>(yaml, "bg_blur_sigma", bg_blur_sigma_));
  center_clear_size_ = std::max(0, read_or<int>(yaml, "center_clear_size", center_clear_size_));
  force_monochrome_ = read_or<bool>(yaml, "force_monochrome", force_monochrome_);
  static_simplify_ = read_or<bool>(yaml, "static_simplify", static_simplify_);
  bake_overlay_ = read_or<bool>(yaml, "bake_overlay", bake_overlay_);
  fixed_test_payload_mode_ = read_or<bool>(yaml, "fixed_test_payload_mode", fixed_test_payload_mode_);
  send_inner_packet_only_ = read_or<bool>(yaml, "send_inner_packet_only", send_inner_packet_only_);
  bandwidth_limit_kbytes_ = std::max(1.0, read_or<double>(yaml, "bandwidth_limit_kbytes", bandwidth_limit_kbytes_));
  max_tx_delay_s_ = std::max(0.05, read_or<double>(yaml, "max_tx_delay_s", max_tx_delay_s_));
  tx_rate_hz_ = std::max(1, read_or<int>(yaml, "tx_rate_hz", tx_rate_hz_));
  x265_preset_ = read_or<std::string>(yaml, "x265_preset", x265_preset_);
  crosshair_offset_x_ = read_or<int>(yaml, "crosshair_offset_x", crosshair_offset_x_);
  crosshair_offset_y_ = read_or<int>(yaml, "crosshair_offset_y", crosshair_offset_y_);
  crosshair_width_ = std::max(1, read_or<int>(yaml, "crosshair_width", crosshair_width_));
  center_circle_radius_ = std::max(0, read_or<int>(yaml, "center_circle_radius", center_circle_radius_));
  center_circle_width_ = std::max(1, read_or<int>(yaml, "center_circle_width", center_circle_width_));
  lobshot_com_port_ = read_or<std::string>(yaml, "lobshot_com_port", lobshot_com_port_);
  lobshot_baudrate_ = read_or<int>(yaml, "lobshot_baudrate", lobshot_baudrate_);

  initialize_gstreamer();
  send_thread_ = std::thread(&Lobshot::send_loop, this);
  tools::logger()->info(
    "[Lobshot] Ready: {}x{}@{}fps bitrate={} preset={} second_port={} baud={} tx={}Hz",
    output_size_,
    output_size_,
    output_fps_,
    target_bitrate_,
    x265_preset_,
    lobshot_com_port_,
    lobshot_baudrate_,
    tx_rate_hz_);
}

Lobshot::~Lobshot()
{
  quit_ = true;
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  shutdown_gstreamer();
}

void Lobshot::set_enabled(bool enabled)
{
  const bool previous = enabled_.exchange(enabled);
  if (previous == enabled) {
    return;
  }

  if (enabled) {
    try {
      auto comm = std::make_shared<io::LobshotComm>(
        lobshot_com_port_,
        lobshot_baudrate_,
        send_inner_packet_only_ ? io::LobshotTxFrameMode::INNER_PACKET_ONLY : io::LobshotTxFrameMode::RM_OUTER_FRAME);
      {
        std::lock_guard<std::mutex> lock(comm_mutex_);
        comm_ = std::move(comm);
      }
    } catch (const std::exception & e) {
      enabled_.store(false);
      tools::logger()->error("[Lobshot] {}", e.what());
      return;
    }
  } else {
    std::lock_guard<std::mutex> lock(comm_mutex_);
    comm_.reset();
  }

  reset_runtime_state();
  tools::logger()->info("[Lobshot] {}", enabled ? "Enabled" : "Disabled");
}

bool Lobshot::enabled() const
{
  return enabled_.load();
}

void Lobshot::process(const cv::Mat & input, std::chrono::steady_clock::time_point timestamp)
{
  if (!enabled_ || input.empty()) {
    return;
  }

  if (!fixed_test_payload_mode_) {
    if (has_last_encode_time_) {
      const auto frame_interval = std::chrono::nanoseconds(1000000000LL / output_fps_);
      if (timestamp - last_encode_time_ < frame_interval) {
        return;
      }
    }
    last_encode_time_ = timestamp;
    has_last_encode_time_ = true;

    auto processed = preprocess_image(input);
    if (bake_overlay_) {
      draw_overlay(processed);
    }

    {
      std::lock_guard<std::mutex> lock(debug_mutex_);
      processed.copyTo(debug_frame_);
    }

    push_frame_to_gstreamer(processed);
    poll_gstreamer_bus();
    pull_stream_and_packetize();
    poll_gstreamer_bus();
  } else {
    std::lock_guard<std::mutex> lock(debug_mutex_);
    input.copyTo(debug_frame_);
  }
}

cv::Mat Lobshot::debug_frame() const
{
  std::lock_guard<std::mutex> lock(debug_mutex_);
  return debug_frame_.clone();
}

void Lobshot::initialize_gstreamer()
{
  std::call_once(gst_init_once_flag, []() {
    gst_init(nullptr, nullptr);
  });

  pipeline_ = gst_pipeline_new("lobshot_encoder");
  appsrc_ = gst_element_factory_make("appsrc", "source");
  appsink_ = gst_element_factory_make("appsink", "sink");
  GstElement * convert = gst_element_factory_make("videoconvert", "convert");
  GstElement * encoder = gst_element_factory_make("x265enc", "encoder");
  GstElement * parser = gst_element_factory_make("h265parse", "parser");

  if (!pipeline_ || !appsrc_ || !appsink_ || !convert || !encoder || !parser) {
    tools::logger()->error(
      "[Lobshot] GStreamer element creation failed. Please check x265enc plugin.");
    return;
  }

  GstCaps * caps = gst_caps_new_simple(
    "video/x-raw",
    "format", G_TYPE_STRING, "BGR",
    "width", G_TYPE_INT, output_size_,
    "height", G_TYPE_INT, output_size_,
    "framerate", GST_TYPE_FRACTION, output_fps_, 1,
    nullptr);
  g_object_set(
    G_OBJECT(appsrc_),
    "caps", caps,
    "stream-type", 0,
    "format", GST_FORMAT_TIME,
    "is-live", TRUE,
    "do-timestamp", TRUE,
    nullptr);
  gst_caps_unref(caps);

  const bool low_bitrate_mode = (target_bitrate_ <= 80);
  const int key_int = low_bitrate_mode ? std::max(output_fps_, 30) : std::max(output_fps_ / 2, 20);

  int speed_preset = 4;
  if (x265_preset_ == "ultrafast") speed_preset = 1;
  else if (x265_preset_ == "superfast") speed_preset = 2;
  else if (x265_preset_ == "veryfast") speed_preset = 3;
  else if (x265_preset_ == "faster") speed_preset = 4;
  else if (x265_preset_ == "fast") speed_preset = 5;
  else if (x265_preset_ == "medium") speed_preset = 6;
  else if (x265_preset_ == "slow") speed_preset = 7;
  else if (x265_preset_ == "slower") speed_preset = 8;
  else if (x265_preset_ == "veryslow") speed_preset = 9;
  else if (x265_preset_ == "placebo") speed_preset = 10;

  g_object_set(
    G_OBJECT(encoder),
    "bitrate", target_bitrate_,
    "key-int-max", key_int,
    "speed-preset", speed_preset,
    "tune", 4,
    "option-string",
    low_bitrate_mode ?
      "bframes=0:rc-lookahead=8:repeat-headers=1:aud=1:scenecut=0:aq-mode=2:aq-strength=1.0:info=0"
      : "bframes=0:rc-lookahead=10:repeat-headers=1:aud=1:scenecut=0:aq-mode=2:aq-strength=1.0:info=0",
    nullptr);

  g_object_set(
    G_OBJECT(parser),
    "config-interval", -1,
    "disable-passthrough", TRUE,
    nullptr);

  GstCaps * h265_caps = gst_caps_new_simple(
    "video/x-h265",
    "stream-format", G_TYPE_STRING, "byte-stream",
    "alignment", G_TYPE_STRING, "au",
    nullptr);
  g_object_set(
    G_OBJECT(appsink_),
    "caps", h265_caps,
    "max-buffers", 5,
    "drop", FALSE,
    "emit-signals", FALSE,
    "sync", FALSE,
    nullptr);
  gst_caps_unref(h265_caps);

  gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, encoder, parser, appsink_, nullptr);
  if (!gst_element_link_many(appsrc_, convert, encoder, parser, appsink_, nullptr)) {
    tools::logger()->error("[Lobshot] Failed to link GStreamer pipeline.");
    return;
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    tools::logger()->error("[Lobshot] Failed to start GStreamer pipeline.");
    return;
  }

  bus_ = gst_element_get_bus(pipeline_);
  gstreamer_ready_ = true;
}

void Lobshot::shutdown_gstreamer()
{
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }
  if (bus_) {
    gst_object_unref(bus_);
    bus_ = nullptr;
  }
  if (pipeline_) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  appsrc_ = nullptr;
  appsink_ = nullptr;
  gstreamer_ready_ = false;
}

void Lobshot::poll_gstreamer_bus()
{
  if (!bus_) {
    return;
  }

  while (true) {
    GstMessage * msg = gst_bus_pop(bus_);
    if (!msg) {
      break;
    }

    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError * err = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        tools::logger()->error(
          "[Lobshot] GStreamer error from {}: {} ({})",
          GST_OBJECT_NAME(msg->src),
          err ? err->message : "unknown",
          debug ? debug : "no debug");
        if (err) {
          g_error_free(err);
        }
        if (debug) {
          g_free(debug);
        }
        break;
      }
      case GST_MESSAGE_WARNING: {
        GError * err = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_warning(msg, &err, &debug);
        tools::logger()->warn(
          "[Lobshot] GStreamer warning from {}: {} ({})",
          GST_OBJECT_NAME(msg->src),
          err ? err->message : "unknown",
          debug ? debug : "no debug");
        if (err) {
          g_error_free(err);
        }
        if (debug) {
          g_free(debug);
        }
        break;
      }
      default:
        break;
    }
    gst_message_unref(msg);
  }
}

cv::Mat Lobshot::preprocess_image(const cv::Mat & input)
{
  int x = std::max(0, (input.cols - crop_size_) / 2);
  int y = std::max(0, (input.rows - crop_size_) / 2);
  int w = std::min(crop_size_, input.cols - x);
  int h = std::min(crop_size_, input.rows - y);

  cv::Mat cropped = input(cv::Rect(x, y, w, h));
  cv::Mat resized;
  cv::resize(
    cropped,
    resized,
    cv::Size(output_size_, output_size_),
    0,
    0,
    cv::INTER_LINEAR);

  cv::Mat working = resized;
  if (force_monochrome_) {
    cv::Mat gray;
    cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray, working, cv::COLOR_GRAY2BGR);
  }

  if (!static_simplify_) {
    return working;
  }

  cv::Mat gray;
  cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
  if (background_gray_f32_.empty()) {
    gray.convertTo(background_gray_f32_, CV_32F);
    return working;
  }

  cv::Mat bg_u8;
  cv::convertScaleAbs(background_gray_f32_, bg_u8);

  cv::Mat diff;
  cv::absdiff(gray, bg_u8, diff);

  cv::Mat motion_mask;
  cv::threshold(diff, motion_mask, motion_threshold_, 255, cv::THRESH_BINARY);

  if (motion_erode_px_ > 0) {
    if (motion_erode_kernel_.empty()) {
      const int kernel_size = 2 * motion_erode_px_ + 1;
      motion_erode_kernel_ = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    }
    cv::erode(motion_mask, motion_mask, motion_erode_kernel_);
  }
  if (motion_dilate_px_ > 0) {
    if (motion_dilate_kernel_.empty()) {
      const int kernel_size = 2 * motion_dilate_px_ + 1;
      motion_dilate_kernel_ = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    }
    cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_);
  }

  const double motion_ratio =
    static_cast<double>(cv::countNonZero(motion_mask)) / static_cast<double>(motion_mask.total());
  const bool suppress_trail = motion_ratio >= trail_disable_motion_ratio_;

  if (center_clear_size_ > 0) {
    const int clear_size = std::min({center_clear_size_, working.cols, working.rows});
    const int clear_x = std::max(0, working.cols / 2 - clear_size / 2);
    const int clear_y = std::max(0, working.rows / 2 - clear_size / 2);
    cv::rectangle(
      motion_mask,
      cv::Rect(clear_x, clear_y, clear_size, clear_size),
      cv::Scalar(255),
      cv::FILLED);
  }

  cv::Mat static_base = working.clone();
  if (!force_monochrome_ && target_bitrate_ <= 80) {
    cv::Mat gray_bg;
    cv::cvtColor(static_base, gray_bg, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray_bg, static_base, cv::COLOR_GRAY2BGR);
  }

  cv::Mat blurred_static;
  cv::GaussianBlur(
    static_base,
    blurred_static,
    cv::Size(),
    bg_blur_sigma_,
    bg_blur_sigma_);

  cv::Mat focused = blurred_static.clone();
  working.copyTo(focused, motion_mask);

  if (motion_trail_frames_ > 0) {
    motion_mask_history_.push_back(motion_mask.clone());
    trail_frame_history_.push_back(working.clone());
    const size_t max_history = static_cast<size_t>(motion_trail_frames_ + 1);
    while (motion_mask_history_.size() > max_history) {
      motion_mask_history_.pop_front();
    }
    while (trail_frame_history_.size() > max_history) {
      trail_frame_history_.pop_front();
    }

    if (!suppress_trail && motion_mask_history_.size() > 1 &&
      motion_mask_history_.size() == trail_frame_history_.size())
    {
      cv::Mat trail_mask = motion_mask.clone();
      cv::Mat trail_img = working.clone();
      for (size_t i = 0; i + 1 < motion_mask_history_.size(); ++i) {
        cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);
        cv::max(trail_img, trail_frame_history_[i], trail_img);
      }
      trail_img.copyTo(focused, trail_mask);
    }
  } else {
    motion_mask_history_.clear();
    trail_frame_history_.clear();
  }

  cv::accumulateWeighted(gray, background_gray_f32_, bg_update_alpha_);
  return focused;
}

void Lobshot::draw_overlay(cv::Mat & frame) const
{
  if (frame.empty()) {
    return;
  }

  const int width = frame.cols;
  const int height = frame.rows;
  const int cx = std::clamp(width / 2 + crosshair_offset_x_, 0, width - 1);
  const int cy = std::clamp(height / 2 + crosshair_offset_y_, 0, height - 1);

  const cv::Scalar crosshair_color(230, 190, 235);
  cv::line(frame, {0, cy}, {width - 1, cy}, crosshair_color, crosshair_width_, cv::LINE_AA);
  cv::line(frame, {cx, 0}, {cx, height - 1}, crosshair_color, crosshair_width_, cv::LINE_AA);

  if (center_circle_radius_ > 0) {
    const cv::Scalar center_color(170, 255, 170);
    cv::circle(
      frame,
      {width / 2, height / 2},
      center_circle_radius_,
      center_color,
      center_circle_width_,
      cv::LINE_AA);
  }
}

void Lobshot::push_frame_to_gstreamer(const cv::Mat & frame)
{
  if (!gstreamer_ready_ || !appsrc_ || frame.empty()) {
    return;
  }

  const cv::Mat contiguous = frame.isContinuous() ? frame : frame.clone();
  const size_t size = contiguous.total() * contiguous.elemSize();
  GstBuffer * buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  if (!buffer) {
    return;
  }

  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    std::memcpy(map.data, contiguous.data, size);
    gst_buffer_unmap(buffer, &map);

    GstFlowReturn ret = GST_FLOW_OK;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    if (ret != GST_FLOW_OK) {
      tools::logger()->warn("[Lobshot] Failed to push GstBuffer: {}", static_cast<int>(ret));
    }
  }
  gst_buffer_unref(buffer);
}

void Lobshot::pull_stream_and_packetize()
{
  if (!gstreamer_ready_ || !appsink_) {
    return;
  }

  const size_t max_backlog_bytes = static_cast<size_t>(bandwidth_limit_kbytes_ * 1000.0 * max_tx_delay_s_);
  bool pulled_any_sample = false;

  while (true) {
    GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 0);
    if (!sample) {
      break;
    }
    pulled_any_sample = true;

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
      gst_sample_unref(sample);
      continue;
    }

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      std::lock_guard<std::mutex> lock(buffer_mutex_);

      std::vector<uint8_t> frame_data(map.size);
      std::memcpy(frame_data.data(), map.data, map.size);
      frame_queue_.push_back(std::move(frame_data));
      encoded_au_count_++;

      size_t total_bytes = current_send_frame_.size();
      for (const auto & queued_frame : frame_queue_) {
        total_bytes += queued_frame.size();
      }
      while (total_bytes > max_backlog_bytes && !frame_queue_.empty()) {
        const size_t dropped = frame_queue_.front().size();
        frame_queue_.pop_front();
        total_bytes -= dropped;
        dropped_bytes_ += dropped;
        dropped_events_++;
        if (dropped_events_ % 20 == 1) {
          tools::logger()->warn(
            "[Lobshot] Backlog clipped: dropped={}B backlog={}B total_dropped={}B",
            dropped,
            total_bytes,
            dropped_bytes_);
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (now - last_telemetry_time_ > 1s) {
        tools::logger()->info(
          "[Lobshot] Encoder stats: aus={} backlog={}B dropped={}B",
          encoded_au_count_,
          total_bytes,
          dropped_bytes_);
        last_telemetry_time_ = now;
      }

      gst_buffer_unmap(buffer, &map);
    }
    gst_sample_unref(sample);
  }

  if (!pulled_any_sample) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_appsink_empty_log_time_ > 1s) {
      size_t queued_bytes = 0;
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        queued_bytes += current_send_frame_.size();
        for (const auto & queued_frame : frame_queue_) {
          queued_bytes += queued_frame.size();
        }
      }
      tools::logger()->info(
        "[Lobshot] Encoder idle: aus={} queued_frames={} queued_bytes={}B",
        encoded_au_count_,
        frame_queue_.size(),
        queued_bytes);
      last_appsink_empty_log_time_ = now;
    }
  }
}

void Lobshot::send_loop()
{
  const auto period = std::chrono::microseconds(1000000 / tx_rate_hz_);
  while (!quit_) {
    const auto start = std::chrono::steady_clock::now();
    send_packet_once();
    std::this_thread::sleep_until(start + period);
  }
}

void Lobshot::send_packet_once()
{
  if (!enabled_) {
    return;
  }

  std::shared_ptr<io::LobshotComm> comm;
  {
    std::lock_guard<std::mutex> lock(comm_mutex_);
    comm = comm_;
  }
  if (!comm) {
    return;
  }

  constexpr size_t header_bytes = 8;
  constexpr size_t payload_bytes = 292;
  constexpr size_t packet_bytes = header_bytes + payload_bytes;

  std::array<uint8_t, packet_bytes> raw_packet{};
  bool has_payload = false;

  if (fixed_test_payload_mode_) {
    const uint16_t frame_no = current_frame_no_++;
    const uint16_t frag_no = 0;
    constexpr uint32_t synthetic_size = 64;
    raw_packet[0] = frame_no & 0xFF;
    raw_packet[1] = (frame_no >> 8) & 0xFF;
    raw_packet[2] = frag_no & 0xFF;
    raw_packet[3] = (frag_no >> 8) & 0xFF;
    raw_packet[4] = synthetic_size & 0xFF;
    raw_packet[5] = (synthetic_size >> 8) & 0xFF;
    raw_packet[6] = (synthetic_size >> 16) & 0xFF;
    raw_packet[7] = (synthetic_size >> 24) & 0xFF;

    std::ostringstream oss;
    oss << "RM0310-TEST frame=" << frame_no << " tick=" << synthetic_sequence_id_++;
    const auto text = oss.str();
    const size_t copy_size = std::min(text.size(), payload_bytes);
    std::memcpy(raw_packet.data() + header_bytes, text.data(), copy_size);
    has_payload = true;
  } else {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (current_send_frame_.empty() && !frame_queue_.empty()) {
      current_send_frame_ = std::move(frame_queue_.front());
      frame_queue_.pop_front();
      current_send_offset_ = 0;
      current_frag_no_ = 0;
    }

    if (!current_send_frame_.empty()) {
      const size_t remaining = current_send_frame_.size() - current_send_offset_;
      const size_t copy_size = std::min(payload_bytes, remaining);

      raw_packet[0] = current_frame_no_ & 0xFF;
      raw_packet[1] = (current_frame_no_ >> 8) & 0xFF;
      raw_packet[2] = current_frag_no_ & 0xFF;
      raw_packet[3] = (current_frag_no_ >> 8) & 0xFF;

      const uint32_t total_bytes = static_cast<uint32_t>(current_send_frame_.size());
      raw_packet[4] = total_bytes & 0xFF;
      raw_packet[5] = (total_bytes >> 8) & 0xFF;
      raw_packet[6] = (total_bytes >> 16) & 0xFF;
      raw_packet[7] = (total_bytes >> 24) & 0xFF;
      std::memcpy(
        raw_packet.data() + header_bytes,
        current_send_frame_.data() + current_send_offset_,
        copy_size);

      current_send_offset_ += copy_size;
      current_frag_no_++;
      has_payload = true;

      if (current_send_offset_ >= current_send_frame_.size()) {
        current_send_frame_.clear();
        current_send_offset_ = 0;
        current_frag_no_ = 0;
        current_frame_no_++;
      }
    }
  }

  if (!has_payload) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_idle_log_time_ > 1s) {
      tools::logger()->info("[Lobshot] Send idle: no packet ready.");
      last_idle_log_time_ = now;
    }
    return;
  }

  comm->send(raw_packet.data(), raw_packet.size());
}

void Lobshot::reset_runtime_state()
{
  has_last_encode_time_ = false;
  last_encode_time_ = {};
  last_telemetry_time_ = {};
  last_idle_log_time_ = {};
  last_appsink_empty_log_time_ = {};
  background_gray_f32_.release();
  motion_mask_history_.clear();
  trail_frame_history_.clear();

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  clear_backlog_locked();
}

void Lobshot::clear_backlog_locked()
{
  frame_queue_.clear();
  current_send_frame_.clear();
  current_frame_no_ = 0;
  current_frag_no_ = 0;
  current_send_offset_ = 0;
  synthetic_sequence_id_ = 0;
}

}  // namespace lobshot
