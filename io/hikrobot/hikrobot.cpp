#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
  namespace
  {

  std::string to_string_or_empty(const unsigned char * raw)
  {
    if (!raw) {
      return "";
    }
    return std::string(reinterpret_cast<const char *>(raw));
  }

  }  // namespace

  HikRobot::HikRobot(
    double exposure_ms, double gain, const std::string & vid_pid,
    const std::string & user_id, const std::string & serial_number)
      : exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false), vid_(-1), pid_(-1),
        user_id_(user_id), serial_number_(serial_number)
  {
    set_vid_pid(vid_pid);
    if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

    daemon_thread_ = std::thread{[this]
                                 {
                                   tools::logger()->info("HikRobot's daemon thread started.");

                                   capture_start();

                                   while (!daemon_quit_)
                                   {
                                     std::this_thread::sleep_for(100ms);

                                     if (capturing_)
                                       continue;

                                     capture_stop();
                                     reset_usb();
                                     capture_start();
                                   }

                                   capture_stop();

                                   tools::logger()->info("HikRobot's daemon thread stopped.");
                                 }};
  }

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  MV_CC_DEVICE_INFO * matched_device = nullptr;
  for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    auto * device_info = device_list.pDeviceInfo[i];
    if (match_device_info(device_info)) {
      matched_device = device_info;
      break;
    }
  }

  if (!matched_device) {
    tools::logger()->warn(
      "No matched HikRobot camera found. vid_pid={}, user_id={}, serial_number={}",
      fmt::format("{:04x}:{:04x}", vid_, pid_),
      user_id_,
      serial_number_);
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, matched_device);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    release_handle();
    return;
  }

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  MV_CC_SetFrameRate(handle_, 150);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    release_handle();
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw;
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

      cvt_param.nWidth = raw.stFrameInfo.nWidth;
      cvt_param.nHeight = raw.stFrameInfo.nHeight;

      cvt_param.pSrcData = raw.pBufAddr;
      cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
      cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

      cvt_param.pDstBuffer = img.data;
      cvt_param.nDstBufferSize = img.total() * img.elemSize();
      cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

      // ret = MV_CC_ConvertPixelType(handle_, &cvt_param);
      const auto & frame_info = raw.stFrameInfo;
      auto pixel_type = frame_info.enPixelType;
      cv::Mat dst_image;
      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB},
        {PixelType_Gvsp_BayerGB10, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG10, cv::COLOR_BayerBG2RGB},
        {PixelType_Gvsp_BayerGR10, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG10, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_Mono8, cv::COLOR_GRAY2BGR},
        {PixelType_Gvsp_Mono10, cv::COLOR_GRAY2BGR},
        {PixelType_Gvsp_Mono12, cv::COLOR_GRAY2BGR}};
      
      auto it = type_map.find(pixel_type);
      if (it != type_map.end()) {
        cv::cvtColor(img, dst_image, it->second);
        img = dst_image;
      } else if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
        // RGB8_Packed 已经是 RGB 格式，转为 BGR
        cv::cvtColor(img, dst_image, cv::COLOR_RGB2BGR);
        img = dst_image;
      } else {
        tools::logger()->warn("Unsupported pixel type: {:#x}, trying BGR8 conversion", pixel_type);
        cv::cvtColor(img, dst_image, cv::COLOR_BayerBG2BGR);
        img = dst_image;
      }

      queue_.push({img, timestamp});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  if (!handle_) {
    capturing_ = false;
    return;
  }

  unsigned int ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
  }

  release_handle();
  capturing_ = false;
}

void HikRobot::release_handle()
{
  if (!handle_) {
    return;
  }

  const unsigned int ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
  }
  handle_ = nullptr;
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

bool HikRobot::match_device_info(const MV_CC_DEVICE_INFO * device_info) const
{
  if (!device_info) {
    return false;
  }

  if (device_info->nTLayerType != MV_USB_DEVICE) {
    return false;
  }

  const auto & info = device_info->SpecialInfo.stUsb3VInfo;
  if (vid_ != -1 && info.idVendor != static_cast<unsigned short>(vid_)) {
    return false;
  }
  if (pid_ != -1 && info.idProduct != static_cast<unsigned short>(pid_)) {
    return false;
  }

  const auto device_user_id = to_string_or_empty(info.chUserDefinedName);
  const auto device_serial = to_string_or_empty(info.chSerialNumber);

  if (!user_id_.empty() && user_id_ != device_user_id) {
    return false;
  }
  if (!serial_number_.empty() && serial_number_ != device_serial) {
    return false;
  }

  tools::logger()->info(
    "Matched HikRobot camera: user_id='{}', serial='{}', device_no={}",
    device_user_id,
    device_serial,
    info.nDeviceNumber);
  return true;
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
