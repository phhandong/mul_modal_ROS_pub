#include "hik_camera.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <opencv2/imgcodecs.hpp>

namespace { WORD bcd(float degrees) { const int value = static_cast<int>(std::lround(std::clamp(degrees, 0.0F, 359.9F) * 10)); return static_cast<WORD>(((value / 1000) << 12) | (((value / 100) % 10) << 8) | (((value / 10) % 10) << 4) | (value % 10)); } }

HikCamera::HikCamera() : Node("hik_camera") {
  profile_ = declare_parameter<std::string>("profile", "mono"); ip_ = declare_parameter<std::string>("ip", "0.0.0.0"); username_ = declare_parameter<std::string>("username", "admin"); password_ = declare_parameter<std::string>("password", ""); password_env_ = declare_parameter<std::string>("password_env", profile_ == "bispectral" ? "HIK_BISPECTRAL_PASSWORD" : "HIK_MONO_PASSWORD"); port_ = static_cast<int>(declare_parameter<int>("port", 8000)); mono_channel_ = static_cast<int>(declare_parameter<int>("mono_channel", 1)); visible_channel_ = static_cast<int>(declare_parameter<int>("visible_channel", 1)); thermal_channel_ = static_cast<int>(declare_parameter<int>("thermal_channel", 2)); jpeg_quality_ = static_cast<int>(std::clamp<int64_t>(declare_parameter<int>("jpeg_quality", 90), 0, 100));
  const auto qos = rclcpp::SensorDataQoS();
  mono_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/hik_mono/image/compressed", qos);
  visible_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/hik_bispectral/visible/image/compressed", qos);
  thermal_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/hik_bispectral/thermal/image/compressed", qos);
  temperature_pub_ = create_publisher<sensor_msgs::msg::Image>("/hik_bispectral/temperature/image_raw", qos);
  p2p_pub_ = create_publisher<message_interface::msg::P2PData>("/p2p_data", 10);
  ptz_sub_ = create_subscription<message_interface::msg::PtzCtrl>("/ptz_ctrl", 10, std::bind(&HikCamera::on_ptz, this, std::placeholders::_1));
  NET_DVR_Init(); NET_DVR_SetConnectTime(2000, 1); NET_DVR_SetReconnect(5000, TRUE);
  const double rate = declare_parameter<double>("capture_rate_hz", profile_ == "bispectral" ? 5.0 : 10.0);
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / std::max(rate, 0.2))), std::bind(&HikCamera::capture, this));
}
HikCamera::~HikCamera() { if (user_id_ >= 0) NET_DVR_Logout(user_id_); NET_DVR_Cleanup(); }
bool HikCamera::login() {
  const char* password = password_.empty() ? std::getenv(password_env_.c_str()) : password_.c_str();
  if (password == nullptr || *password == '\0') { RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Set the password parameter or environment variable %s", password_env_.c_str()); return false; }
  NET_DVR_USER_LOGIN_INFO info{}; NET_DVR_DEVICEINFO_V40 device{}; info.wPort = static_cast<WORD>(port_); info.bUseAsynLogin = false;
  std::snprintf(reinterpret_cast<char*>(info.sDeviceAddress), sizeof(info.sDeviceAddress), "%s", ip_.c_str()); std::snprintf(reinterpret_cast<char*>(info.sUserName), sizeof(info.sUserName), "%s", username_.c_str()); std::snprintf(reinterpret_cast<char*>(info.sPassword), sizeof(info.sPassword), "%s", password);
  user_id_ = NET_DVR_Login_V40(&info, &device); if (user_id_ < 0) { RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Hikvision login failed: SDK error %u", NET_DVR_GetLastError()); return false; }
  RCLCPP_INFO(get_logger(), "Connected to Hikvision camera at %s", ip_.c_str()); return true;
}
void HikCamera::publish_jpeg(const std::vector<char>& bytes, const std::string& frame, const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& publisher) { sensor_msgs::msg::CompressedImage msg; msg.header.stamp = now(); msg.header.frame_id = frame; msg.format = "jpeg"; const auto decoded=cv::imdecode(bytes, cv::IMREAD_COLOR); if (!decoded.empty()) cv::imencode(".jpg", decoded, msg.data, {cv::IMWRITE_JPEG_QUALITY,jpeg_quality_}); else msg.data.assign(bytes.begin(), bytes.end()); publisher->publish(std::move(msg)); }
void HikCamera::capture() {
  if (user_id_ < 0 && !login()) return;
  if (profile_ != "bispectral") { NET_DVR_JPEGPARA para{}; para.wPicQuality=0; DWORD size{}; if (!NET_DVR_CaptureJPEGPicture_NEW(user_id_, mono_channel_, &para, jpeg_buffer_.data(), jpeg_buffer_.size(), &size)) { RCLCPP_WARN(get_logger(), "JPEG capture failed: SDK error %u", NET_DVR_GetLastError()); return; } jpeg_buffer_.resize(size); publish_jpeg(jpeg_buffer_, "hik_mono", mono_pub_); jpeg_buffer_.resize(16*1024*1024); return; }
  NET_DVR_JPEGPICTURE_WITH_APPENDDATA data{}; data.dwSize=sizeof(data); data.dwChannel=thermal_channel_; data.pJpegPicBuff=jpeg_buffer_.data(); data.pVisiblePicBuff=visible_buffer_.data(); data.pP2PDataBuff=p2p_buffer_.data(); if (!NET_DVR_CaptureJPEGPicture_WithAppendData(user_id_, thermal_channel_, &data)) { RCLCPP_WARN(get_logger(), "Bi-spectrum capture failed: SDK error %u", NET_DVR_GetLastError()); return; }
  if (data.dwJpegPicLen) { std::vector<char> image(jpeg_buffer_.begin(), jpeg_buffer_.begin()+data.dwJpegPicLen); publish_jpeg(image,"hik_bispectral_thermal",thermal_pub_); }
  if (data.dwVisiblePicLen) { std::vector<char> image(visible_buffer_.begin(), visible_buffer_.begin()+data.dwVisiblePicLen); publish_jpeg(image,"hik_bispectral_visible",visible_pub_); }
  if (data.dwP2PDataLen) { message_interface::msg::P2PData raw; raw.header.stamp=now(); raw.header.frame_id="hik_bispectral_thermal"; raw.width=data.dwJpegPicWidth; raw.height=data.dwJpegPicHeight; raw.data.assign(p2p_buffer_.begin(), p2p_buffer_.begin()+data.dwP2PDataLen); p2p_pub_->publish(raw); if (data.dwP2PDataLen % sizeof(float) == 0 && data.dwJpegPicWidth > 0) { const size_t count=data.dwP2PDataLen/sizeof(float); const uint32_t width=data.dwJpegPicWidth; if (count % width == 0) { sensor_msgs::msg::Image image; image.header=raw.header; image.height=count/width; image.width=width; image.encoding="32FC1"; image.is_bigendian=false; image.step=width*sizeof(float); image.data=raw.data; temperature_pub_->publish(image); } } }
}
void HikCamera::on_ptz(const message_interface::msg::PtzCtrl::SharedPtr msg) { if (user_id_ < 0 || profile_ != "bispectral") return; NET_DVR_PTZPOS position{}; position.wAction=1; position.wPanPos=bcd(msg->pan_deg); position.wTiltPos=bcd(std::clamp(msg->tilt_deg,0.0F,90.0F)); NET_DVR_PTZPOS current{}; DWORD returned{}; if (NET_DVR_GetDVRConfig(user_id_, NET_DVR_GET_PTZPOS, thermal_channel_, &current, sizeof(current), &returned)) position.wZoomPos=static_cast<WORD>(std::clamp(msg->zoom,0,65535)); if (!NET_DVR_SetDVRConfig(user_id_, NET_DVR_SET_PTZPOS, thermal_channel_, &position, sizeof(position))) RCLCPP_WARN(get_logger(), "PTZ update failed: SDK error %u", NET_DVR_GetLastError()); }
int main(int argc, char** argv) { rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<HikCamera>()); rclcpp::shutdown(); }
