#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_interface/msg/p2_p_data.hpp>
#include <message_interface/msg/ptz_ctrl.hpp>
#include <vector>
#include "hik_sdk/HCNetSDK.h"

class HikCamera final : public rclcpp::Node {
 public:
  HikCamera();
  ~HikCamera() override;
 private:
  bool login();
  void capture();
  void publish_jpeg(const std::vector<char>& bytes, const std::string& topic_frame, const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& publisher);
  void on_ptz(const message_interface::msg::PtzCtrl::SharedPtr msg);
  LONG user_id_{-1};
  std::string ip_, username_, password_, password_env_, profile_;
  int port_{8000}, mono_channel_{1}, visible_channel_{1}, thermal_channel_{2}, jpeg_quality_{90};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr mono_pub_, visible_pub_, thermal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr temperature_pub_;
  rclcpp::Publisher<message_interface::msg::P2PData>::SharedPtr p2p_pub_;
  rclcpp::Subscription<message_interface::msg::PtzCtrl>::SharedPtr ptz_sub_;
  std::vector<char> jpeg_buffer_ = std::vector<char>(16 * 1024 * 1024);
  std::vector<char> visible_buffer_ = std::vector<char>(16 * 1024 * 1024);
  std::vector<char> p2p_buffer_ = std::vector<char>(16 * 1024 * 1024);
};
