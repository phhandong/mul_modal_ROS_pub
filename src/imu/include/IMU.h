#pragma once
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <message_interface/msg/env_data.hpp>
#include <message_interface/msg/ownship.hpp>

class Imu final : public rclcpp::Node {
 public:
  Imu();
  ~Imu() override;
 private:
  bool open_socket();
  void poll();
  void parse(const char* data, size_t length);
  int socket_{-1};
  std::string multicast_group_;
  int port_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<message_interface::msg::Ownship>::SharedPtr ownship_pub_;
  rclcpp::Publisher<message_interface::msg::EnvData>::SharedPtr env_pub_;
};
