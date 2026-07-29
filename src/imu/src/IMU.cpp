#include "IMU.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <array>
#include <cstring>
#include <rapidjson/document.h>

namespace {
float number(const rapidjson::Value& obj, const char* key, float fallback = 0.0F) {
  if (!obj.HasMember(key)) return fallback;
  const auto& value = obj[key];
  if (value.IsNumber()) return value.GetFloat();
  if (value.IsString()) { try { return std::stof(value.GetString()); } catch (...) {} }
  return fallback;
}
uint32_t uint_number(const rapidjson::Value& obj, const char* key) { return obj.HasMember(key) && obj[key].IsUint() ? obj[key].GetUint() : 0U; }
std::string text(const rapidjson::Value& obj, const char* key) { return obj.HasMember(key) && obj[key].IsString() ? obj[key].GetString() : ""; }
}

Imu::Imu() : Node("imu") {
  multicast_group_ = declare_parameter<std::string>("multicast_group", "230.168.50.16");
  port_ = declare_parameter<int>("port", 20000);
  ownship_pub_ = create_publisher<message_interface::msg::Ownship>("/ownship", 10);
  env_pub_ = create_publisher<message_interface::msg::EnvData>("/envdata", 10);
  if (open_socket()) timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&Imu::poll, this));
}
Imu::~Imu() { if (socket_ >= 0) close(socket_); }
bool Imu::open_socket() {
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ < 0) { RCLCPP_ERROR(get_logger(), "Unable to create UDP socket"); return false; }
  int reuse = 1; setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in address{}; address.sin_family = AF_INET; address.sin_port = htons(static_cast<uint16_t>(port_)); address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) { RCLCPP_ERROR(get_logger(), "Cannot bind UDP port %d", port_); close(socket_); socket_ = -1; return false; }
  ip_mreq request{}; request.imr_multiaddr.s_addr = inet_addr(multicast_group_.c_str()); request.imr_interface.s_addr = htonl(INADDR_ANY);
  if (request.imr_multiaddr.s_addr == INADDR_NONE || setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &request, sizeof(request)) < 0) { RCLCPP_ERROR(get_logger(), "Cannot join multicast group %s", multicast_group_.c_str()); close(socket_); socket_ = -1; return false; }
  fcntl(socket_, F_SETFL, fcntl(socket_, F_GETFL, 0) | O_NONBLOCK);
  return true;
}
void Imu::poll() { std::array<char, 8192> buffer{}; for (;;) { const auto len = recv(socket_, buffer.data(), buffer.size(), 0); if (len <= 0) break; parse(buffer.data(), static_cast<size_t>(len)); } }
void Imu::parse(const char* data, size_t length) {
  rapidjson::Document doc; doc.Parse(data, length);
  if (doc.HasParseError() || !doc.IsObject() || !doc.HasMember("DataType") || !doc["DataType"].IsString() || !doc.HasMember("content") || !doc["content"].IsObject()) { RCLCPP_WARN(get_logger(), "Discarded malformed IMU JSON datagram"); return; }
  const auto stamp = now(); const auto type = text(doc, "DataType"); const auto& c = doc["content"];
  if (type == "OwnShip") {
    message_interface::msg::Ownship msg; msg.header.stamp = stamp; msg.data_type = type; msg.date_time = text(doc, "DateTime"); msg.mmsi = uint_number(doc, "MMSI");
    msg.cog=number(c,"cog"); msg.draft=number(c,"draft"); msg.ground_mile_all=number(c,"groundMileAll"); msg.ground_mile_clean=number(c,"groundMileClean"); msg.head=number(c,"head"); msg.head_ratio=number(c,"headRatio"); msg.latitude=number(c,"lat"); msg.longitude=number(c,"lon"); msg.pitch=number(c,"pitch"); msg.pitch_ratio=number(c,"pitchRatio"); msg.roll=number(c,"roll"); msg.roll_ratio=number(c,"rollRatio"); msg.sea_depth=number(c,"seaDepth"); msg.sog=number(c,"sog"); msg.sog_x=number(c,"sogX"); msg.sog_y=number(c,"sogY"); msg.stw=number(c,"stw"); msg.stw_x=number(c,"stwX"); msg.stw_y=number(c,"stwY"); msg.water_mile_all=number(c,"waterMileAll"); msg.water_mile_clean=number(c,"waterMileClean"); ownship_pub_->publish(msg);
  } else if (type == "EnvirData") {
    message_interface::msg::EnvData msg; msg.header.stamp=stamp; msg.data_type=type; msg.date_time=text(doc,"DateTime"); msg.mmsi=uint_number(doc,"MMSI"); msg.current_ang=number(c,"CurrentAng"); msg.current_spd=number(c,"CurrentSpd"); msg.fog_mod=static_cast<uint8_t>(number(c,"FogMod")); msg.rain_mod=static_cast<uint8_t>(number(c,"RainMod")); msg.snow_mod=static_cast<uint8_t>(number(c,"SnowMod")); msg.surge_ang=number(c,"SurgeAng"); msg.surge_height=number(c,"SurgeHeight"); msg.surge_period=number(c,"SurgePeirod"); msg.water_depth=number(c,"WaterDeep"); msg.wave_height_sea=number(c,"WaveHeightSea"); msg.wave_height_wind=number(c,"WaveHeightWind"); msg.wave_period=number(c,"WavePeirod"); msg.wind_ang_a=number(c,"WindAngA"); msg.wind_ang_r=number(c,"WindAngR"); msg.wind_spd_a=number(c,"WindSpdA"); msg.wind_spd_r=number(c,"WindSpdR"); env_pub_->publish(msg);
  }
}
int main(int argc, char** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<Imu>()); rclcpp::shutdown(); }
