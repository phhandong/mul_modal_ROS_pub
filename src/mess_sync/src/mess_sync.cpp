#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MessageSynchronizer final : public rclcpp::Node {
 public:
  MessageSynchronizer() : Node("mess_sync") {
    camera_topics_ = declare_parameter<std::vector<std::string>>(
      "camera_topics", {"/hik_mono/image/compressed", "/hik_bispectral/visible/image/compressed", "/hik_bispectral/thermal/image/compressed"});
    output_topics_ = declare_parameter<std::vector<std::string>>(
      "output_topics", {"/compressedimg1", "/compressedimg2", "/compressedimg3"});
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/rslidar_points");
    output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/pointcloud");
    max_delta_ns_ = static_cast<int64_t>(declare_parameter<double>("max_delta_sec", 0.10) * 1e9);
    if (camera_topics_.empty() || camera_topics_.size() > 4 || camera_topics_.size() != output_topics_.size()) {
      throw std::runtime_error("camera_topics and output_topics must contain the same 1..4 entries");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    latest_images_.resize(camera_topics_.size());
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      image_publishers_.push_back(create_publisher<sensor_msgs::msg::CompressedImage>(output_topics_[i], sensor_qos));
      image_subscriptions_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(
        camera_topics_[i], sensor_qos, [this, i](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) { latest_images_[i] = std::move(msg); }));
    }
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, sensor_qos);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, sensor_qos, std::bind(&MessageSynchronizer::on_cloud, this, std::placeholders::_1));
  }

 private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
    for (const auto& image : latest_images_) {
      if (!image || std::llabs((rclcpp::Time(image->header.stamp) - rclcpp::Time(cloud_msg->header.stamp)).nanoseconds()) > max_delta_ns_) return;
    }
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud, cloud, indices);
    sensor_msgs::msg::PointCloud2 filtered;
    pcl::toROSMsg(cloud, filtered);
    filtered.header = cloud_msg->header;
    cloud_publisher_->publish(filtered);
    for (size_t i = 0; i < latest_images_.size(); ++i) image_publishers_[i]->publish(*latest_images_[i]);
  }

  std::vector<std::string> camera_topics_, output_topics_;
  std::string cloud_topic_, output_cloud_topic_;
  int64_t max_delta_ns_{};
  std::vector<sensor_msgs::msg::CompressedImage::ConstSharedPtr> latest_images_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> image_subscriptions_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> image_publishers_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

int main(int argc, char** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<MessageSynchronizer>()); rclcpp::shutdown(); }
