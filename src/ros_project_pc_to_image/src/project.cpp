#include <fstream>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct Camera { cv::Mat k, rt, distortion; std::string image_topic; sensor_msgs::msg::CompressedImage::ConstSharedPtr image; rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher; };
class Projector final : public rclcpp::Node {
 public:
  Projector() : Node("project_pc_to_image") {
    const auto package=ament_index_cpp::get_package_share_directory("ros_project_pc_to_image");
    const auto topics=declare_parameter<std::vector<std::string>>("image_topics", {"/compressedimg1","/compressedimg2","/compressedimg3","/compressedimg4"});
    for(size_t i=0;i<topics.size() && i<4;i++) { Camera camera; camera.image_topic=topics[i]; load(package+"/cfg/camera_"+std::to_string(i+1)+".txt",camera); camera.publisher=create_publisher<sensor_msgs::msg::Image>("/project_pc_image"+std::to_string(i+1),10); cameras_.push_back(std::move(camera)); }
    for(size_t i=0;i<cameras_.size();i++) subscriptions_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(cameras_[i].image_topic,rclcpp::SensorDataQoS(),[this,i](sensor_msgs::msg::CompressedImage::ConstSharedPtr message){cameras_[i].image=std::move(message);}));
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(declare_parameter<std::string>("cloud_topic","/pointcloud"),rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){on_cloud(*msg);});
  }
 private:
  static void load(const std::string& path, Camera& camera) { std::ifstream file(path); std::string ignored; file>>ignored>>ignored; camera.k=cv::Mat::zeros(3,4,CV_64F); camera.rt=cv::Mat::eye(4,4,CV_64F); camera.distortion=cv::Mat::zeros(5,1,CV_64F); for(int r=0;r<3;r++)for(int c=0;c<4;c++)file>>camera.k.at<double>(r,c); for(int r=0;r<4;r++)for(int c=0;c<4;c++)file>>camera.rt.at<double>(r,c); for(int i=0;i<5;i++)file>>camera.distortion.at<double>(i); }
  void on_cloud(const sensor_msgs::msg::PointCloud2& message) { pcl::PointCloud<pcl::PointXYZI> cloud; pcl::fromROSMsg(message,cloud); for(auto& camera:cameras_) if(camera.image) render(cloud,camera); }
  void render(const pcl::PointCloud<pcl::PointXYZI>& cloud, Camera& camera) { try { auto cv_image=cv_bridge::toCvCopy(camera.image,"bgr8"); cv::Mat output=cv_image->image.clone(); for(const auto& point:cloud.points) { if(!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(point.z)) continue; cv::Mat x=(cv::Mat_<double>(4,1)<<point.x,point.y,point.z,1.0); cv::Mat y=camera.k*camera.rt*x; if(y.at<double>(2)<=0.001) continue; const int u=static_cast<int>(y.at<double>(0)/y.at<double>(2)), v=static_cast<int>(y.at<double>(1)/y.at<double>(2)); if(u<0||v<0||u>=output.cols||v>=output.rows) continue; cv::circle(output,{u,v},1,{0,255,0},-1); } cv_image->image=output; cv_image->header=camera.image->header; camera.publisher->publish(*cv_image->toImageMsg()); } catch(const cv_bridge::Exception& error) { RCLCPP_WARN(get_logger(),"Image decode failed: %s",error.what()); } }
  std::vector<Camera> cameras_; std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> subscriptions_; rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Projector>());rclcpp::shutdown();}
