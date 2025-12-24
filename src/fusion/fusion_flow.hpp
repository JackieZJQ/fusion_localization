#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "models/converter/cloud_convert.hpp"
#include "models/converter/utm_convert.hpp"
#include "sensor_data/imu_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/cloud_data.hpp"
#include "fusion/fusion.hpp"

namespace localization {
class FusionFlow {
public:
  FusionFlow(const rclcpp::Node::SharedPtr& node);

  void InitIO();

  using Ptr = std::shared_ptr<localization::FusionFlow>;

private:
  //传感器回调函数
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr);
  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg_ptr);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

  CloudConvert::Ptr cloud_converter_ptr_;
  std::shared_ptr<Fusion> fusion_ptr_;
  rclcpp::Node::SharedPtr node_;

  //todo
  //发布rviz信息(*^__^*) 嘻嘻…
};
} // namesapce localization

