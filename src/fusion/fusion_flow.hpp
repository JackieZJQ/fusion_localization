/**
 * ************************************************************************
 * 
 * @file fusion_flow.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 融合定位ROS输入输出IO
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "models/converter/cloud_convert.hpp"
#include "models/converter/utm_convert.hpp"
#include "sensor_data/imu_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/cloud_data.hpp"
#include "fusion/fusion.hpp"

namespace localization {
class FusionFlow {
public:
  FusionFlow() = delete;
  FusionFlow(const rclcpp::Node::SharedPtr& node);

  ~FusionFlow() = default;

  void InitRosInterfaces();

  using Ptr = std::shared_ptr<localization::FusionFlow>;

private:
  //传感器回调函数
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr);
  void GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg_ptr);
  void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr);

  //发布rviz消息
  void PublishStaticPointcloudMap();
  void PublishUndistortScan();
  void PublishRegistrationTf();
  void PublishRegistrationOdom();

  // 订阅传感器数据
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

  // 发布定位数据
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr undistort_scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pointcloud_map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fusion_localization_publisher_;

  CloudConvert::Ptr cloud_converter_ptr_;
  std::shared_ptr<Fusion> fusion_ptr_;
  rclcpp::Node::SharedPtr node_;

  // 回调组
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_; // GNSS/Cloud 回调组，串行
};
} // namesapce localization

