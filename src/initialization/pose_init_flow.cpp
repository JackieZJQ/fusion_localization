/**
 * ************************************************************************
 *
 * @file initialization_flow.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 初始化ROS收发结点
 *
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include "initialization/pose_init_flow.hpp"

namespace localization {

PoseInitFlow::PoseInitFlow(const rclcpp::Node::SharedPtr& node)
  : node_(node) {
  
  InitRosInterfaces();
}

void PoseInitFlow::InitRosInterfaces() {

  // NEW创建回调组
  sensor_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // GNSS/LIDAR 回调组（共用同一 MutuallyExclusive 回调组，串行处理）
  rclcpp::SubscriptionOptions sensor_opts;
  sensor_opts.callback_group = sensor_callback_group_;

  // 订阅传感器消息
  init_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/initial_pose", 10);
  imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>("/imu", 50, std::bind(&PoseInitFlow::ImuCallback, this, std::placeholders::_1), sensor_opts);
  gnss_subscriber_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>("/navsatfix", 10, std::bind(&PoseInitFlow::GnssCallback, this, std::placeholders::_1), sensor_opts);
  cloud_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points", 10, std::bind(&PoseInitFlow::CloudCallback, this, std::placeholders::_1), sensor_opts);
}

void PoseInitFlow::GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {

}

void PoseInitFlow::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {

}

void PoseInitFlow::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  
}

void PoseInitFlow::PublishInitPose() {

}
} // namespace localization