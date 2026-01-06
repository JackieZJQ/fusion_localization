/**
 * ************************************************************************
 * 
 * @file fusion_flow.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 融合定位ROS输入输出IO
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include "fusion/fusion_flow.hpp"
#include "common/point_types.h"

namespace localization {
FusionFlow::FusionFlow(const rclcpp::Node::SharedPtr& node) 
  : node_(node) {
  
  //todo
  //yaml文件的地址写入cmakelists文件
  const std::string config_path = "/home/jackie/robobus_localization/fusion_localization_ws/src/fusion_localization/config/localization_robosense.yaml";
  auto yaml = YAML::LoadFile(config_path);

  fusion_ptr_ = std::make_shared<Fusion>(yaml);
  cloud_converter_ptr_ = std::make_shared<CloudConvert>(yaml);

  //NEW创建回调组
  sensor_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  InitIO();

  PublishMap();
}

void FusionFlow::InitIO() {
  //订阅（共用同一 MutuallyExclusive 回调组，串行处理）
  rclcpp::SubscriptionOptions sensor_opts;
  sensor_opts.callback_group = sensor_cb_group_;

  //订阅传感器消息
  imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&FusionFlow::ImuCallback, this, std::placeholders::_1), sensor_opts);
  gnss_subscriber_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>("/navsatfix", 10, std::bind(&FusionFlow::GnssCallback, this, std::placeholders::_1), sensor_opts);
  cloud_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points", 10, std::bind(&FusionFlow::CloudCallback, this, std::placeholders::_1), sensor_opts);

  //发布定位数据
  current_scan_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/current_scan_undistorted", 10);
  odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
  localization_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/gps", 10);

  //大地图发布（使用 Reentrant 回调组，可并行处理）
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local().reliable();
  map_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/map", map_qos);

  //初始化tf广播器
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void FusionFlow::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr) {
  
  //转换为IMU格式
  IMU::Ptr imu = std::make_shared<localization::IMU>(imu_msg_ptr);
  fusion_ptr_->ProcessIMU(imu);
}

void FusionFlow::GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg_ptr) {

  //转换为GNSS格式
  GNSS::Ptr gnss(new GNSS(gnss_msg_ptr));
  ConvertGps2UTMOnlyTrans(*gnss);

  if (std::isnan(gnss->lat_lon_alt_[2]))
    return;

  fusion_ptr_->ProcessRTK(gnss);
}

void FusionFlow::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr) {
  //Timer timer("CloudCallback");
  // 对点云数量做滤波
  CLOUD::Ptr cloud_ptr(new CLOUD);
  cloud_ptr->timestamp_ = cloud_msg_ptr->header.stamp.sec + cloud_msg_ptr->header.stamp.nanosec * 1e-9;
  cloud_converter_ptr_->Process(cloud_msg_ptr, cloud_ptr->full_cloud_ptr_);
  fusion_ptr_->ProcessPointCloud(cloud_ptr);

  //todo
  //点云定位完成后,获取当前eskf状态
  PublishOdom();
  PublishLidarTf();

  //PublishCurrentScan();
}

void FusionFlow::PublishCurrentScan() {
  FullCloudPtr current_scan_undistorted = fusion_ptr_->GetCurrentScan();

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*current_scan_undistorted, cloud_msg);
  cloud_msg.header.frame_id = "rslidar";
  cloud_msg.header.stamp.sec = current_scan_undistorted->header.stamp / 1000000;
  cloud_msg.header.stamp.nanosec = (current_scan_undistorted->header.stamp % 1000000) * 1000;

  current_scan_publisher_->publish(cloud_msg);
}

void FusionFlow::PublishMap() {
  CloudPtr map = fusion_ptr_->GetVisualMap();

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*map, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();

  map_publisher_->publish(msg);
}

void FusionFlow::PublishOdom() {
  NavStated::Ptr state = fusion_ptr_->GetCurrentState();

  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "map";
  msg.child_frame_id = "rslidar";
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(state->timestamp_ * 1e9));
  
  msg.pose.pose.position.x = state->p_.x();
  msg.pose.pose.position.y = state->p_.y();
  msg.pose.pose.position.z = state->p_.z();
  
  msg.pose.pose.orientation.w = state->R_.unit_quaternion().w();
  msg.pose.pose.orientation.x = state->R_.unit_quaternion().x();
  msg.pose.pose.orientation.y = state->R_.unit_quaternion().y();
  msg.pose.pose.orientation.z = state->R_.unit_quaternion().z();

  odometry_publisher_->publish(msg);
}

void FusionFlow::PublishLidarTf() {
  NavStated::Ptr state = fusion_ptr_->GetCurrentState();

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = rclcpp::Time(static_cast<int64_t>(state->timestamp_ * 1e9)); 
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "rslidar";

  transform_stamped.transform.translation.x = state->p_.x();
  transform_stamped.transform.translation.y = state->p_.y();
  transform_stamped.transform.translation.z = state->p_.z();

  transform_stamped.transform.rotation.w = state->R_.unit_quaternion().w();
  transform_stamped.transform.rotation.x = state->R_.unit_quaternion().x();
  transform_stamped.transform.rotation.y = state->R_.unit_quaternion().y();
  transform_stamped.transform.rotation.z = state->R_.unit_quaternion().z();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void FusionFlow::PublishLidarLocalization() {
  NavStated::Ptr state = fusion_ptr_->GetCurrentState();

  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(3);
  msg.data[0] = state->p_.x();
  msg.data[1] = state->p_.y();
  msg.data[2] = state->p_.z();

  localization_publisher_->publish(msg);
}
} // namespace localizationmap