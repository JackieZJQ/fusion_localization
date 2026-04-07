/**
 * ************************************************************************
 * 
 * @file tile_manager_flow.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sophus/se3.hpp>

#include "tiles/tile_manager_flow.hpp"
#include "common/point_types.h"

namespace localization {
TileManagerFlow::TileManagerFlow(const rclcpp::Node::SharedPtr& node)
  : node_(node) {

  const std::string config_path = "/home/jackie/2026/localization/src/fusion_localization/config/localization_robosense.yaml";
  auto yaml = YAML::LoadFile(config_path);

  tile_manager_ptr_ = std::make_shared<TileManager>(yaml);

  InitRosInterfaces();

  PublishStaticPointcloudMap();
}

void TileManagerFlow::InitRosInterfaces() {
  
  // 大地图发布，一般给RVIZ显示使用
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local().reliable();
  static_pointcloud_map_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/static_pointcloud_map", map_qos);
  cloud_submap_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/cloud_submap", map_qos);

  registration_state_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>("/localization/registration_state", 10, std::bind(&TileManagerFlow::RegistrationStateCallback, this, std::placeholders::_1));
}

void TileManagerFlow::RegistrationStateCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg_ptr) {

  // 旋转四元数
  double w = odom_msg_ptr->pose.pose.orientation.w;
  double x = odom_msg_ptr->pose.pose.orientation.x;
  double y = odom_msg_ptr->pose.pose.orientation.y;
  double z = odom_msg_ptr->pose.pose.orientation.z;
  Eigen::Quaterniond q(w, x, y, z);
  q.normalize();

  // 平移向量
  double px = odom_msg_ptr->pose.pose.position.x;
  double py = odom_msg_ptr->pose.pose.position.y;
  double pz = odom_msg_ptr->pose.pose.position.z;
  Eigen::Vector3d t(px, py, pz);

  // 构造 Sophus::SE3d
  Sophus::SE3d registration_state(q, t);  
  tile_manager_ptr_->UpdateCurrentPose(registration_state);
 
  // 1.没初始化就别发布
  if (!tile_manager_ptr_->HasMapInitialized()) return;

  // 2.没变化就别发布
  if (!tile_manager_ptr_->HasMapChanged()) return;

  // 3.取图
  auto cloud_submap = tile_manager_ptr_->GetRefCloud();
  
  // 4.如果图为空，跳过
  if (!cloud_submap || cloud_submap->empty()) return;

  PublishSubMap(cloud_submap);
}

void TileManagerFlow::PublishSubMap(CloudPtr cloud) {

  // 构造msg
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();

  cloud_submap_publisher_->publish(msg);
}

void TileManagerFlow::PublishStaticPointcloudMap() {

  // 加载滤波后的全局点云地图，用于可视化，不用于定位
  auto static_pointcloud_map = tile_manager_ptr_->GetStaticPointcloudMap();

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*static_pointcloud_map, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();

  static_pointcloud_map_publisher_->publish(msg);
}
} // namespace localization