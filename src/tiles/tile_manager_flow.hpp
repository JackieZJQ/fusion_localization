/**
 * ************************************************************************
 * 
 * @file tile_flow.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 点云块服务端
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <yaml-cpp/yaml.h>

#include "common/eigen_types.hpp"
#include "common/point_types.h"
#include "tiles/tile_manager.hpp"

namespace localization {
class TileManagerFlow {
public:
  TileManagerFlow() = delete;
  TileManagerFlow(const rclcpp::Node::SharedPtr& node);

  ~TileManagerFlow() = default;

  using Ptr = std::shared_ptr<localization::TileManagerFlow>;

private:
  void InitRosInterfaces();

  void PublishStaticPointcloudMap();

  void PublishSubMap(CloudPtr cloud);

  void RegistrationStateCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg_ptr);

  TileManager::Ptr tile_manager_ptr_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr registration_state_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pointcloud_map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_submap_publisher_;
  rclcpp::Node::SharedPtr node_;
  YAML::Node yaml_;
};
} // namespace localization
