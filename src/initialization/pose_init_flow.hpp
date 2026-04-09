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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace localization {
class PoseInitFlow {
public:
  PoseInitFlow() = delete;
  PoseInitFlow(const rclcpp::Node::SharedPtr& node);

  ~PoseInitFlow() = default;

  using Ptr = std::shared_ptr<localization::PoseInitFlow>;

private:
  void InitRosInterfaces();

  void PublishInitPose();

  // 传感器回调函数
  void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr init_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

  // 回调组,并行处理传感器数据
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
};
} // namespace localization