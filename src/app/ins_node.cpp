/**
 * ************************************************************************
 * 
 * @file ins_node.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 使用惯性导航系统定位
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/msg/imu.hpp>
#include <beidou_ins_driver/msg/inspva.hpp>
#include <autoware_localization_msgs/msg/kinematic_state.hpp>

#include <math.h>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <glog/logging.h>

#include "common/math_utils.h"
#include "models/converter/utm_convert.hpp"

class SyncImuGPS : public rclcpp::Node {
public:
SyncImuGPS(const char* name, const YAML::Node& yaml) 
  : Node(name), sync_policy_(10), sync_(sync_policy_) {
  
  LOG(INFO) << "initializating ins localization node...";

  inspva_subscriber_.subscribe(this , "/beidou/inspva");
  imu_subscriber_.subscribe(this, "/beidou/corrimudata");
  sync_.connectInput(inspva_subscriber_, imu_subscriber_);
  sync_.registerCallback(std::bind(&SyncImuGPS::SyncCallback, this, std::placeholders::_1, std::placeholders::_2));

  sync_pub_ = this->create_publisher<autoware_localization_msgs::msg::KinematicState>("/localization/kinematicstate", 10);

  // 读取本地地图原点
  auto origin_data = yaml["origin"].as<std::vector<double>>();
  map_origin_ = Eigen::Vector3d(origin_data[0], origin_data[1], origin_data[2]);
}

private:
// message 同步参数
typedef message_filters::sync_policies::ApproximateTime<beidou_ins_driver::msg::Inspva, sensor_msgs::msg::Imu> sync_policy;
sync_policy sync_policy_;
message_filters::Subscriber<beidou_ins_driver::msg::Inspva> inspva_subscriber_;
message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
message_filters::Synchronizer<sync_policy> sync_;

// 发布同步后的消息
rclcpp::Publisher<autoware_localization_msgs::msg::KinematicState>::SharedPtr sync_pub_;

// 本地地图原点
Eigen::Vector3d map_origin_;

void SyncCallback(const beidou_ins_driver::msg::Inspva::ConstSharedPtr &inspva,
                  const sensor_msgs::msg::Imu::ConstSharedPtr &imu) {
  autoware_localization_msgs::msg::KinematicState msg;

  // 时间戳和坐标系
  msg.header.stamp = imu->header.stamp;
  msg.header.frame_id = "map";
  msg.child_frame_id = "imu_link";

  // 1.全局位姿
  localization::GNSS::UTMCoordinate utm_coor;
  localization::LatLon2UTM(Eigen::Vector2d(inspva->latitude, inspva->longitude), utm_coor);
  // 1.1位置
  msg.pose_with_covariance.pose.position.x = utm_coor.xy_[0] - map_origin_.x();
  msg.pose_with_covariance.pose.position.y = utm_coor.xy_[1] - map_origin_.y();
  msg.pose_with_covariance.pose.position.z = inspva->height - map_origin_.z(); // 以地图原点为基准

  // 1.2姿态
  double roll = inspva->roll * localization::math::kDEG2RAD;
  double pitch = -inspva->pitch * localization::math::kDEG2RAD;
  double yaw = (90.0 - inspva->azimuth) * localization::math::kDEG2RAD; // 北向为0度，顺时针增加

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  msg.pose_with_covariance.pose.orientation.w = q.w();
  msg.pose_with_covariance.pose.orientation.x = q.x();
  msg.pose_with_covariance.pose.orientation.y = q.y();
  msg.pose_with_covariance.pose.orientation.z = q.z();

  // 2.速度
  // 2.1全局坐标系速度
  msg.twist_with_covariance.twist.linear.x = inspva->east_velocity;
  msg.twist_with_covariance.twist.linear.y = inspva->north_velocity;
  msg.twist_with_covariance.twist.linear.z = inspva->up_velocity;
  // 2.2自车坐标系角速度（IMU直接获取）
  msg.twist_with_covariance.twist.angular.x = imu->angular_velocity.x;
  msg.twist_with_covariance.twist.angular.y = imu->angular_velocity.y;
  msg.twist_with_covariance.twist.angular.z = imu->angular_velocity.z;

  // 3.加速度
  // 3.1自车坐标系线加速度
  msg.accel_with_covariance.accel.linear.x = imu->linear_acceleration.x;
  msg.accel_with_covariance.accel.linear.y = imu->linear_acceleration.y;
  msg.accel_with_covariance.accel.linear.z = imu->linear_acceleration.z;

  sync_pub_->publish(msg);
}
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  rclcpp::init(argc, argv);

  // 加载yaml配置文件
  std::string yaml_path = "/home/jackie/2026/localization/src/fusion_localization/config/localization_robosense.yaml";
  auto yaml = YAML::LoadFile(yaml_path);

  auto beidou_sync_node = std::make_shared<SyncImuGPS>("beidou_sync_node", yaml);

  rclcpp::spin(beidou_sync_node);
  rclcpp::shutdown();

  return 0;
}