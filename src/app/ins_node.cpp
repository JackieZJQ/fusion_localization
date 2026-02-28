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
#include <localization_msgs/msg/kinematic_state.hpp>
#include <beidou_ins_driver/msg/inspva.hpp>

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

  sync_pub_ = this->create_publisher<localization_msgs::msg::KinematicState>("/localization", 10);

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
rclcpp::Publisher<localization_msgs::msg::KinematicState>::SharedPtr sync_pub_;

// 本地地图原点
Eigen::Vector3d map_origin_;

void SyncCallback(const beidou_ins_driver::msg::Inspva::ConstSharedPtr& inspva,
                  const sensor_msgs::msg::Imu::ConstSharedPtr& imu) {
  localization_msgs::msg::KinematicState msg;  

  // 时间戳和坐标系
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "map";
  msg.child_frame_id = "imu_link";
  
  // 全局位置
  localization::GNSS::UTMCoordinate utm_coor;
  localization::LatLon2UTM(Eigen::Vector2d(inspva->latitude, inspva->longitude), utm_coor);
  msg.position.x = utm_coor.xy_[0] - map_origin_.x();
  msg.position.y = utm_coor.xy_[1] - map_origin_.y();
  msg.position.z = inspva->height - map_origin_.z(); // 以地图原点为基准

  // 全局姿态
  msg.orientation_rpy.x = inspva->roll * localization::math::kDEG2RAD;
  msg.orientation_rpy.y = -inspva->pitch * localization::math::kDEG2RAD;
  msg.orientation_rpy.z = (90.0-inspva->azimuth) * localization::math::kDEG2RAD;  // 北向为0度，顺时针增加

  tf2::Quaternion q;
  q.setRPY(msg.orientation_rpy.x, msg.orientation_rpy.y, msg.orientation_rpy.z);
  q.normalize();  

  msg.orientation_q.w = q.w();
  msg.orientation_q.x = q.x();
  msg.orientation_q.y = q.y();
  msg.orientation_q.z = q.z();

  //全局速度
  msg.linear_velocity.x = inspva->east_velocity;
  msg.linear_velocity.y = inspva->north_velocity;
  msg.linear_velocity.z = inspva->up_velocity;

  // IMU测量的线加速度和角速度
  msg.linear_acceleration.x = imu->linear_acceleration.x;
  msg.linear_acceleration.y = imu->linear_acceleration.y;
  msg.linear_acceleration.z = imu->linear_acceleration.z;

  msg.angular_velocity.x = imu->angular_velocity.x;
  msg.angular_velocity.y = imu->angular_velocity.y;
  msg.angular_velocity.z = imu->angular_velocity.z;

  // 惯导状态信息
  msg.status = 255;
  msg.source = "beidou_ins";
  msg.map_id = "map";
  msg.gravity = 9.81;
  msg.imu_compensated = true;

  sync_pub_->publish(msg);
}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // 加载yaml配置文件
  std::string yaml_path = "/home/jackie/2026/localization/src/fusion_localization/config/localization_robosense.yaml";
  auto yaml = YAML::LoadFile(yaml_path);

  auto beidou_sync_node = std::make_shared<SyncImuGPS>("beidou_sync_node", yaml);

  rclcpp::spin(beidou_sync_node);
  rclcpp::shutdown();

  return 0;
}