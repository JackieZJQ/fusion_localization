/**
 * ************************************************************************
 * 
 * @file imu_data.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief IMU数据结构
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2024 
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once 

#include <Eigen/Dense>
#include <sensor_msgs/msg/imu.hpp>

namespace localization {
struct IMU {
public:
  IMU() = default;
  IMU(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acce)
      : timestamp_(t), gyro_(gyro), acce_(acce) {}

  IMU(sensor_msgs::msg::Imu::Ptr msg) {
    timestamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    gyro_ = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    acce_ = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  }

  using Ptr = std::shared_ptr<localization::IMU>;

public:
  double timestamp_ = 0.0;
  Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acce_ = Eigen::Vector3d::Zero();
};
} //namespace localization