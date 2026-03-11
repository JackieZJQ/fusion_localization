/**
 * ************************************************************************
 * 
 * @file imu_predictor_node.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief IMU 预测独立节点（独立进程），订阅 /imu 和 /localization/corrected_state
 *        以 200Hz 发布高频 TF 和 odom，完全不受 GICP 配准耗时影响
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <glog/logging.h>
#include <deque>

#include "fusion/imu_predictor.hpp"

class ImuPredictorNode : public rclcpp::Node {
public:
ImuPredictorNode() : Node("imu_predictor_node") {
  
  // 订阅 IMU
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", 50,
        std::bind(&ImuPredictorNode::OnImu, this, std::placeholders::_1));

  // 订阅 fusion_node 发布的校正状态
  corrected_sub_ = create_subscription<nav_msgs::msg::Odometry>("/localization/registration_state", 10,
        std::bind(&ImuPredictorNode::OnCorrected, this, std::placeholders::_1));

  // 发布高频 odom 和 TF
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/imu_od", 50);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  LOG(INFO) << "ImuPredictorNode started.";
}

private:  
std::deque<localization::IMU> imu_buffer_;
static constexpr double kImuBufferKeepSec = 3.0;

void OnImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!predictor_.IsInitialized()) return;

  localization::IMU imu;
  imu.timestamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  
  imu.acce_ = Eigen::Vector3d(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);
  
  imu.gyro_ = Eigen::Vector3d(msg->angular_velocity.x,
                                msg->angular_velocity.y,
                                msg->angular_velocity.z);

  // 1. 入缓存
  imu_buffer_.push_back(imu);
  // 裁剪过期帧
  while (!imu_buffer_.empty() &&
         (imu.timestamp_ - imu_buffer_.front().timestamp_) > kImuBufferKeepSec) {
    imu_buffer_.pop_front();
  }

  predictor_.Predict(imu);

  // 发布 odometry
  auto state = predictor_.GetState();
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = "rslidar";
  odom.pose.pose.position.x = state.p_.x();
  odom.pose.pose.position.y = state.p_.y();
  odom.pose.pose.position.z = state.p_.z();
  odom.pose.pose.orientation.w = state.R_.unit_quaternion().w();
  odom.pose.pose.orientation.x = state.R_.unit_quaternion().x();
  odom.pose.pose.orientation.y = state.R_.unit_quaternion().y();
  odom.pose.pose.orientation.z = state.R_.unit_quaternion().z();
  odom_pub_->publish(odom);

  // 发布 TF
  geometry_msgs::msg::TransformStamped tf;
  tf.header = odom.header;
  tf.child_frame_id = "rslidar";
  tf.transform.translation.x = state.p_.x();
  tf.transform.translation.y = state.p_.y();
  tf.transform.translation.z = state.p_.z();
  tf.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf);
}

void OnCorrected(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // 从 fusion_node 收到 GICP 校正后的状态，重置递推器
  localization::NavStated state;
  state.timestamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  state.p_ = Eigen::Vector3d(msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z);
  Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z);
  state.R_ = Sophus::SO3d(q);
  state.v_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                              msg->twist.twist.linear.y,
                              msg->twist.twist.linear.z);
  // bg 和 ba 通过 twist.angular 和 covariance 传递
  state.bg_ = Eigen::Vector3d(msg->twist.twist.angular.x,
                                msg->twist.twist.angular.y,
                                msg->twist.twist.angular.z);
  state.ba_ = Eigen::Vector3d(msg->pose.covariance[0],
                                msg->pose.covariance[1],
                                msg->pose.covariance[2]);
  Eigen::Vector3d gravity(msg->pose.covariance[3],
                            msg->pose.covariance[4],
                            msg->pose.covariance[5]);

  predictor_.SetState(state, gravity);

  for (const auto& imu : imu_buffer_) {
    if (imu.timestamp_ > state.timestamp_) {
      predictor_.Predict(imu);
    }
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Corrected state received, t=%.3f", state.timestamp_);
  }

  localization::ImuPredictor predictor_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr corrected_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPredictorNode>());
  rclcpp::shutdown();
  return 0;
}