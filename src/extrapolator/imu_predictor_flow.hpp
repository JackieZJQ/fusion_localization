/**
 * ************************************************************************
 *
 * @file imu_predictor_node.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief IMU 预测独立节点
 *        - 订阅 /imu，高频积分维护 p/v/R
 *        - 订阅 /localization/registration_state，收到 GICP 校正后：
 *            1. 用两次 GICP 位置差分替代 ESKF 速度（解决前后抖动）
 *            2. 重放 IMU 缓存追赶当前时刻（解决校正后停顿）
 *        - 以 25Hz 发布 odom 和 TF（IMU 全频积分，降频发布）
 *
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_localization_msgs/msg/kinematic_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <glog/logging.h>
#include <deque>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "extrapolator/imu_predictor.hpp"

namespace localization {

class ImuPredictorFlow : public rclcpp::Node {
public:
  ImuPredictorFlow() : Node("imu_predictor_flow") {

  // 订阅 IMU（全频接收，不降频，保证积分精度）
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", 200, std::bind(&ImuPredictorFlow::OnImu, this, std::placeholders::_1));

  // 订阅 fusion_node 发布的 GICP 校正状态
  registration_state_sub_ = create_subscription<nav_msgs::msg::Odometry>("/localization/registration_state", 10, std::bind(&ImuPredictorFlow::OnCorrected, this, std::placeholders::_1));

  // 发布 odom 和 TF
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/imu_predictor_odom", 10);

  // 发布轨迹
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/localization/imu_predictor_path", 10);

  kinematicstate_pub_ = create_publisher<autoware_localization_msgs::msg::KinematicState>("/localization/kinematicstate", 10);

  imu_path_.header.frame_id = "map";

  LOG(INFO) << "ImuPredictorNode started.";
}

private:
  static constexpr double kImuBufferKeepSec = 3.0;   // IMU 缓存时长
  static constexpr double kPublishPeriodSec  = 1.0 / 60.0; // 50Hz 发布周期

  localization::ImuPredictor predictor_;

  // IMU 缓存（用于 GICP 校正后的重放）
  std::deque<localization::IMU> imu_buffer_;

  // 限频发布控制
  double last_publish_time_ = 0.0;

  // 差分速度计算：记录上一次 GICP 校正的位置和时间
  bool last_corrected_valid_ = false;
  Eigen::Vector3d last_corrected_p_ = Eigen::Vector3d::Zero();
  double last_corrected_p_time_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path imu_path_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr registration_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<autoware_localization_msgs::msg::KinematicState>::SharedPtr kinematicstate_pub_;

  // IMU 回调：全频积分 + 限频发布
  void OnImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!predictor_.IsInitialized()) return;

    // 解析 IMU
    localization::IMU imu;
    imu.timestamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    imu.acce_ = Eigen::Vector3d(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);

    imu.gyro_ = Eigen::Vector3d(msg->angular_velocity.x,
                                msg->angular_velocity.y,
                                msg->angular_velocity.z);

    // 入缓存并裁剪过期帧
    imu_buffer_.push_back(imu);
    while (!imu_buffer_.empty() &&
           (imu.timestamp_ - imu_buffer_.front().timestamp_) > kImuBufferKeepSec) {
      imu_buffer_.pop_front();
    }

    // 全频积分（不管是否发布，每帧都积分保证精度）
    predictor_.Predict(imu);

    // 限频发布：距上次发布超过 kPublishPeriodSec 才发布
    if ((imu.timestamp_ - last_publish_time_) < kPublishPeriodSec) return;
    last_publish_time_ = imu.timestamp_;

    PublishState(msg->header.stamp, imu);
  }

  // GICP 校正回调：差分速度 + 重放缓存
  void OnCorrected(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // 解析校正状态（p, R 来自 GICP，精度高）
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

    // 关键：用两次 GICP 位置差分计算速度，替代 ESKF 速度
    // GICP 位置精度 ~cm，差分速度比 ESKF 无约束积分的速度稳定得多，
    // 可有效消除运动方向的前后抖动。
    if (last_corrected_valid_) {
      double dt = state.timestamp_ - last_corrected_p_time_;
      // dt 合理（0.05s ~ 1.0s，对1~20Hz 的 GICP 更新）
      if (dt > 0.05 && dt < 1.0) {
        state.v_ = (state.p_ - last_corrected_p_) / dt;
        RCLCPP_DEBUG(get_logger(), "Diff velocity: [%.3f, %.3f, %.3f] m/s",
                     state.v_.x(), state.v_.y(), state.v_.z());
      } else {
        // dt 不合理时 fallback：使用 ESKF 传来的速度
        state.v_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                   msg->twist.twist.linear.y,
                                   msg->twist.twist.linear.z);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Diff dt=%.3f out of range, fallback to ESKF velocity.", dt);
      }
    } else {
      // 第一次校正，没有上一帧，直接用 ESKF 速度作为初值
      state.v_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                 msg->twist.twist.linear.y,
                                 msg->twist.twist.linear.z);
    }

    // 记录本次 GICP 位置，供下次差分
    last_corrected_p_      = state.p_;
    last_corrected_p_time_ = state.timestamp_;
    last_corrected_valid_  = true;

    // 解析 bg, ba, gravity
    state.bg_ = Eigen::Vector3d(msg->twist.twist.angular.x,
                                msg->twist.twist.angular.y,
                                msg->twist.twist.angular.z);
    state.ba_ = Eigen::Vector3d(msg->pose.covariance[0],
                                msg->pose.covariance[1],
                                msg->pose.covariance[2]);
    Eigen::Vector3d gravity(msg->pose.covariance[3],
                            msg->pose.covariance[4],
                            msg->pose.covariance[5]);

    // 重置到校正时刻
    predictor_.SetState(state, gravity);

    // 重放校正时刻之后的 IMU，追赶到最新时刻（消除校正后的停顿）
    int replay_count = 0;
    for (const auto& imu : imu_buffer_) {
      if (imu.timestamp_ > state.timestamp_) {
        predictor_.Predict(imu);
        ++replay_count;
      }
    }

    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    //     "Corrected: t=%.3f, replayed %d IMU frames, "
    //     "v=[%.3f,%.3f,%.3f]",
    //     state.timestamp_, replay_count,
    //     state.v_.x(), state.v_.y(), state.v_.z());

    // RCLCPP_WARN(get_logger(),
    //             "GICP p=[%.4f,%.4f,%.4f] t=%.4f | diff_v=[%.4f,%.4f,%.4f]",
    //             state.p_.x(), state.p_.y(), state.p_.z(), state.timestamp_,
    //             state.v_.x(), state.v_.y(), state.v_.z());
  }

  // 发布当前状态（odom）
  void PublishState(const rclcpp::Time& stamp, const localization::IMU& imu) {
    auto state = predictor_.GetState();

    // 1.发布轨迹点
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = state.p_.x();
    pose_stamped.pose.position.y = state.p_.y();
    pose_stamped.pose.position.z = state.p_.z();
    pose_stamped.pose.orientation.w = state.R_.unit_quaternion().w();
    pose_stamped.pose.orientation.x = state.R_.unit_quaternion().x();
    pose_stamped.pose.orientation.y = state.R_.unit_quaternion().y();
    pose_stamped.pose.orientation.z = state.R_.unit_quaternion().z();

    imu_path_.header.stamp = stamp;
    imu_path_.poses.push_back(pose_stamped);

    // 限制轨迹点数量，防止内存无限增长
    if (imu_path_.poses.size() > 5000) {
      imu_path_.poses.erase(imu_path_.poses.begin());
    }

    path_pub_->publish(imu_path_);

    // 2.发布odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id  = "rslidar";
    odom.pose.pose.position.x    = state.p_.x();
    odom.pose.pose.position.y    = state.p_.y();
    odom.pose.pose.position.z    = state.p_.z();
    odom.pose.pose.orientation.w = state.R_.unit_quaternion().w();
    odom.pose.pose.orientation.x = state.R_.unit_quaternion().x();
    odom.pose.pose.orientation.y = state.R_.unit_quaternion().y();
    odom.pose.pose.orientation.z = state.R_.unit_quaternion().z();
    odom_pub_->publish(odom);

    // 3.发布autoware_localization_msgs
    autoware_localization_msgs::msg::KinematicState kinematicstate;
    kinematicstate.header.stamp = stamp;
    kinematicstate.header.frame_id = "map";
    kinematicstate.child_frame_id = "rslidar";

    // 3.1位置
    kinematicstate.pose_with_covariance.pose.position.x = state.p_.x();
    kinematicstate.pose_with_covariance.pose.position.y = state.p_.y();
    kinematicstate.pose_with_covariance.pose.position.z = state.p_.z();
    // 3.2姿态
    kinematicstate.pose_with_covariance.pose.orientation.w = state.R_.unit_quaternion().w();
    kinematicstate.pose_with_covariance.pose.orientation.x = state.R_.unit_quaternion().x();
    kinematicstate.pose_with_covariance.pose.orientation.y = state.R_.unit_quaternion().y();
    kinematicstate.pose_with_covariance.pose.orientation.z = state.R_.unit_quaternion().z();

    // 3.3全局坐标系速度
    kinematicstate.twist_with_covariance.twist.linear.x = state.v_.x();
    kinematicstate.twist_with_covariance.twist.linear.y = state.v_.y();
    kinematicstate.twist_with_covariance.twist.linear.z = state.v_.z();

    // 3.4自车坐标系角速度（IMU直接获取）
    kinematicstate.twist_with_covariance.twist.angular.x = imu.gyro_.x();
    kinematicstate.twist_with_covariance.twist.angular.y = imu.gyro_.y();
    kinematicstate.twist_with_covariance.twist.angular.z = imu.gyro_.z();

    // 3.加速度
    // 3.1自车坐标系线加速度
    kinematicstate.accel_with_covariance.accel.linear.x = imu.acce_.x();
    kinematicstate.accel_with_covariance.accel.linear.y = imu.acce_.y();
    kinematicstate.accel_with_covariance.accel.linear.z = imu.acce_.z();

    kinematicstate_pub_->publish(kinematicstate);

    // 每次发布都打印，不限频
    // RCLCPP_INFO(get_logger(),
    //             "[Pub] t=%.4f p=[%.4f, %.4f, %.4f]",
    //             state.timestamp_,
    //             state.p_.x(), state.p_.y(), state.p_.z());
  }
};
} // namespace localization

