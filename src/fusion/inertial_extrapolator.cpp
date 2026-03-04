/**
 * ************************************************************************
 * 
 * @file inertial_extrapolator.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief IMU 定位预测模块
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include "fusion/inertial_extrapolator.hpp"

namespace localization {

InertialExtrapolator::InertialExtrapolator(const Options& options)
  : options_(options) {

  // todo
}

void InertialExtrapolator::PushImu(IMU::Ptr imu) {
  latest_imu_time_ = imu->timestamp_;
  imu_buffer_.push_back(imu);

  // 裁剪缓存，缓存一般保存5S以内的IMU数据，超过这个时间的就丢弃
  while (!imu_buffer_.empty() && 
          (latest_imu_time_ - imu_buffer_.front()->timestamp_) > options_.buffer_keep_s) {
    imu_buffer_.pop_front();
  }

  // 这帧IMU数据的时间戳不应该早于上次校正的时间戳，否则说明校正之后的重放还没开始就来了新的IMU数据，可能会导致状态不连续
  // 这种情况比较极端，通常是系统刚启动时第一批IMU数据还没来就收到了第一帧点云并完成了第一次校正，此时就会出现这种情况
  // 这里选择直接丢弃这帧IMU数据，等待下一帧IMU数据的到来
  if (latest_imu_time_ <= eskf_corrected_time_) return;
  eskf_pred_.Predict(*imu);

  // 如果之前有校正等待重放，这里可以顺便做，也可以留给定时器
  if (need_replay_) ReplayImuBuffer();
}

void InertialExtrapolator::CorrectState(const ESKFD& eskf_corrected, double eskf_corrected_time) {
  eskf_pred_ = eskf_corrected;  // 深拷贝状态与协方差
  eskf_corrected_time_ = eskf_corrected_time;
  need_replay_ = true;
}

void InertialExtrapolator::ReplayImuBuffer() {
  if (!need_replay_ || eskf_corrected_time_ < 0.0) return;
  is_replaying_ = true;

  for (auto& imu : imu_buffer_) {
    if (imu->timestamp_ > eskf_corrected_time_)
      eskf_pred_.Predict(*imu);
  }

  need_replay_ = false;
  is_replaying_ = false;
}

bool InertialExtrapolator::IsReplaying() const { 
  return is_replaying_; 
}
  
NavStated InertialExtrapolator::GetState() const { 
  return eskf_pred_.GetNominalState(); 
}
  
double InertialExtrapolator::LatestImuTime() const { 
  return latest_imu_time_; 
}
}  // namespace localization