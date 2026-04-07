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

#include "extrapolator/inertial_extrapolator.hpp"

namespace localization {

InertialExtrapolator::InertialExtrapolator(const Options& options)
  : options_(options) {
}

void InertialExtrapolator::PushImu(IMU::Ptr imu) {
  latest_imu_time_ = imu->timestamp_;
  imu_buffer_.push_back(imu);

  // 裁剪缓存
  while (!imu_buffer_.empty() && 
          (latest_imu_time_ - imu_buffer_.front()->timestamp_) > options_.buffer_keep_s) {
    imu_buffer_.pop_front();
  }

  if (latest_imu_time_ <= eskf_corrected_time_) return;

  // ★ 修复：先重放再递推，二者互斥
  if (need_replay_) {
    ReplayImuBuffer();    // 重放包含当前这帧，不需要再单独 Predict
  } else {
    eskf_pred_.Predict(*imu);
  }
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