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

  // ========== 关键修改：先判断是否需要重放 ==========
  if (need_replay_) {
    // 有待处理的校正：从校正时刻开始，把缓存中所有晚于校正时刻的 IMU 
    // （包括刚入缓存的这帧）一次性重放，保证递推顺序正确
    ReplayImuBuffer();
  } else {
    // 没有待处理的校正：正常逐帧递推当前 IMU
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