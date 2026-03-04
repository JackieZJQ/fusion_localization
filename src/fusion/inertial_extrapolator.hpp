/**
 * ************************************************************************
 * 
 * @file inertial_extrapolator.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief IMU 定位预测模块
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once
#include <deque>
#include <atomic>

#include "models/kalmanfilter/eskf.hpp"
#include "sensor_data/imu_data.hpp"

namespace localization {
class InertialExtrapolator {
public:
  struct Options {
    double buffer_keep_s = 5.0;    // IMU 缓存时长
    double publish_rate_hz = 40.0; // 预测状态发布频率（Hz）
  };

  InertialExtrapolator(const Options& options);

  // 来一帧 IMU：入缓存 + 推进预测滤波器
  void PushImu(IMU::Ptr imu);

  // 在观测（雷达/GNSS）校正后，将主滤波器状态拷贝进预测滤波器
  // eskf_corrected_time = 校正对应的时间戳（通常是 lidar_end_time）
  void CorrectState(const ESKFD& eskf_corrected, double eskf_corrected_time);

  // 把校正时刻之后的 IMU 重放, 让预测 ESKF 推进到最新 IMU 时间
  void ReplayImuBuffer();

  // 查询
  bool IsReplaying() const;

  NavStated GetState() const;

  double LatestImuTime() const;

private:
  Options options_;
  ESKFD eskf_pred_;  // 预测滤波器状态

  std::deque<IMU::Ptr> imu_buffer_;
  double latest_imu_time_ = -1.0;
  double eskf_corrected_time_ = -1.0;
  
  std::atomic<bool> need_replay_ = false;
  std::atomic<bool> is_replaying_ = false;
};
}  // namespace localization