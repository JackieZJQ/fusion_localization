/**
 * ************************************************************************
 *
 * @file measure_sync.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 将雷达与IMU数据同步
 *
 * ************************************************************************
 * @copyright Copyright (c) 2025
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once 

#include <glog/logging.h>
#include <deque>

#include "models/converter/cloud_convert.hpp"
#include "sensor_data/imu_data.hpp"
#include "sensor_data/cloud_data.hpp"
#include "common/point_types.h"

namespace localization {
class MessageSync {
public:
  struct MeasureGroup { //雷达与IMU同步的数据结构
    MeasureGroup() : lidar_(new FullPointCloudType()) { };

    double lidar_begin_time_ = 0;   // 雷达包的起始时间
    double lidar_end_time_ = 0;     // 雷达的终止时间
    FullCloudPtr lidar_ = nullptr;  // 雷达点云
    std::deque<IMU::Ptr> imu_;      // 上一时时刻到现在的IMU读数
  };

  MessageSync(std::function<void(const MeasureGroup&)> cb);

  void ProcessIMU(IMU::Ptr imu);       //处理IMU 
  void ProcessCloud(CLOUD::Ptr cloud); //处理雷达点云
  
  using Ptr = std::shared_ptr<localization::MessageSync>;

private:
  bool Sync();   //尝试同步IMU与激光数据，成功时返回true

  std::function<void(const MeasureGroup &)> callback_; // 同步数据后的回调函数
  std::deque<FullCloudPtr> lidar_buffer_;              // 雷达数据缓冲
  std::deque<IMU::Ptr> imu_buffer_;                    // imu数据缓冲
  double last_timestamp_imu_ = -1.0;                   // 最近imu时间
  double last_timestamp_lidar_ = 0;                    // 最近lidar时间
  std::deque<double> time_buffer_;
  bool lidar_pushed_ = false;
  MeasureGroup measures_;
  double lidar_end_time_ = 0;
};
}  // namespace localization
