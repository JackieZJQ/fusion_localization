#include "models/synchronization/measure_sync.hpp"

namespace localization {
MessageSync::MessageSync(std::function<void(const MeasureGroup&)> cb) 
  : callback_(cb)  {
  LOG(INFO) << "Message sync init success.";
}

void MessageSync::ProcessIMU(IMU::Ptr imu) {
  double timestamp = imu->timestamp_;
  if (timestamp < last_timestamp_imu_) {
    LOG(WARNING) << "Imu loop back, clear buffer.";
    imu_buffer_.clear();
  }

  last_timestamp_imu_ = timestamp;
  imu_buffer_.emplace_back(imu);
}

void MessageSync::ProcessCloud(CLOUD::Ptr cloud) {
  double timestamp = cloud->timestamp_;
  if (timestamp < last_timestamp_lidar_) {
    LOG(WARNING) << "Lidar loop back, clear buffer.";
    lidar_buffer_.clear();
  }

  lidar_buffer_.push_back(cloud->full_cloud_ptr_);
  time_buffer_.push_back(timestamp);
  last_timestamp_lidar_ = timestamp;

  Sync();
}

bool MessageSync::Sync() {
  if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;
  
  if (!lidar_pushed_) {
    measures_.lidar_ = lidar_buffer_.front();
    measures_.lidar_begin_time_ = time_buffer_.front();

    lidar_end_time_ = measures_.lidar_begin_time_ + measures_.lidar_->points.back().time / double(1000);

    measures_.lidar_end_time_ = lidar_end_time_;
    lidar_pushed_ = true;
  }

  if (last_timestamp_imu_ < lidar_end_time_) return false;
  
  double imu_time = imu_buffer_.front()->timestamp_;
  measures_.imu_.clear();
  while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
    imu_time = imu_buffer_.front()->timestamp_;
    if (imu_time > lidar_end_time_) {
      break;
    }
    measures_.imu_.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
  lidar_pushed_ = false;

  if (callback_) {
    callback_(measures_);
  }

  return true;
}

}  // namespace localization