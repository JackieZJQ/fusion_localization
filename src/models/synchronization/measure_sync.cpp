#include "models/synchronization/measure_sync.hpp"

namespace localization {
MessageSync::MessageSync(std::function<void(const MeasureGroup&)> cb) 
  : callback_(cb)  {
  LOG(INFO) << "Message sync init success.";
}

void MessageSync::ProcessIMU(IMU::Ptr imu) {
  
  // 1.检查 IMU 时间戳是否回退，如果回退则清空 IMU缓冲
  double timestamp = imu->timestamp_;
  if (timestamp < last_timestamp_imu_) {
    LOG(WARNING) << "Imu loop back, clear buffer.";
    imu_buffer_.clear();
  }

  // 2.将 IMU 数据添加到缓冲，并记录最新的 IMU 时间戳
  imu_buffer_.emplace_back(imu);
  last_timestamp_imu_ = timestamp;
}

void MessageSync::ProcessCloud(CLOUD::Ptr cloud) {
  
  // 1.检查雷达时间戳是否回退，如果回退则清空雷达缓冲
  double timestamp = cloud->timestamp_;
  if (timestamp < last_timestamp_lidar_) {
    LOG(WARNING) << "Lidar loop back, clear buffer.";
    lidar_buffer_.clear();
  }

  // 2.将雷达数据和雷达时间添加到缓冲，并记录最新的雷达时间戳
  // todo
  // time_buffer_需要删除吗？
  lidar_buffer_.push_back(cloud->full_cloud_ptr_);
  time_buffer_.push_back(timestamp);
  last_timestamp_lidar_ = timestamp;

  // 3.尝试同步数据，成功时调用回调函数
  Sync();
}

bool MessageSync::Sync() {
  if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;
  
  // 1.存储雷达数据至measures_，并记录雷达数据的开始和结束时间
  if (!lidar_pushed_) {
    measures_.lidar_ = lidar_buffer_.front();
    measures_.lidar_begin_time_ = time_buffer_.front();

    //todo
    //确认雷达数据每个点的time的意义和单位，距离第一个点的时间差，单位为毫秒？
    lidar_end_time_ = measures_.lidar_begin_time_ + 
                      measures_.lidar_->points.back().time / 1000.0; 

    measures_.lidar_end_time_ = lidar_end_time_;
    lidar_pushed_ = true;
  }

  // 2.检查imu数据是否覆盖雷达数据的起始范围, 时间需要 [lidar_begin, lidar_end]
  // 这里加一个小的epsilon，允许 IMU 时间戳略微落后于雷达开始时间，考虑到传感器时间戳的误差和不确定性
  constexpr double epsilon = 0.002;
  if (last_timestamp_imu_ + epsilon < measures_.lidar_begin_time_) return false;
  
  double imu_time = imu_buffer_.front()->timestamp_;
  measures_.imu_.clear();

  // 3.取出覆盖 [lidar_begin, lidar_end] 的 IMU 数据
  while (!imu_buffer_.empty() && imu_time < lidar_end_time_ + epsilon) {
    imu_time = imu_buffer_.front()->timestamp_;
    if (imu_time > lidar_end_time_ + epsilon) break;
    
    measures_.imu_.emplace_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  // 4.同步成功，清除已使用的雷达数据，lidar_pushed_置为false，等待下一次雷达数据的到来触发回调
  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
  lidar_pushed_ = false;

  // 5.调用回调函数，传递同步好的数据
  callback_(measures_);
  return true;
}

}  // namespace localization