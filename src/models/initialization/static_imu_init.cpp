#include "models/initialization/static_imu_init.hpp"

namespace localization {
StaticIMUInit::StaticIMUInit(Options options) 
  : options_(options) { }

bool StaticIMUInit::AddIMU(const IMU& imu) {
  if (init_success_) return true;
  if (imu.acce_.norm() > 0.1) return false; //过滤加速度记异常值

  if (options_.use_speed_for_static_checking_ && !is_static_) {
    LOG(WARNING) << "等待车辆静止";
    init_imu_deque_.clear();
    return false;
  }

  if (init_imu_deque_.empty()) 
    init_start_time_ = imu.timestamp_; //记录初始静止时间

  init_imu_deque_.push_back(imu);      //记入初始化队列

  double init_time = imu.timestamp_ - init_start_time_;  //初始化经过时间
  if (init_time > options_.init_time_seconds_) 
    TryInit(); //尝试初始化逻辑
  
  //维持初始化队列长度
  while (init_imu_deque_.size() > options_.init_imu_queue_max_size_) {
    init_imu_deque_.pop_front();
  }

  current_time_ = imu.timestamp_;
  return false;
}

bool StaticIMUInit::AddODOM(const ODOM& odom) {
  if (init_success_) return true; 
  
  // 判断车辆是否静止
  if (odom.left_pulse_ < options_.static_odom_pulse_ && odom.right_pulse_ < options_.static_odom_pulse_) {
    is_static_ = true;
  } else {
    is_static_ = false;
  }

  current_time_ = odom.timestamp_;
  return true;
}

bool StaticIMUInit::TryInit() {
  if (init_imu_deque_.size() < 10) return false; 

  //计算均值和方差
  Vec3d mean_gyro, mean_acce;
  math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU& imu) { return imu.gyro_; });
  math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce_; });

  //以acce均值为方向，取9.8长度为重力
  LOG(INFO) << "mean acce: " << mean_acce.transpose();
  gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;

  //重新计算加计的协方差
  math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                              [this](const IMU& imu) { return imu.acce_ + gravity_; });

  //检查IMU噪声
  if (cov_gyro_.norm() > options_.max_static_gyro_var) {
    LOG(ERROR) << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > " << options_.max_static_gyro_var;
    return false;
  }

  if (cov_acce_.norm() > options_.max_static_acce_var) {
    LOG(ERROR) << "加速度计测量噪声太大" << cov_acce_.norm() << " > " << options_.max_static_acce_var;
    return false;
  }

  //估计测量噪声和零偏
  init_bg_ = mean_gyro;
  init_ba_ = mean_acce;

  LOG(INFO) << "IMU 初始化成功，初始化时间= " << current_time_ - init_start_time_ << "\n" 
            << "陀螺仪初始零偏 bg = " << init_bg_.transpose() << "\n"
            << "加速度记初始零偏 ba = " << init_ba_.transpose() << "\n" 
            << "陀螺仪方差 gyro sq = " << cov_gyro_.transpose() << "\n"
            << "加速度记方差 acce sq = " << cov_acce_.transpose() << "\n"
            << "重力 grav = " << gravity_.transpose() << "\n" //重力为加速度计归一化后*options的重力值
            << "重力 norm = " << gravity_.norm();
  LOG(INFO) << "陀螺仪均值 mean gyro = " << mean_gyro.transpose() << "\n"
            << "加速度记均值 mean acce = " << mean_acce.transpose();
  
  init_success_ = true;
  return true;
}

bool StaticIMUInit::InitSuccess() const { 
  return init_success_; 
}

Vec3d StaticIMUInit::GetCovGyro() const { 
  return cov_gyro_; 
}

Vec3d StaticIMUInit::GetCovAcce() const { 
  return cov_acce_; 
}

Vec3d StaticIMUInit::GetInitBg() const { 
  return init_bg_; 
}

Vec3d StaticIMUInit::GetInitBa() const { 
  return init_ba_; 
}

Vec3d StaticIMUInit::GetGravity() const { 
  return gravity_; 
}
}  // namespace localization
