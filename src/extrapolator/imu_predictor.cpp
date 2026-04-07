/**
 * ************************************************************************
 *
 * @file imu_predictor.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 轻量级 IMU 纯递推器，只递推 nominal state（p,v,R），不算协方差
 *        计算量：~50 次浮点运算/帧（vs ESKF 的 ~12000 次）
 *
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include "extrapolator/imu_predictor.hpp"

namespace localization {

void ImuPredictor::SetState(const NavStated& state, const Vec3d& gravity) {
  p_ = state.p_;
  v_ = state.v_;
  R_ = state.R_;
  bg_ = state.bg_;
  ba_ = state.ba_;
  g_ = gravity;
  current_time_ = state.timestamp_;
  initialized_ = true;
}

bool ImuPredictor::Predict(const IMU& imu) {
  if (!initialized_) return false;

  double dt = imu.timestamp_ - current_time_;

  // dt < 0：时间倒退，属于传感器异常，重置时间但不积分
  if (dt < 0) {
    current_time_ = imu.timestamp_;
    return false;
  }

  // dt > 0.5s：系统刚初始化或长时间无数据，防止积分爆炸
  if (dt > 0.5) {
    current_time_ = imu.timestamp_;
    return false;
  }

  // dt 在 (0, 0.5] 范围内正常积分
  // 只递推 p, v, R，不算 F、不算 cov
  Vec3d new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
  Vec3d new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
  SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

  p_ = new_p;
  v_ = new_v;
  R_ = new_R;
  current_time_ = imu.timestamp_;
  return true;
}

NavStated ImuPredictor::GetState() const {
  return NavStated(current_time_, R_, p_, v_, bg_, ba_);
}

bool ImuPredictor::IsInitialized() const { 
  return initialized_; 
}

} // namespace localization