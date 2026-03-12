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

#pragma once

#include "common/eigen_types.hpp"
#include "sensor_data/imu_data.hpp"
#include "sensor_data/nav_state.hpp"

namespace localization {

class ImuPredictor {
public:
  ImuPredictor() = default;

  void SetState(const NavStated& state, const Vec3d& gravity);

  bool Predict(const IMU& imu);

  NavStated GetState() const;

  bool IsInitialized() const;

private:
  Vec3d p_ = Vec3d::Zero();
  Vec3d v_ = Vec3d::Zero();
  SO3 R_;
  Vec3d bg_ = Vec3d::Zero();
  Vec3d ba_ = Vec3d::Zero();
  Vec3d g_ = {0, 0, -9.8};
  
  double current_time_ = 0.0;
  bool initialized_ = false;
};

}  // namespace localization