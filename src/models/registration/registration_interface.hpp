/**
 * ************************************************************************
 * 
 * @file registration_interface.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 点云匹配模块接口
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2025 
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once 

#include <Eigen/Dense>

#include "common/point_types.h"

namespace localization {
class RegistrationInterface {
public:
  virtual ~RegistrationInterface() = default;

  virtual bool SetInputTarget(const CloudPtr& input_target) = 0;
  virtual bool ScanMatch(const CloudPtr& input_source,
                         const Eigen::Matrix4f& predict_pose,
                         CloudPtr result_cloud_ptr,
                         Eigen::Matrix4f& result_pose) = 0;
  virtual float GetFitnessScore() = 0;
  virtual float GetTransformationProbaility() = 0;
  virtual float GetFinalIterNum() = 0;
  virtual bool HasConverged() = 0;


  using Ptr = std::shared_ptr<localization::RegistrationInterface>;
};
} //namespace localization