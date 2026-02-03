/**
 * ************************************************************************
 * 
 * @file ndtomp_registration.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief NDTOMP匹配
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2025 
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>
#include <glog/logging.h>

#include "models/registration/registration_interface.hpp"
#include "common/point_types.h"

namespace localization {
class NDTOMPRegistration : public RegistrationInterface {
public:
  NDTOMPRegistration(const YAML::Node& yaml);
  NDTOMPRegistration(float res, float step_size, float trans_eps, int max_iter);

  bool SetInputTarget(const CloudPtr& input_target) override;
  bool ScanMatch(const CloudPtr& input_source,
                 const Eigen::Matrix4f& predict_pose,
                 CloudPtr result_cloud_ptr,
                 Eigen::Matrix4f& result_pose) override;
  float GetFitnessScore() override;
  float GetTransformationProbaility() override;

  using Ptr = std::shared_ptr<localization::NDTOMPRegistration>;

private:
  bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);
  bool WarmUp(const CloudPtr& input_target);

private:
  pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr ndtomp_ptr_;
  bool is_warmedup_ = false;
};
} //namespace localization