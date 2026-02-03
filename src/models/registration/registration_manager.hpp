/**
 * ************************************************************************
 * 
 * @file registration_manager.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 双NDT管理模块，用于点云地图无缝切换
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2025 
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "models/registration/registration_interface.hpp"
#include "models/registration/ndt_omp_registration.hpp"
#include "models/registration/fast_gicp_registration.hpp"

#include "common/point_types.h"

namespace localization {
class RegistrationManager {
public:
  RegistrationManager() = default;
  RegistrationManager(const YAML::Node& yaml);
  
  ~RegistrationManager();

  void UpdateRefCloud(const CloudPtr& ref_cloud_ptr);

  bool Align(const CloudPtr& cloud_ptr, const Eigen::Matrix4f& predict_pose, Eigen::Matrix4f& result_pose);
  
  float GetFitnessScore();

  float GetTransformationProbaility();

  void Stop();

  using Ptr = std::shared_ptr<localization::RegistrationManager>;

private:
  void WorkerThreadLoop();

  RegistrationInterface::Ptr registration_ptr_;
  RegistrationInterface::Ptr registration_secondary_ptr_;
  mutable std::mutex registration_mutex_;

  CloudPtr ref_cloud_ptr_;
  mutable std::mutex ref_cloud_mutex_;

  std::atomic_bool ref_cloud_updated_{false};
  std::atomic_bool is_running_{false};
  std::thread worker_;
  YAML::Node yaml_; //ndt参数配置
};
} // namespace localization