#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <glog/logging.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

#include "common/point_types.h"

namespace localization {
class NdtManager {
public:
  NdtManager() = default;
  NdtManager(const YAML::Node& yaml);

  ~NdtManager();

  void UpdateRefCloud(const CloudPtr& ref_cloud);

  bool Align(const CloudPtr &cloud_ptr, const Eigen::Matrix4f &predict_pose, Eigen::Matrix4f &result_pose);

  double GetFitnessScore();

  void Stop();

  using Ptr = std::shared_ptr<localization::NdtManager>;
  
private:
  using Registration = pcl::NormalDistributionsTransform<PointType, PointType>;

  void WorkerThreadLoop();
  void SetOptions(Registration::Ptr ndt, double resolution, double trans_eps, double step_size, int max_iter);

  Registration::Ptr ndt_;
  Registration::Ptr ndt_secondary_;
  mutable std::mutex ndt_mutex_;
  
  CloudPtr ref_cloud_;
  mutable std::mutex ref_cloud_mutex_;

  std::atomic_bool ref_cloud_updated_{false};
  std::atomic_bool is_running_{false};

  std::thread worker_;
};
} // namespace localization