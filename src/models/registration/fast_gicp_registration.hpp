#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include <fast_gicp/gicp/fast_gicp.hpp>

#include "models/registration/registration_interface.hpp"
#include "common/point_types.h"

namespace localization {
class FastGICPRegistration : public RegistrationInterface {
public:
  FastGICPRegistration(const YAML::Node& yaml);
  FastGICPRegistration(int num_threads, int k_correspondences, int max_iter, double trans_eps, double corr_dist);

  bool SetInputTarget(const CloudPtr &input_target) override;
  bool ScanMatch(const CloudPtr &input_source,
                  const Eigen::Matrix4f &predict_pose,
                  CloudPtr result_cloud_ptr,
                  Eigen::Matrix4f &result_pose) override;

  float GetFitnessScore() override;
  float GetTransformationProbaility() override;

  using Ptr = std::shared_ptr<localization::FastGICPRegistration>;

private:
  bool SetRegistrationParam(int num_threads, int k_correspondences, int max_iter, double trans_eps, double corr_dist);

  //预热：避免首次配准耗时尖峰
  bool WarmUp(const CloudPtr& input_target);

private:
  fast_gicp::FastGICP<PointType, PointType>::Ptr fast_gicp_ptr_;
  bool is_warmedup_ = false;
};
} // namespace localization