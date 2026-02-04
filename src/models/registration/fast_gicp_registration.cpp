#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>

#include "models/registration/fast_gicp_registration.hpp"
#include "common/point_types.h"
#include "common/timer.hpp"

namespace localization {
FastGICPRegistration::FastGICPRegistration(const YAML::Node& yaml)
  : fast_gicp_ptr_(new fast_gicp::FastGICP<PointType, PointType>()) {
  int num_threads = yaml["gicp"]["num_threads"] ? yaml["gicp"]["num_threads"].as<int>() : 4;
  int k_correspondences = yaml["gicp"]["k_correspondences"] ? yaml["gicp"]["k_correspondences"].as<int>() : 20;
  int max_iter = yaml["gicp"]["max_iter"] ? yaml["gicp"]["max_iter"].as<int>() : 40;
  double trans_eps = yaml["gicp"]["trans_eps"] ? yaml["gicp"]["trans_eps"].as<double>() : 1e-3;
  double corr_dist = yaml["gicp"]["corr_dist"] ? yaml["gicp"]["corr_dist"].as<double>() : 1.0;

  SetRegistrationParam(num_threads, k_correspondences, max_iter, trans_eps, corr_dist);
}

FastGICPRegistration::FastGICPRegistration(int num_threads, int k_correspondences, int max_iter, double trans_eps, double corr_dist)
  : fast_gicp_ptr_(new fast_gicp::FastGICP<PointType, PointType>()) {

  SetRegistrationParam(num_threads, k_correspondences, max_iter, trans_eps, corr_dist);
}

bool FastGICPRegistration::SetRegistrationParam(int num_threads, int k_correspondences, int max_iter, double trans_eps, double corr_dist) {

  fast_gicp_ptr_->setNumThreads(num_threads);
  fast_gicp_ptr_->setCorrespondenceRandomness(k_correspondences);

  fast_gicp_ptr_->setMaximumIterations(max_iter);
  fast_gicp_ptr_->setTransformationEpsilon(trans_eps);
  fast_gicp_ptr_->setMaxCorrespondenceDistance(corr_dist);

  LOG(INFO) << "############初始化新FASTGICP, 匹配参数如下##############\n"
            << "num of threads: " << num_threads << "\n"
            << "correspondence randomness: " << k_correspondences << "\n"
            << "max correspondenceDistance: " << corr_dist << "\n"
            << "transformation epsilon: " << trans_eps << "\n"
            << "maximum iterations: " << max_iter << "\n";

  return true;
}

bool FastGICPRegistration::SetInputTarget(const CloudPtr &input_target) {
  //Timer t("FastGICP::SetInputTartget", 40);

  if (!fast_gicp_ptr_) {
    LOG(INFO) << "fast gicp ptr is bull, but receive input target";  
    return false;
  }
    
  fast_gicp_ptr_->setInputTarget(input_target);

  if (!is_warmedup_)
    WarmUp(input_target);

  return true;
}

bool FastGICPRegistration::ScanMatch(const CloudPtr &input_source,
                                     const Eigen::Matrix4f &predict_pose,
                                     CloudPtr result_cloud_ptr,
                                     Eigen::Matrix4f &result_pose) {
  if (!fast_gicp_ptr_) {
    LOG(INFO) << "fast gicp ptr is null, but receive input source";
    return false;
  }

  {
    //Timer t("FastGICP::SetInputSource", 40);
    fast_gicp_ptr_->setInputSource(input_source);
  }

  {
    //Timer t("FastGICP::Align", 40);
    fast_gicp_ptr_->align(*result_cloud_ptr, predict_pose);
  }


  result_pose = fast_gicp_ptr_->getFinalTransformation();

  return fast_gicp_ptr_->hasConverged();
}

float FastGICPRegistration::GetFitnessScore() {
  if (!fast_gicp_ptr_) {
    return 0.0f;
  }
  
  return fast_gicp_ptr_->getFitnessScore();
}

float FastGICPRegistration::GetTransformationProbaility() {
  if (!fast_gicp_ptr_) {
    return 0.0f;
  }
  
  return fast_gicp_ptr_->getFitnessScore();
}

float FastGICPRegistration::GetFinalIterNum() {
  if (!fast_gicp_ptr_) {
    return 0.0f;
  }

  return fast_gicp_ptr_->getFinalNumIteration();
}

bool FastGICPRegistration::HasConverged() {
  if (!fast_gicp_ptr_) {
    return false;
  }

  return fast_gicp_ptr_->hasConverged();
}

    bool FastGICPRegistration::WarmUp(const CloudPtr &input_target)
{
  if (!fast_gicp_ptr_ || !input_target) {
    return false;
  }

  PointCloudType::Ptr dummy(new PointCloudType);
  pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud(input_target);
  vg.setLeafSize(1.0f, 1.0f, 1.0f); // 预热用可设大一点
  vg.filter(*dummy);

  fast_gicp_ptr_->setInputSource(dummy);
  PointCloudType out;
  fast_gicp_ptr_->align(out, Eigen::Matrix4f::Identity());
  fast_gicp_ptr_->setInputSource(nullptr); // 清掉dummy，之后再设置真实 source

  is_warmedup_ = true;

  return true;
}
} // namespace localization