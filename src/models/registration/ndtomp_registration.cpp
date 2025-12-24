/**
 * ************************************************************************
 * 
 * @file ndtomp_registration.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief NDTOMP匹配
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2025 
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include "models/registration/ndtomp_registration.hpp"

namespace localization {
NDTOMPRegistration::NDTOMPRegistration(const YAML::Node& yaml)
  :ndtomp_ptr_(new pclomp::NormalDistributionsTransform<PointType, PointType>()) {
  float res = yaml["ndt"]["resolution"].as<float>();
  float step_size = yaml["ndt"]["step_size"].as<float>();
  float trans_eps = yaml["ndt"]["transformation_epsilon"].as<float>();
  int max_iter = yaml["ndt"]["max_iter"].as<int>();

  SetRegistrationParam(res, step_size, trans_eps, max_iter);


}

NDTOMPRegistration::NDTOMPRegistration(float res, float step_size, float trans_eps, int max_iter)
  :ndtomp_ptr_(new pclomp::NormalDistributionsTransform<PointType, PointType>()) {
  SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTOMPRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
  ndtomp_ptr_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndtomp_ptr_->setResolution(res);
  ndtomp_ptr_->setStepSize(step_size);
  ndtomp_ptr_->setTransformationEpsilon(trans_eps);
  ndtomp_ptr_->setMaximumIterations(max_iter);

  LOG(INFO) << "初始化新NDTOMP, 匹配参数为:\n"
            << "res: " << res << "\n"
            << "step_size: " << step_size << "\n"
            << "trans_eps: " << trans_eps << "\n"
            << "max_iter: " << max_iter << "\n"
            << std::endl;

  return true;
}

bool NDTOMPRegistration::SetInputTarget(const CloudPtr& input_target) {
  ndtomp_ptr_->setInputTarget(input_target);

  if (!is_warmedup_)
    WarmUp(input_target);

  return true;
}

bool NDTOMPRegistration::WarmUp(const CloudPtr& input_target) {
  //预热ndt
  PointCloudType::Ptr dummy(new PointCloudType);
  pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud(input_target);
  vg.setLeafSize(1.0f, 1.0f, 1.0f); // 预热用可设大一点
  vg.filter(*dummy);

  ndtomp_ptr_->setInputSource(dummy);
  ndtomp_ptr_->setMaximumIterations(1); // 只跑一次迭代触发构建
  PointCloudType out;
  ndtomp_ptr_->align(out, Eigen::Matrix4f::Identity());
  ndtomp_ptr_->setInputSource(nullptr); // 清掉dummy，之后再设置真实 source

  is_warmedup_ = true;

  return true;
}

bool NDTOMPRegistration::ScanMatch(const CloudPtr& input_source,
          const Eigen::Matrix4f& predict_pose,
          CloudPtr result_cloud_ptr,
          Eigen::Matrix4f& result_pose) { 
  ndtomp_ptr_->setInputSource(input_source);
  ndtomp_ptr_->align(*result_cloud_ptr, predict_pose);
  result_pose = ndtomp_ptr_->getFinalTransformation();

  return true;
}

float NDTOMPRegistration::GetFitnessScore() {
  return ndtomp_ptr_->getFitnessScore();
}

float NDTOMPRegistration::GetTransformationProbaility() {
  return ndtomp_ptr_->getTransformationProbability();
}

} //namespace localization