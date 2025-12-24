#include "models/registration/registration_manager.hpp"

namespace localization {
RegistrationManager::RegistrationManager(const YAML::Node& yaml)
  : yaml_(yaml) {

  // 主ndt_初始化
  registration_ptr_ = std::make_shared<localization::NDTOMPRegistration>(yaml_);

  is_running_ = true;
  worker_ = std::thread(&RegistrationManager::WorkerThreadLoop, this); 
}

RegistrationManager::~RegistrationManager() {
  Stop();
}

void RegistrationManager::Stop() {
  if (!is_running_) return;

  is_running_ = false;
  if (worker_.joinable()) worker_.join();
}

//需要一个存储是否初始化的变量
void RegistrationManager::UpdateRefCloud(const CloudPtr& ref_cloud_ptr) {
  std::lock_guard<std::mutex> mutex(ref_cloud_mutex_);
  ref_cloud_ptr_ = ref_cloud_ptr;
  ref_cloud_updated_.store(true, std::memory_order_release);
}

bool RegistrationManager::Align(const CloudPtr& cloud_ptr, const Eigen::Matrix4f& predict_pose, Eigen::Matrix4f& result_pose) {
  std::lock_guard<std::mutex> mutex(registration_mutex_);

  CloudPtr result_cloud_ptr(new PointCloudType);  
  bool result = registration_ptr_->ScanMatch(cloud_ptr, predict_pose, result_cloud_ptr, result_pose);

  return result;
}

float RegistrationManager::GetFitnessScore() {
  return registration_ptr_->GetFitnessScore();
}

float RegistrationManager::GetTransformationProbaility() {
  return registration_ptr_->GetTransformationProbaility();
}

void RegistrationManager::WorkerThreadLoop() {
  LOG(INFO) << "Registration manager worker started.";

  while (is_running_) {
    //睡眠50ms
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!ref_cloud_updated_.load(std::memory_order_acquire)) continue;

    //获取ref_cloud
    CloudPtr ref_cloud_ptr;
    {
      std::lock_guard<std::mutex> mutex(ref_cloud_mutex_);
      ref_cloud_ptr = ref_cloud_ptr_;
      ref_cloud_updated_.store(false, std::memory_order_release);
    }

    //重置registration_secondar ndt
    registration_secondary_ptr_ = std::make_shared<localization::NDTOMPRegistration>(yaml_);
    registration_secondary_ptr_->SetInputTarget(ref_cloud_ptr);

    //切换ndt_与ndt_secondary
    {
      std::lock_guard<std::mutex> mutex(registration_mutex_);
      registration_ptr_.swap(registration_secondary_ptr_);
      // LOG(INFO) << "Rebuild and switch registration.";
    }
  }

  LOG(INFO) << "Registration manager worker stopped.";
}
} // namespace localization