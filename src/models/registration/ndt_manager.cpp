#include "models/registration/ndt_manager.hpp"

namespace localization {
NdtManager::NdtManager(const YAML::Node& yaml) {

  //主ndt_初始化
  ndt_.reset(new pcl::NormalDistributionsTransform<PointType, PointType>);

  //加载yaml参数
  double resolution = yaml["ndt"]["resolution"].as<double>();
  double trans_eps = yaml["ndt"]["transformation_epsilon"].as<double>();
  double step_size = yaml["ndt"]["step_size"].as<double>();
  int max_iter = yaml["ndt"]["max_iter"].as<int>();

  SetOptions(ndt_, resolution, trans_eps, step_size, max_iter);
  
  is_running_ = true;
  worker_ = std::thread(&NdtManager::WorkerThreadLoop, this); 
}

NdtManager::~NdtManager() {
  Stop();
}

void NdtManager::Stop() {
  if (!is_running_) return;

  is_running_ = false;
  if (worker_.joinable()) worker_.join();
}

void NdtManager::UpdateRefCloud(const CloudPtr& ref_cloud) {
  std::lock_guard<std::mutex> mutex(ref_cloud_mutex_);
  ref_cloud_ = ref_cloud;
  ref_cloud_updated_.store(true, std::memory_order_release);
}

bool NdtManager::Align(const CloudPtr& cloud, const Eigen::Matrix4f& predict_pose, Eigen::Matrix4f& result_pose) {
  std::lock_guard<std::mutex> mutex(ndt_mutex_);

  ndt_->setInputSource(cloud);

  CloudPtr output(new PointCloudType);
  ndt_->align(*output, predict_pose);

  result_pose = ndt_->getFinalTransformation();

  //todo
  //设置阈值
  // double temp = 1.0;
  // if (ndt_->getFitnessScore() < temp) {
  //   return false;
  // } else {
  //   pose = ndt_->getFinalTransformation();
  //   return true;
  // }

  return true;
}

double NdtManager::GetFitnessScore() {
  return ndt_->getTransformationProbability();
}

void NdtManager::SetOptions(Registration::Ptr ndt, double resolution, double trans_eps, double step_size, int max_iter) {
  ndt->setResolution(resolution);
  ndt->setTransformationEpsilon(trans_eps);
  ndt->setStepSize(step_size);
  ndt->setMaximumIterations(max_iter);
}

void NdtManager::WorkerThreadLoop() {
  LOG(INFO) << "NdtManager worker started.";

  while (is_running_) {
    //睡眠50ms
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!ref_cloud_updated_.load(std::memory_order_acquire)) continue;

    //获取ref_cloud
    CloudPtr ref_cloud;
    {
      std::lock_guard<std::mutex> mutex(ref_cloud_mutex_);
      ref_cloud = ref_cloud_;
      ref_cloud_updated_.store(false, std::memory_order_release);
    }

    //重置secondary ndt
    ndt_secondary_.reset(new pcl::NormalDistributionsTransform<PointType, PointType>);

    //设置ndt参数
    SetOptions(
      ndt_secondary_,
      ndt_->getResolution(), 
      ndt_->getTransformationEpsilon(),
      ndt_->getStepSize(),
      ndt_->getStepSize());

    ndt_secondary_->setInputTarget(ref_cloud);

    int max_iteration = ndt_secondary_->getMaximumIterations();
    // 预热secondary ndt
    PointCloudType::Ptr dummy(new PointCloudType);
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(ref_cloud);
    vg.setLeafSize(1.0f, 1.0f, 1.0f); //预热用可设大一点
    vg.filter(*dummy);

    ndt_secondary_->setInputSource(dummy);
    ndt_secondary_->setMaximumIterations(1); //只跑一次迭代触发构建
    PointCloudType out;
    ndt_secondary_->align(out, Eigen::Matrix4f::Identity());
    ndt_secondary_->setInputSource(nullptr); //清掉dummy，之后再设置真实 source
    ndt_secondary_->setMaximumIterations(max_iteration);

    //切换ndt_与ndt_secondary
    {
      std::lock_guard<std::mutex> mutex(ndt_mutex_);

      ndt_.swap(ndt_secondary_);
    }
  }

  LOG(INFO) << "NdtManager worker stopped.";
}

} // namespace localization