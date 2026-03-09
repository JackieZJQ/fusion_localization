/**
 * ************************************************************************
 * 
 * @file fusion.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 融合定位核心流程
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include "fusion/fusion.hpp"
#include "glog/logging.h"

namespace localization {
Fusion::Fusion(const YAML::Node& yaml)
  : yaml_(yaml) {

  registration_manager_ptr_ = std::make_shared<RegistrationManager>(yaml_);
  tile_manager_ptr_ = std::make_shared<TileManager>(yaml_);

  InitConfig();
  InitImu();
}

bool Fusion::InitConfig() {
  //地图原点
  auto origin_data = yaml_["origin"].as<std::vector<double>>();
  map_origin_ = Vec3d(origin_data[0], origin_data[1], origin_data[2]);

  //lidar和IMU消息同步
  //捕获此类的ProcessMeasurements, 传递给MessageSync类
  sync_ptr_ = std::make_shared<MessageSync>([this](const MessageSync::MeasureGroup &m) { 
    ProcessMeasurements(m); 
  });

  //lidar和IMU外参
  std::vector<double> ext_t = yaml_["mapping"]["extrinsic_T"].as<std::vector<double>>();
  std::vector<double> ext_r = yaml_["mapping"]["extrinsic_R"].as<std::vector<double>>();
  Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
  Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
  TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);

  return true;
}

bool Fusion::InitImu() {
  bool init_imu_online = yaml_["imu"]["init_imu_online"].as<bool>();
  if (init_imu_online) {
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;
    imu_init_ = StaticIMUInit(imu_init_options);
    // state_ 保持 kNOT_READY，等在线初始化完成
    return true;
  } else {
    InitImuOffline();
    // ★ 离线初始化完成，直接切状态
    TransitionTo(State::kWAITING_FOR_RTK);
    return true;
  }
}

void Fusion::ProcessMeasurements(const MessageSync::MeasureGroup& meas) {
  synced_measures_ = meas;

  switch (state_) {
    case State::kNOT_READY:
      InitImuOnline();
      break;

    case State::kWAITING_FOR_RTK:
      PrepareCurrentScan();
      if (last_gnss_ != nullptr) {
        TransitionTo(State::kSEARCHING);
        // 直接尝试搜索
        if (SearchRtk()) {
          TransitionTo(State::kINITIALIZED);
        }
      }
      break;

    case State::kSEARCHING:
      PrepareCurrentScan();
      if (SearchRtk()) {
        TransitionTo(State::kINITIALIZED);
      }
      break;

    case State::kINITIALIZED:
      PrepareCurrentScan();
      if (DoLidarLocalization()) {
        TransitionTo(State::kWORKING);
      }
      break;

    case State::kWORKING:
      EskfPredict();
      DoUndistort();
      PrepareCurrentScan();
      DoLidarLocalization();
      break;

    case State::kLOST:
      // 预留
      break;
  }
}

void Fusion::InitImuOnline() {
  for (auto imu : synced_measures_.imu_) {
    imu_init_.AddIMU(*imu);
  }

  if (imu_init_.InitSuccess()) {
    localization::ESKFD::Options options;
    options.update_bias_acce_ = false;
    options.update_bias_gyro_ = false;
    eskf_.SetInitialConditions(options, imu_init_.GetInitBg(),
                               imu_init_.GetInitBa(), 
                               imu_init_.GetGravity());

    LOG(INFO) << "IMU在线初始化成功";
    // ★ 不再用 imu_need_init_，直接切状态
    TransitionTo(State::kWAITING_FOR_RTK);
  }
}

void Fusion::InitImuOffline() {
  std::vector<double> init_bg_array = yaml_["imu"]["init_bg"].as<std::vector<double>>();
  std::vector<double> init_ba_array = yaml_["imu"]["init_ba"].as<std::vector<double>>();
  std::vector<double> gravity_array = yaml_["imu"]["gravity"].as<std::vector<double>>();
  Vec3d init_bg = math::VecFromArray(init_bg_array);
  Vec3d init_ba = math::VecFromArray(init_ba_array);
  Vec3d gravity = math::VecFromArray(gravity_array);

  localization::ESKFD::Options options;
  options.update_bias_acce_ = false;
  options.update_bias_gyro_ = false;
  eskf_.SetInitialConditions(options, init_bg, init_ba, gravity);

  // ★ 删掉 imu_need_init_ = false; 状态转换由调用方负责

  LOG(INFO) << "\n===============使用YAML初始化IMU成功========================\n"
            << "Init Bg: " << init_bg.transpose() << "\n"
            << "Init Ba: " << init_ba.transpose() << "\n"
            << "Gravity: " << gravity.transpose() << "\n"
            << "============================================================";
}

void Fusion::EskfPredict() {

  // 主滤波器用该雷达帧对应的 IMU 段数据预测到雷达帧结束时刻的状态，供雷达去畸变和配准使用
  // 1.清理imu_states_，重新预测并记录状态
  imu_states_.clear();
  imu_states_.emplace_back(eskf_.GetNominalState());

  // 2.使用imu数据对主eskf递推，记录每个imu时刻的状态
  for (auto& imu : synced_measures_.imu_) {
    eskf_.Predict(*imu);
    imu_states_.emplace_back(eskf_.GetNominalState());
  }
}

void Fusion::DoUndistort() {
  auto cloud = synced_measures_.lidar_;
  auto imu_state = eskf_.GetNominalState();  //最后时刻的状态
  SE3 T_end = SE3(imu_state.R_, imu_state.p_);

  //将所有点转到最后时刻状态上
  std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](auto& pt) {
    SE3 Ti = T_end;
    NavStated match;

    //根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
    math::PoseInterp<NavStated>(
      synced_measures_.lidar_begin_time_ + pt.time * 1e-3, imu_states_, [](const NavStated& s) { return s.timestamp_; },
      [](const NavStated& s) { return s.GetSE3(); }, Ti, match);

    Vec3d pi = ToVec3d(pt);
    Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

    pt.x = p_compensate(0);
    pt.y = p_compensate(1);
    pt.z = p_compensate(2);
  });

  undistorted_scan_ = cloud;

}

bool Fusion::SearchRtk() {
  if (init_failed_) {
    if ((last_gnss_->utm_pose_.translation() - last_searched_pose_.translation()).norm() < 20.0) {
      LOG(INFO) << "skip this position.";
      return false;
    }
  }

  //todo
  //使用带姿态的Rtk进行Ndt的初始化

  //由于RTK不带姿态，我们必须先搜索一定的角度范围
  std::vector<GridSearchResult> search_poses;
  tile_manager_ptr_->UpdateCurrentPose(last_gnss_->utm_pose_);
  
  //todo
  //tile_manager_ptr是不是可以单独放在其他一个函数/线程，单独检测tile_manager的状态
  if (!tile_manager_ptr_->HasMapInitialized()) return false;
  else {
    LOG(INFO) << "\n==============tile pointcloud map initialized==============\n"
              << "===========================================================";
  }

  if (tile_manager_ptr_->HasMapChanged()) {
    CloudPtr ref_cloud = tile_manager_ptr_->GetRefCloud();
    std::map<Vec2i, CloudPtr, less_vec<2>> map_data = tile_manager_ptr_->GetLoadedTiles();

    //todo
    //此处不用一直初始化Ndt吧？应该初始化一次就行了
    registration_manager_ptr_->UpdateRefCloud(ref_cloud);
  }

  //由于RTK不带角度，这里按固定步长扫描RTK角度
  double grid_ang_range = 360.0, grid_ang_step = 10;  // 角度搜索范围与步长
  for (double ang = 0; ang < grid_ang_range; ang += grid_ang_step) {
    SE3 pose(SO3::rotZ(ang * math::kDEG2RAD), Vec3d(0, 0, 0) + last_gnss_->utm_pose_.translation());
    GridSearchResult gr;
    gr.pose_ = pose;
    search_poses.emplace_back(gr);
  }

  LOG(INFO) << "grid search poses: " << search_poses.size();
  std::for_each(std::execution::par_unseq, search_poses.begin(), search_poses.end(),
                [this](GridSearchResult& gr) { AlignForGrid(gr); });

  //选择最优的匹配结果
  auto max_ele = std::max_element(search_poses.begin(), search_poses.end(),
                                  [](const auto& g1, const auto& g2) { return g1.score_ < g2.score_; });
  LOG(INFO) << "max score: " << max_ele->score_ << ", pose: \n" << max_ele->result_pose_.matrix();
  
  if (max_ele->score_ > rtk_search_min_score_) {
      LOG(INFO) << "初始化成功, score: " << max_ele->score_ << ">" << rtk_search_min_score_;

      // 重置滤波器状态
      auto nav = eskf_.GetNominalState();
      nav.R_ = max_ele->result_pose_.so3();
      nav.p_ = max_ele->result_pose_.translation();
      nav.v_.setZero();
      nav.timestamp_ = synced_measures_.lidar_end_time_;  // ★ 修复根因！
      eskf_.SetX(nav, eskf_.GetGravity());

      ESKFD::Mat18T cov;
      cov = ESKFD::Mat18T::Identity() * 1e-4;
      cov.block<12, 12>(6, 6) = Eigen::Matrix<double, 12, 12>::Identity() * 1e-6;
      eskf_.SetCov(cov);

      return true;
  }

  init_failed_ = true;
  last_searched_pose_ = last_gnss_->utm_pose_;
  return false;
}

//todo
//使用ndt_omp/ndt_gpu提高配准速度
void Fusion::AlignForGrid(GridSearchResult& gr) {
  //多分辨率
  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  ndt.setTransformationEpsilon(0.05);
  ndt.setStepSize(0.7);
  ndt.setMaximumIterations(40);

  ndt.setInputSource(current_scan_);
  auto map = tile_manager_ptr_->GetRefCloud();

  CloudPtr output(new PointCloudType);
  std::vector<double> res{10.0, 5.0, 4.0, 3.0};
  Mat4f T = gr.pose_.matrix().cast<float>();
  for (auto& r : res) {
    auto rough_map = VoxelCloud(map, r * 0.1);
    ndt.setInputTarget(rough_map);
    ndt.setResolution(r);
    ndt.align(*output, T);
    T = ndt.getFinalTransformation();
  }

  gr.score_ = ndt.getTransformationProbability();
  gr.result_pose_ = Mat4ToSE3(ndt.getFinalTransformation());
}

bool Fusion::DoLidarLocalization() {
  SE3 pred = eskf_.GetNominalSE3();
  tile_manager_ptr_->UpdateCurrentPose(pred);
  if (tile_manager_ptr_->HasMapChanged()) {
    CloudPtr ref_cloud = tile_manager_ptr_->GetRefCloud();
    std::map<Vec2i, CloudPtr, less_vec<2>> map_data = tile_manager_ptr_->GetLoadedTiles();

    registration_manager_ptr_->UpdateRefCloud(ref_cloud);
  }

  Eigen::Matrix4f pred_pose = pred.matrix().cast<float>();
  Eigen::Matrix4f result_pose;

  double th = 40;
  double elapsed_ms;
  {
    Timer t("FastGICP::Align", th);
    bool converged = registration_manager_ptr_->Align(current_scan_, pred_pose, result_pose);

    elapsed_ms = t.Stop();
    //LOG(INFO) << "elapsed_ms: " << elapsed_ms;
  }

  if (elapsed_ms > th) {
    float dx = (result_pose.block<3, 1>(0, 3) - pred_pose.block<3, 1>(0, 3)).norm();

    LOG(WARNING) << "\niter=" << registration_manager_ptr_->GetFinalIterNum()
              << " converged=" << registration_manager_ptr_->HasConverged()
              << " fitness=" << registration_manager_ptr_->GetFitnessScore()
              << " pred->res`ult delta = " << dx
              << "\n==================================";
  }

  SE3 pose_se3 = Mat4ToSE3(result_pose);
  eskf_.ObserveSE3(pose_se3, 1e-1, 1e-2);

  //更新eskf定位预测器
  inertial_extrapolator_.CorrectState(eskf_, synced_measures_.lidar_end_time_);

  return true;
}

void Fusion::ProcessImu(IMU::Ptr imu) { 
  // 1. 始终给消息同步器
  sync_ptr_->ProcessIMU(imu);

  // 2. 只有 kWORKING 状态才给 extrapolator
  if (state_ != State::kWORKING) return;
  //inertial_extrapolator_.PushImu(imu);  // ★ 取消注释！
}

void Fusion::ProcessPointCloud(CLOUD::Ptr cloud) {
  sync_ptr_->ProcessCloud(cloud);
}

void Fusion::ProcessRtk(GNSS::Ptr gnss) {
  gnss->utm_pose_.translation() -= map_origin_; //减掉地图原点
  last_gnss_ = gnss;
}

bool Fusion::IsImuReplaying() {
  return inertial_extrapolator_.IsReplaying();
}

const char* Fusion::StateToString(State s) {
  switch (s) {
    case State::kNOT_READY:       return "NOT_READY";
    case State::kWAITING_FOR_RTK: return "WAITING_FOR_RTK";
    case State::kSEARCHING:       return "SEARCHING";
    case State::kINITIALIZED:     return "INITIALIZED";
    case State::kWORKING:         return "WORKING";
    case State::kLOST:            return "LOST";
    default:                      return "UNKNOWN";
  }
}

void Fusion::TransitionTo(State new_state) {
  LOG(INFO) << "状态转换: " << StateToString(state_) << " -> " << StateToString(new_state);

  switch (new_state) {
    case State::kWAITING_FOR_RTK:
      // IMU初始化完成，无特殊操作
      break;

    case State::kINITIALIZED: {
      // ★ SearchRtk 成功后，修正 eskf_ 的 current_time_
      auto nav = eskf_.GetNominalState();
      nav.timestamp_ = synced_measures_.lidar_end_time_;
      eskf_.SetX(nav, eskf_.GetGravity());
      break;
    }

    case State::kWORKING:
      // 首次配准成功，extrapolator 已在 DoLidarLocalization 中被 CorrectState
      break;

    default:
      break;
  }

  state_ = new_state;
}

void Fusion::PrepareCurrentScan() {
  // 非WORKING状态：不去畸变，直接用原始点云
  if (state_ != State::kWORKING) {
    undistorted_scan_ = synced_measures_.lidar_;
    undistorted_scan_->header.stamp =
        static_cast<uint64_t>(synced_measures_.lidar_begin_time_ * 1e6);
  }

  // 坐标转换 + 降采样（所有状态通用）
  FullCloudPtr scan_trans(new FullPointCloudType);
  pcl::transformPointCloud(*undistorted_scan_, *scan_trans, TIL_.matrix());
  undistorted_scan_ = scan_trans;
  current_scan_ = ConvertToCloud<FullPointType>(undistorted_scan_);
  current_scan_ = VoxelCloud(current_scan_, 0.5);
}

NavStated::Ptr Fusion::GetRegistrationState() const {
  if (state_ == State::kWORKING) {
    return std::make_shared<NavStated>(eskf_.GetNominalState());
  }
  return nullptr;
}

NavStated::Ptr Fusion::GetImuPredictedState() const {
  if (state_ == State::kWORKING) {
    return std::make_shared<NavStated>(inertial_extrapolator_.GetState());
  }
  return nullptr;
}

FullCloudPtr Fusion::GetUndistorScan() const {
  return undistorted_scan_;
}

CloudPtr Fusion::GetStaticPointcloudMap() const {
  return tile_manager_ptr_->GetStaticPointcloudMap();
}
}  //namespace localization