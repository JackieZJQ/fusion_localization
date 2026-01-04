#include "fusion/fusion.hpp"

namespace localization {
Fusion::Fusion(const YAML::Node& yaml)
  : yaml_(yaml) {

  registration_manager_ptr_ = std::make_shared<RegistrationManager>(yaml_);
  tile_manager_ptr_ = std::make_shared<TileManager>(yaml_);

  InitConfig();
  InitIMU();

  //ui初始化
  ui_ptr_ = std::make_shared<ui::PangolinWindow>();
  ui_ptr_->Init();
  ui_ptr_->SetCurrentScanSize(50);
}

bool Fusion::InitConfig() {
  //地图原点
  auto origin_data = yaml_["origin"].as<std::vector<double>>();
  map_origin_ = Vec3d(origin_data[0], origin_data[1], origin_data[2]);

  //lidar和IMU消息同步
  //捕获此类的ProcessMeasurements, 传递给MessageSync类
  sync_ptr_ = std::make_shared<MessageSync>([this](const MessageSync::MeasureGroup &m) { ProcessMeasurements(m); });

  //lidar和IMU外参
  std::vector<double> ext_t = yaml_["mapping"]["extrinsic_T"].as<std::vector<double>>();
  std::vector<double> ext_r = yaml_["mapping"]["extrinsic_R"].as<std::vector<double>>();
  Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
  Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
  TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);

  return true;
}

bool Fusion::InitIMU() {
  StaticIMUInit::Options imu_init_options;
  imu_init_options.use_speed_for_static_checking_ = false; //暂时不用轮速计
  imu_init_ = StaticIMUInit(imu_init_options);

  return true;
}

void Fusion::ProcessMeasurements(const MessageSync::MeasureGroup& meas) {
  //Timer timer("Fusion::ProcessMeasurements");
  measures_ = meas;

  if (imu_need_init_) {
    InitImuOffline();
    return;
  }

  //以下三步与LIO一致，只是align完成地图匹配工作
  if (status_ == Status::WORKING) {
    Predict();
    Undistort();
  } else {
    scan_undistort_ = measures_.lidar_;
    scan_undistort_->header.stamp = static_cast<uint64_t>(measures_.lidar_begin_time_ * 1e6);
  }

  Align();
}

void Fusion::InitImuOnline() {
  for (auto imu : measures_.imu_) {
    //每一次ADDIMU后，都会计算是否符合初始化条件，如果符合，则计算IMU初始化数据
    imu_init_.AddIMU(*imu); 
  }

  if (imu_init_.InitSuccess()) {
    //读取初始零偏，设置ESKF
    localization::ESKFD::Options options;
    // 噪声由初始化器估计
    // options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
    // options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
    options.update_bias_acce_ = false;
    options.update_bias_gyro_ = false;
    eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
    imu_need_init_ = false;

    LOG(INFO) << "IMU初始化成功";

    //todo
    //imu初始化成功后，把数据记录于yaml文件中
    

  }
}

void Fusion::InitImuOffline() {
  std::vector<double> init_bg_array = yaml_["imu"]["init_bg"].as<std::vector<double>>();
  std::vector<double> init_ba_array = yaml_["imu"]["init_ba"].as<std::vector<double>>();
  std::vector<double> gravity_array = yaml_["imu"]["gravity"].as<std::vector<double>>();
  Vec3d init_bg = math::VecFromArray(init_bg_array);
  Vec3d init_ba = math::VecFromArray(init_ba_array);
  Vec3d gravity = math::VecFromArray(gravity_array);

  // 读取初始零偏，设置ESKF
  localization::ESKFD::Options options;
  
  options.update_bias_acce_ = false;
  options.update_bias_gyro_ = false;
  eskf_.SetInitialConditions(options, init_bg, init_ba, gravity);
  
  imu_need_init_ = false;

  LOG(INFO) << "IMU初始化成功";
}

void Fusion::Predict() {
  imu_states_.clear();
  imu_states_.emplace_back(eskf_.GetNominalState());

  //对IMU状态进行预测
  for (auto& imu : measures_.imu_) {
    eskf_.Predict(*imu);
    imu_states_.emplace_back(eskf_.GetNominalState());

    //todo
    //需要每一次predict()一次，发布一次状态吗？
  }
}

void Fusion::Undistort() {
  auto cloud = measures_.lidar_;
  auto imu_state = eskf_.GetNominalState();  //最后时刻的状态
  SE3 T_end = SE3(imu_state.R_, imu_state.p_);

  //将所有点转到最后时刻状态上
  std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](auto& pt) {
    SE3 Ti = T_end;
    NavStated match;

    //根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
    math::PoseInterp<NavStated>(
      measures_.lidar_begin_time_ + pt.time * 1e-3, imu_states_, [](const NavStated& s) { return s.timestamp_; },
      [](const NavStated& s) { return s.GetSE3(); }, Ti, match);

    Vec3d pi = ToVec3d(pt);
    Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

    pt.x = p_compensate(0);
    pt.y = p_compensate(1);
    pt.z = p_compensate(2);
  });

  scan_undistort_ = cloud;

}

void Fusion::Align() {
  FullCloudPtr scan_undistort_trans(new FullPointCloudType);
  pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix());
  scan_undistort_ = scan_undistort_trans;
  current_scan_ = ConvertToCloud<FullPointType>(scan_undistort_);
  current_scan_ = VoxelCloud(current_scan_, 0.5);

  if (status_ == Status::WAITING_FOR_RTK) {
    //若存在最近的RTK信号，则尝试初始化
    if (last_gnss_ != nullptr) {
      if (SearchRTK()) {
        status_ == Status::WORKING;
        
        ui_ptr_->UpdateScan(current_scan_, eskf_.GetNominalSE3());
        ui_ptr_->UpdateNavState(eskf_.GetNominalState());
      }
    }
  } else {
    LidarLocalization();
    
    ui_ptr_->UpdateScan(current_scan_, eskf_.GetNominalSE3());
    ui_ptr_->UpdateNavState(eskf_.GetNominalState());
  }
}

bool Fusion::SearchRTK() {
  if (init_has_failed_) {
    if ((last_gnss_->utm_pose_.translation() - last_searched_pos_.translation()).norm() < 20.0) {
      LOG(INFO) << "skip this position";
      return false;
    }
  }

  //由于RTK不带姿态，我们必须先搜索一定的角度范围
  std::vector<GridSearchResult> search_poses;
  tile_manager_ptr_->UpdateCurrentPose(last_gnss_->utm_pose_);
  
  if (!tile_manager_ptr_->HasMapInitialized()) {
    LOG(INFO) << "map uninitialized";
    return false;
  }

  if (tile_manager_ptr_->HasMapChanged()) {
    CloudPtr ref_cloud = tile_manager_ptr_->GetRefCloud();
    std::map<Vec2i, CloudPtr, less_vec<2>> map_data = tile_manager_ptr_->GetLoadedTiles();

    registration_manager_ptr_->UpdateRefCloud(ref_cloud);
    ui_ptr_->UpdatePointCloudGlobal(map_data);
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
    status_ = Status::WORKING;

    //重置滤波器状态
    auto state = eskf_.GetNominalState();
    state.R_ = max_ele->result_pose_.so3();
    state.p_ = max_ele->result_pose_.translation();
    state.v_.setZero();
    eskf_.SetX(state, eskf_.GetGravity());

    ESKFD::Mat18T cov;
    cov = ESKFD::Mat18T::Identity() * 1e-4;
    cov.block<12, 12>(6, 6) = Eigen::Matrix<double, 12, 12>::Identity() * 1e-6;
    eskf_.SetCov(cov);

    return true;
  }

  init_has_failed_ = true;
  last_searched_pos_ = last_gnss_->utm_pose_;
  return false;
}

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

bool Fusion::LidarLocalization() {
  SE3 pred = eskf_.GetNominalSE3();
  tile_manager_ptr_->UpdateCurrentPose(pred);
  if (tile_manager_ptr_->HasMapChanged()) {
    CloudPtr ref_cloud = tile_manager_ptr_->GetRefCloud();
    std::map<Vec2i, CloudPtr, less_vec<2>> map_data = tile_manager_ptr_->GetLoadedTiles();

    registration_manager_ptr_->UpdateRefCloud(ref_cloud);
    ui_ptr_->UpdatePointCloudGlobal(map_data);
  }

  Eigen::Matrix4f pred_pose = pred.matrix().cast<float>();
  Eigen::Matrix4f result_pose;
  registration_manager_ptr_->Align(current_scan_, pred_pose, result_pose);

  SE3 pose_se3 = Mat4ToSE3(result_pose);
  eskf_.ObserveSE3(pose_se3, 1e-1, 1e-2);

  return true;
}

void Fusion::ProcessIMU(IMU::Ptr imu) { 
  sync_ptr_->ProcessIMU(imu); 
}

void Fusion::ProcessPointCloud(CLOUD::Ptr cloud) {
  sync_ptr_->ProcessCloud(cloud);
}

void Fusion::ProcessRTK(GNSS::Ptr gnss) {
  gnss->utm_pose_.translation() -= map_origin_; //减掉地图原点
  last_gnss_ = gnss;
}

NavStated::Ptr Fusion::GetCurrentState() const {
  if (status_ == Status::WORKING) {
    return std::make_shared<NavStated>(eskf_.GetNominalState());
  } else {
    //todo
    //未初始化下，应该执行的操作
  }
}

FullCloudPtr Fusion::GetCurrentScan() const {
  return scan_undistort_;
}

TileManager::Ptr Fusion::GetTileManager() const {
  return tile_manager_ptr_;
}
}  //namespace localization