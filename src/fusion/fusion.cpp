#include "fusion/fusion.hpp"

namespace localization {
Fusion::Fusion(const YAML::Node& yaml)
  : yaml_(yaml) {

  registration_manager_ptr_ = std::make_shared<RegistrationManager>(yaml_);
  map_loader_ptr_ = std::make_shared<MapLoader>(yaml_);

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
    TryInitIMU();
    return;
  }

  //以下三步与LIO一致，只是align完成地图匹配工作
  if (status_ == Status::WORKING) {
    Predict();
    Undistort();
  } else {
    scan_undistort_ = measures_.lidar_;
  }

  Align();
}

void Fusion::TryInitIMU() {
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

// void Fusion::InitIMUwithYaml() {
//   std::vector<double> init_bg = yaml_["imu"]["init_bg"].as<std::vector<double>>();
//   std::vector<double> init_ba = yaml_["imu"]["init_ba"].as<std::vector<double>>();
//   std::vector<double> gravity = yaml_["imu"]["gravity"].as<std::vector<double>>();
//   Vec3d init__ = math::VecFromArray(init_bg);
//   Mat3d init__ = math::MatFromArray(init_ba);

//   // 读取初始零偏，设置ESKF
//   localization::ESKFD::Options options;
//   // 噪声由初始化器估计
//   // options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
//   // options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
//   options.update_bias_acce_ = false;
//   options.update_bias_gyro_ = false;
//   eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
// }

/**
 * @brief 使用IMU数据进行状态预测
 * 
 * 功能：
 * 1. 清空之前的预测状态列表
 * 2. 保存当前ESKF状态作为起始状态
 * 3. 对每个IMU数据调用ESKF预测
 * 4. 保存每次预测后的状态（用于点云去畸变）
 * 
 * 原理：
 * IMU频率高（100-200Hz），LiDAR扫描一帧需要时间（~100ms）
 * 在一帧点云扫描期间，会有多个IMU测量
 * 通过逐次预测，可以得到扫描期间的连续运动轨迹
 * 
 * 注意：
 * 预测状态存储在imu_states_中，供Undistort()函数使用
 */
void Fusion::Predict() {
  // 清空之前的预测状态
  imu_states_.clear();
  
  // 保存起始状态
  imu_states_.emplace_back(eskf_.GetNominalState());

  // 对每个IMU数据进行预测
  for (auto& imu : measures_.imu_) {
    // ESKF预测：根据IMU测量更新位置、速度、姿态
    eskf_.Predict(*imu);
    
    // 保存预测后的状态
    imu_states_.emplace_back(eskf_.GetNominalState());

    // TODO: 是否需要每次predict()后都发布一次状态？
  }
}

/**
 * @brief 对点云进行畸变矫正（运动补偿）
 * 
 * 功能：
 * 对点云中的每个点进行运动补偿，将其转换到扫描结束时刻的坐标系
 * 
 * 原理：
 * LiDAR扫描一帧需要时间（如100ms），在此期间车辆在运动
 * 不同时刻扫描的点在不同位置，直接使用会产生"拖影"（畸变）
 * 需要根据运动轨迹将所有点统一到同一时刻（通常是扫描结束时刻）
 * 
 * 算法流程：
 * 1. 获取扫描结束时刻的位姿 T_end
 * 2. 对每个点：
 *    a. 根据点的时间戳在imu_states_中插值得到该时刻位姿 Ti
 *    b. 计算补偿变换：TIL^-1 * T_end^-1 * Ti * TIL
 *    c. 将点从LiDAR坐标系变换到IMU坐标系，再应用补偿变换
 * 3. 更新点的坐标
 * 
 * 注意：
 * - pt.time: 点的时间戳（相对扫描开始时间，单位毫秒）
 * - TIL_: LiDAR到IMU的外参变换
 * - 使用并行算法（std::execution::par_unseq）加速处理
 */
void Fusion::Undistort() {
  auto cloud = measures_.lidar_;
  
  // 获取扫描结束时刻的IMU状态
  auto imu_state = eskf_.GetNominalState();
  SE3 T_end = SE3(imu_state.R_, imu_state.p_);

  // 并行处理：将所有点转换到扫描结束时刻
  std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](auto& pt) {
    SE3 Ti = T_end;  // 该点对应时刻的位姿
    NavStated match; // 插值匹配的状态

    // 根据点的时间戳在IMU状态序列中插值
    // pt.time: 该点相对扫描开始的时间（毫秒）
    math::PoseInterp<NavStated>(
      measures_.lidar_begin_time_ + pt.time * 1e-3,  // 点的绝对时间戳
      imu_states_,                                    // IMU预测状态序列
      [](const NavStated& s) { return s.timestamp_; }, // 获取状态时间戳
      [](const NavStated& s) { return s.GetSE3(); },   // 获取状态位姿
      Ti,                                             // 输出：插值得到的位姿
      match);                                         // 输出：匹配的状态

    // 计算点的坐标
    Vec3d pi = ToVec3d(pt);
    
    // 运动补偿变换：
    // TIL^-1: IMU -> LiDAR
    // T_end^-1: 结束时刻世界坐标系 -> 结束时刻IMU坐标系
    // Ti: 当前时刻IMU坐标系 -> 当前时刻世界坐标系
    // TIL: LiDAR -> IMU
    Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

    // 更新点坐标
    pt.x = p_compensate(0);
    pt.y = p_compensate(1);
    pt.z = p_compensate(2);
  });

  // 保存去畸变后的点云
  scan_undistort_ = cloud;
}

/**
 * @brief 执行点云配准并更新状态
 * 
 * 功能：
 * 1. 将去畸变的点云转换到IMU坐标系
 * 2. 点云降采样（体素滤波）
 * 3. 根据系统状态执行不同操作：
 *    - WAITING_FOR_RTK: 尝试RTK初始化（网格搜索）
 *    - WORKING: 执行正常的激光定位
 * 4. 更新可视化界面
 * 
 * 状态机：
 * WAITING_FOR_RTK -> (搜索成功) -> WORKING
 */
void Fusion::Align() {
  // 将点云从LiDAR坐标系转换到IMU坐标系
  FullCloudPtr scan_undistort_trans(new FullPointCloudType);
  pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix());
  scan_undistort_ = scan_undistort_trans;
  
  // 转换点云格式并进行体素滤波降采样（体素大小0.5m）
  current_scan_ = ConvertToCloud<FullPointType>(scan_undistort_);
  current_scan_ = VoxelCloud(current_scan_, 0.5);

  // 根据系统状态执行不同操作
  if (status_ == Status::WAITING_FOR_RTK) {
    // 初始化阶段：若存在最近的RTK信号，则尝试初始化
    if (last_gnss_ != nullptr) {
      if (SearchRTK()) {
        // 搜索成功，切换到工作状态
        status_ = Status::WORKING;
        
        // 更新可视化
        ui_ptr_->UpdateScan(current_scan_, eskf_.GetNominalSE3());
        ui_ptr_->UpdateNavState(eskf_.GetNominalState());
      }
    }
  } else {
    // 工作阶段：执行正常的激光定位
    LidarLocalization();
    
    // 更新可视化
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
  map_loader_ptr_->UpdatePose(last_gnss_->utm_pose_);
  
  if (!map_loader_ptr_->MapInitialized()) {
    LOG(INFO) << "map uninitialized";
    return false;
  }

  if (map_loader_ptr_->MapChanged()) {
    CloudPtr ref_cloud = map_loader_ptr_->GetRefCloud();
    std::map<Vec2i, CloudPtr, less_vec<2>> map_data = map_loader_ptr_->GetMapData();

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
  auto map = map_loader_ptr_->GetRefCloud();

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
  map_loader_ptr_->UpdatePose(pred);
  if (map_loader_ptr_->MapChanged()) {
    CloudPtr ref_cloud = map_loader_ptr_->GetRefCloud();
    std::map<Vec2i, CloudPtr, less_vec<2>> map_data = map_loader_ptr_->GetMapData();

    registration_manager_ptr_->UpdateRefCloud(ref_cloud);
    ui_ptr_->UpdatePointCloudGlobal(map_data);
  }

  Eigen::Matrix4f pred_pose = pred.matrix().cast<float>();
  Eigen::Matrix4f result_pose;
  registration_manager_ptr_->Align(current_scan_, pred_pose, result_pose);

  SE3 pose_se3 = Mat4ToSE3(result_pose);
  eskf_.ObserveSE3(pose_se3, 1e-1, 1e-2);

  // LOG(INFO) << "Lidar localization, transformation probaility: " << registration_manager_ptr_->GetTransformationProbaility()
  //           << ", fitness score: " << registration_manager_ptr_->GetFitnessScore() << "\n"

  // LOG(INFO) << "ndt  pose: " << pose_se3.translation().transpose()[0] << " " << pose_se3.translation().transpose()[1] << "\n";
  // LOG(INFO) << "gnss pose: " << last_gnss_->utm_pose_.translation().transpose()[0] << " " << last_gnss_->utm_pose_.translation().transpose()[1] << "\n";

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
  }
}

FullCloudPtr Fusion::GetCurrentScan() const {
  return scan_undistort_;
}

MapLoader::Ptr Fusion::GetMapLoader() const {
  return map_loader_ptr_;
}
}  //namespace localization