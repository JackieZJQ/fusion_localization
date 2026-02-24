/**
 * ************************************************************************
 * 
 * @file fusion.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 融合定位核心流程
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>
#include <execution>

#include "models/kalmanfilter/eskf.hpp"
#include "models/initialization/static_imu_init.hpp"
#include "models/registration/registration_manager.hpp"
#include "models/synchronization/measure_sync.hpp"

#include "sensor_data/imu_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/nav_state.hpp"

#include "common/point_types.h"
#include "common/timer.hpp"
#include "common/lidar_utils.hpp"
#include "common/eigen_types.hpp"

#include "tiles/tile_manager.hpp"

namespace localization {
class Fusion {
public:
  Fusion() = delete;
  Fusion(const YAML::Node& yaml);

  ~Fusion() = default;

  bool InitConfigFromYaml();
  bool InitImu();

  //处理输入
  void ProcessRtk(GNSS::Ptr gnss);
  void ProcessImu(IMU::Ptr imu);
  void ProcessPointCloud(CLOUD::Ptr cloud);

  //获取当前状态
  NavStated::Ptr GetCurrentState() const;
  FullCloudPtr GetCurrentScan() const;
  CloudPtr GetStaticPointcloudMap() const;

  //RTK状态
  enum class state {
    kWAITINGFORRTK,  //等待初始的RTK
    kWORKING,        //正常工作
  };

  //网格搜索时的结构
  struct GridSearchResult {
    SE3 pose_;
    SE3 result_pose_;
    double score_ = 0.0;
  };

  using Ptr = std::shared_ptr<localization::Fusion>;
  
private:
  void ProcessMeasurements(const MessageSync::MeasureGroup& meas);   //处理同步之后的IMU和雷达数据

  bool SearchRtk();   //在初始RTK附近搜索车辆位置

  void AlignForGrid(GridSearchResult& gr);   //对网格搜索的某个点进行配准，得到配准后位姿与配准分值

  void InitImuOnline();   //在线估计IMU初始零偏

  void InitImuOffline();  //离线估计IMU初始零偏，使用yaml中的配置

  //激光定位
  bool DoLidarLocalization();   

  //利用IMU预测状态信息,这段时间的预测数据会放入imu_states_里
  void DoPredict();

  //对measures_中的点云去畸变
  void DoUndistort();

  //执行一次配准和观测
  void DoAlign();

  //标志位
  state state_ = state::kWAITINGFORRTK;

  //地图原点
  Vec3d map_origin_ = Vec3d::Zero();                

  std::shared_ptr<MessageSync> sync_ptr_ = nullptr;  //消息同步器
  StaticIMUInit imu_init_;                           //IMU静止初始化

  //滤波器
  ESKFD eskf_;
  ESKFD eskf_realtime_;
  std::vector<NavStated> imu_states_;  //ESKF预测期间的状态

  FullCloudPtr undistorted_scan_ { new FullPointCloudType() }; //矫过畸变之后的点云
  CloudPtr current_scan_ = nullptr;

  SE3 TIL_;
  MessageSync::MeasureGroup synced_measures_;         //同步IMU与雷达扫描
  GNSS::Ptr last_gnss_ = nullptr;

  bool init_failed_ = false;  //RTK初始化是否失败过
  SE3 last_searched_pose_;    //上次搜索的RTK位置
  bool imu_need_init_ = true; //是否需要估计IMU初始零偏

  RegistrationManager::Ptr registration_manager_ptr_;

  //参数
  double rtk_search_min_score_ = 4.5;

  //点云地图区块管理
  TileManager::Ptr tile_manager_ptr_ = nullptr;

  YAML::Node yaml_; //参数配置
};
}  // namespace localization


