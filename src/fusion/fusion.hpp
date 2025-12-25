/**
 * ************************************************************************
 * 
 * @file fusion.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 高精度融合定位核心类
 * 
 * 本文件实现第10章展示的高精度融合定位功能，融合：
 * - IMU（惯性测量单元）：提供高频运动测量
 * - RTK（实时动态差分GPS）：提供高精度全球位置
 * - 激光点云定位：通过NDT配准提供高精度局部位置
 * 
 * 工作流程：
 * 1. 初始化阶段（WAITING_FOR_RTK）
 *    - IMU静态初始化：估计零偏和重力
 *    - RTK网格搜索：在RTK附近搜索准确的初始位置
 * 2. 工作阶段（WORKING）
 *    - 预测：使用IMU进行ESKF预测
 *    - 去畸变：补偿点云扫描期间的运动
 *    - 配准：NDT配准点云与地图
 *    - 更新：使用配准结果更新ESKF状态
 * 
 * NOTE: 一些IMU的异常处理没有加在这里，有可能会被IMU带歪
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2025
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
#include "models/registration/ndt_manager.hpp"
#include "models/synchronization/measure_sync.hpp"

#include "sensor_data/imu_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/nav_state.hpp"

#include "common/point_types.h"
#include "common/timer.hpp"
#include "common/lidar_utils.hpp"
#include "common/eigen_types.hpp"

#include "ui/pangolin_window.hpp"
#include "map/map_loader.hpp"

namespace localization {

/**
 * @class Fusion
 * @brief 多传感器融合定位核心类
 * 
 * 主要职责：
 * 1. 管理定位系统状态机（WAITING_FOR_RTK -> WORKING）
 * 2. 处理多传感器数据（IMU、RTK、LiDAR）
 * 3. 执行ESKF预测和更新
 * 4. 执行点云配准定位
 * 5. 提供定位结果
 */
class Fusion {
public:
  Fusion() = delete;  // 禁止默认构造
  
  /**
   * @brief 构造函数
   * @param yaml 配置文件节点
   * 
   * 初始化：
   * - 配准管理器
   * - 地图加载器
   * - IMU初始化器
   * - 可视化界面
   */
  Fusion(const YAML::Node& yaml);

  /**
   * @brief 初始化配置参数
   * @return 成功返回true
   * 
   * 从YAML配置文件加载：
   * - 地图原点坐标
   * - LiDAR-IMU外参
   * - 消息同步器配置
   */
  bool InitConfig();
  
  /**
   * @brief 初始化IMU
   * @return 成功返回true
   * 
   * 配置IMU静态初始化器选项
   */
  bool InitIMU();

  // ========== 传感器数据处理接口 ==========
  
  /**
   * @brief 处理RTK数据
   * @param gnss GNSS数据指针
   * 
   * 功能：
   * - 保存最新的GNSS数据
   * - 用于初始化时的网格搜索
   */
  void ProcessRTK(GNSS::Ptr gnss);
  
  /**
   * @brief 处理IMU数据
   * @param imu IMU数据指针
   * 
   * 功能：
   * - 送入消息同步器
   * - 等待与点云数据同步
   */
  void ProcessIMU(IMU::Ptr imu);
  
  /**
   * @brief 处理点云数据
   * @param cloud 点云数据指针
   * 
   * 功能：
   * - 送入消息同步器
   * - 触发同步数据的处理（ProcessMeasurements）
   */
  void ProcessPointCloud(CLOUD::Ptr cloud);

  // ========== 状态查询接口 ==========
  
  /**
   * @brief 获取当前导航状态
   * @return 导航状态指针（位置、速度、姿态等）
   */
  NavStated::Ptr GetCurrentState() const;
  
  /**
   * @brief 获取当前扫描点云
   * @return 点云指针
   */
  FullCloudPtr GetCurrentScan() const;
  
  /**
   * @brief 获取地图加载器
   * @return 地图加载器指针
   */
  MapLoader::Ptr GetMapLoader() const;

  /**
   * @enum Status
   * @brief 系统状态枚举
   */
  enum class Status {
    WAITING_FOR_RTK,  ///< 等待RTK初始化（初始化阶段）
    WORKING,          ///< 正常工作（定位阶段）
  };

  /**
   * @struct GridSearchResult
   * @brief 网格搜索结果结构
   * 
   * 用于RTK初始化时的网格搜索，存储每个搜索点的配准结果
   */
  struct GridSearchResult {
    SE3 pose_;         ///< 搜索位姿（网格点）
    SE3 result_pose_;  ///< 配准后位姿
    double score_ = 0.0;  ///< 配准得分（越高越好）
  };

  using Ptr = std::shared_ptr<localization::Fusion>;
  
private:
  // ========== 核心处理函数 ==========
  
  /**
   * @brief 处理同步后的IMU和点云数据
   * @param meas 同步的测量数据组
   * 
   * 流程：
   * 1. 检查IMU是否需要初始化
   * 2. 预测（Predict）
   * 3. 去畸变（Undistort）
   * 4. 配准（Align）
   */
  void ProcessMeasurements(const MessageSync::MeasureGroup& meas);

  /**
   * @brief 在RTK附近搜索车辆初始位置
   * @return 搜索成功返回true
   * 
   * 算法流程：
   * 1. 在RTK位置周围生成网格搜索点
   * 2. 对每个网格点进行NDT配准
   * 3. 选择配准得分最高的位置
   * 4. 初始化ESKF状态
   * 
   * 搜索策略：
   * - 位置范围：RTK周围一定范围
   * - 角度范围：0-360度，步长10度
   * - 评价标准：NDT配准得分
   */
  bool SearchRTK();

  /**
   * @brief 对网格搜索的某个点进行配准
   * @param gr 网格搜索结果（输入输出）
   * 
   * 功能：
   * - 使用当前点云与地图进行NDT配准
   * - 更新gr中的result_pose_和score_
   */
  void AlignForGrid(GridSearchResult& gr);

  /**
   * @brief 执行激光定位
   * @return 定位成功返回true
   * 
   * 流程：
   * 1. 使用ESKF预测位姿作为初值
   * 2. 执行NDT配准
   * 3. 使用配准结果更新ESKF状态
   * 4. 更新地图（如果位置变化较大）
   */
  bool LidarLocalization();

  /**
   * @brief 尝试使用IMU初始化
   * 
   * 流程：
   * 1. 累积静止的IMU数据
   * 2. 估计陀螺仪和加速度计零偏
   * 3. 估计重力方向
   * 4. 初始化ESKF
   */
  void TryInitIMU();

  /**
   * @brief 从YAML文件初始化IMU参数（备用方案）
   * 
   * 用于跳过静态初始化，直接使用配置文件中的参数
   */
  void InitIMUwithYaml();

  /**
   * @brief 利用IMU预测状态信息
   * 
   * 功能：
   * - 对每个IMU测量执行ESKF预测
   * - 预测数据存入imu_states_（用于点云去畸变）
   * 
   * 注意：这段时间的预测数据会放入imu_states_里
   */
  void Predict();

  /**
   * @brief 对点云进行畸变矫正
   * 
   * 原理：
   * LiDAR扫描一帧需要时间，期间车辆在运动
   * 使用IMU预测的运动轨迹补偿每个点的位置
   * 
   * 算法：
   * - 根据点的时间戳在imu_states_中插值
   * - 将点变换到扫描结束时刻
   */
  void Undistort();

  /**
   * @brief 执行点云配准和状态更新
   * 
   * 流程：
   * 1. 点云坐标变换（LiDAR -> IMU）
   * 2. 点云降采样
   * 3. 根据状态执行：
   *    - WAITING_FOR_RTK: RTK搜索
   *    - WORKING: 正常定位
   * 4. 更新可视化
   */
  void Align();

  // ========== 状态变量 ==========
  
  Status status_ = Status::WAITING_FOR_RTK;  ///< 系统状态

  // ========== 数据变量 ==========
  
  Vec3d map_origin_ = Vec3d::Zero();  ///< 地图原点（UTM坐标）
  std::string data_path_;             ///< 地图数据目录

  std::shared_ptr<MessageSync> sync_ptr_ = nullptr;  ///< 消息同步器
  StaticIMUInit imu_init_;  ///< IMU静止初始化器

  // ========== 滤波器相关 ==========
  
  ESKFD eskf_;  ///< 误差状态卡尔曼滤波器（18维）
  std::vector<NavStated> imu_states_;  ///< ESKF预测期间的状态序列（用于去畸变）

  // ========== 点云数据 ==========
  
  FullCloudPtr scan_undistort_{ new FullPointCloudType() };  ///< 去畸变后的点云
  CloudPtr current_scan_ = nullptr;  ///< 当前扫描点云

  // ========== 坐标变换 ==========
  
  SE3 TIL_;  ///< LiDAR到IMU的外参变换
  
  // ========== 同步数据 ==========
  
  MessageSync::MeasureGroup measures_;  ///< 同步后的IMU与点云数据
  GNSS::Ptr last_gnss_ = nullptr;       ///< 最新的GNSS数据

  // ========== 初始化相关 ==========
  
  bool init_has_failed_ = false;  ///< 初始化是否失败过
  SE3 last_searched_pos_;         ///< 上次RTK搜索的位置

  bool imu_need_init_ = true;  ///< 是否需要估计IMU初始零偏

  // ========== 配准相关 ==========
  
  RegistrationManager::Ptr registration_manager_ptr_;  ///< 点云配准管理器

  // ========== 参数 ==========
  
  double rtk_search_min_score_ = 4.5;  ///< RTK搜索最小配准得分阈值

  // ========== 地图管理 ==========
  
  MapLoader::Ptr map_loader_ptr_ = nullptr;  ///< 点云地图加载器

  // ========== 可视化 ==========
  
  std::shared_ptr<ui::PangolinWindow> ui_ptr_ = nullptr;  ///< Pangolin 3D可视化窗口

  // ========== 配置 ==========
  
  YAML::Node yaml_;  ///< YAML配置文件节点
};

}  // namespace localization


