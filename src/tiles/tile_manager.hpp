/**
 * ************************************************************************
 * 
 * @file map_loader.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 点云地图加载/卸载模块
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2025 XXX 
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once 

#include <mutex>
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>

#include "common/eigen_types.hpp"
#include "common/point_types.h"

namespace localization {
class TileManager {
public:
  TileManager() = delete;
  TileManager(const YAML::Node& yaml);
  
  ~TileManager();

  void UpdateCurrentPose(const SE3& pose);
  bool HasMapChanged();
  bool HasMapInitialized();
  
  CloudPtr GetRefCloud();     // todo 重命名
  CloudPtr GetStaticPointcloudMap();

  std::map<Vec2i, CloudPtr, less_vec<2>> GetLoadedTiles(); // 用于pangolin可视化，todo删除

  using Ptr = std::shared_ptr<localization::TileManager>;

private:
  bool LoadAvailableTileIndices();               // 加载地图格子索引
  CloudPtr LoadTileFromDisk(const Vec2i& index, bool voxel_filter = false, float leaf_size = 1.0f); //从文件加载地图格子

  Vec2i PoseToTile(const SE3& pose);
  std::set<Vec2i, less_vec<2>> PoseToSurroundTiles(const SE3& pose);

  void BackgroundTileManagementLoop();                   // 后台线程加载卸载地图

  void RequestStop();

  std::string map_tiles_root_dir_;                       // 地图数据根目录
  std::set<Vec2i, less_vec<2>> available_tile_indices_;  // 所有地图索引
  std::map<Vec2i, CloudPtr, less_vec<2>> loaded_tiles_;  //当前加载的地图点云，一个索引对应一块点云
  mutable std::mutex loaded_tiles_mutex_;
  
  CloudPtr ref_cloud_;                              // NDT用于参考的点云
  mutable std::mutex ref_cloud_mutex_;              // 保护ref_cloud_的互斥锁
  std::atomic_bool ref_cloud_changed_{ false };     // ref_cloud_是否更新标志
  std::atomic_bool ref_cloud_initialized_{ false }; // ref_cloud_是否初始化标志
  
  SE3 current_pose_;                                   // 此时定位姿态
  mutable std::mutex current_pose_mutex_;              // 保护current_pose_的互斥锁
  std::atomic_bool pose_update_notification_{ false }; // current_pose_是否更新标志

  std::thread tile_management_thread_;               // 后台线程
  std::atomic_bool tile_thread_should_run_{ false }; // 后台线程是否运行标志

  double tile_size_m_ = 100.f;                       // 地图格子大小，单位米
};
} // namespace localization

