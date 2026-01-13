/**
 * ************************************************************************
 * 
 * @file map_loader.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 点云地图加载/卸载模块
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2025
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include <fstream>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

#include "tiles/tile_manager.hpp"

namespace localization {
TileManager::TileManager(const YAML::Node& yaml) {

  tile_size_m_ = yaml["tile_size_meter"].as<double>();
  map_tiles_root_dir_  = yaml["map_data"].as<std::string>();
  
  //加载地图索引
  if(!LoadAvailableTileIndices()) {
    LOG(ERROR) << "Failed to load available tile indices from " + map_tiles_root_dir_ + "/map_index.txt";
  }

  tile_thread_should_run_.store(true, std::memory_order_release);
  tile_management_thread_ = std::thread(&TileManager::BackgroundTileManagementLoop, this);
}

TileManager::~TileManager() {
  RequestStop();
}

void TileManager::RequestStop() {
  if (!tile_thread_should_run_.load(std::memory_order_acquire)) return;

  tile_thread_should_run_.store(false, std::memory_order_release);
  if (tile_management_thread_.joinable()) tile_management_thread_.join();
}

void TileManager::UpdateCurrentPose(const SE3& pose) {
  std::lock_guard<std::mutex> mutex(current_pose_mutex_);
  current_pose_ = pose;

  pose_update_notification_.store(true, std::memory_order_release);
}

bool TileManager::HasMapChanged() {
  return ref_cloud_changed_.load(std::memory_order_acquire);
}

bool TileManager::HasMapInitialized() {
  return ref_cloud_initialized_.load(std::memory_order_acquire);
}

CloudPtr TileManager::GetRefCloud() {
  std::lock_guard<std::mutex> lock(ref_cloud_mutex_);

  // 将地图点云指针传输出去后，重置map_changed_标志
  ref_cloud_changed_.store(false, std::memory_order_release);
  
  return ref_cloud_;
}

CloudPtr TileManager::GetVisFullCloud() {
  // 使用缓存避免重复加载阻塞
  // 使用双重检查锁定模式(Double-Checked Locking)确保线程安全
  if (vis_full_cloud_loaded_.load(std::memory_order_acquire)) {
    std::lock_guard<std::mutex> lock(vis_full_cloud_mutex_);
    // 再次检查，防止竞态条件
    if (vis_full_cloud_) {
      return vis_full_cloud_;
    }
  }
  
  // 首次加载可视化地图（使用特殊的tile索引）
  CloudPtr static_full_map = LoadTileFromDisk(Vec2i(VIS_MAP_TILE_X, VIS_MAP_TILE_Y));
  
  {
    std::lock_guard<std::mutex> lock(vis_full_cloud_mutex_);
    // 只有成功加载才设置缓存和标志
    if (static_full_map) {
      vis_full_cloud_ = static_full_map;
      vis_full_cloud_loaded_.store(true, std::memory_order_release);
    } else {
      LOG(WARNING) << "Failed to load visualization map from disk";
    }
  }
  
  return static_full_map;
}

std::map<Vec2i, CloudPtr, less_vec<2>> TileManager::GetLoadedTiles() {
  std::lock_guard<std::mutex> lock(loaded_tiles_mutex_);

  return loaded_tiles_;
}

bool TileManager::LoadAvailableTileIndices() {
  std::string index_file_path = map_tiles_root_dir_ + "/map_index.txt";
  std::ifstream fin(index_file_path);

  //1.增强错误处理：检查文件是否成功打开
  if (!fin.is_open()) {
    LOG(ERROR) << "Failed to open tile index file: " << index_file_path;
    return false;
  }

  //2.优化读取逻辑：逐行读取并解析坐标，避免使用EOF作为循环条件
  int x, y;
  available_tile_indices_.clear(); //清空现有数据
  while (fin >> x >> y) {
    available_tile_indices_.emplace(x, y);
  }

  //3.检查是否因错误而非EOF结束
  if (!fin.eof()) {
    LOG(WARNING) << "Tile index file may be malformed or read error occurred: " << index_file_path;
    // 注意：即使中间出错，已成功读取的数据仍可用，取决于你的需求是容忍还是失败
  }

  fin.close();
  LOG(INFO) << "Loaded " << available_tile_indices_.size() << " tile indices from " << index_file_path;
  return true;
}

Vec2i TileManager::PoseToTile(const SE3& pose) {
  //计算pose所对应的点云地图索引
  int gx = floor((pose.translation().x() - tile_size_m_ * 0.5) / tile_size_m_);
  int gy = floor((pose.translation().y() - tile_size_m_ * 0.5) / tile_size_m_);
  Vec2i key(gx, gy);

  return key;
}

std::set<Vec2i, less_vec<2>> TileManager::PoseToSurroundTiles(const SE3& pose) {
  //计算pose所对应的点云地图索引
  int gx = floor((pose.translation().x() - tile_size_m_ * 0.5) / tile_size_m_);
  int gy = floor((pose.translation().y() - tile_size_m_ * 0.5) / tile_size_m_);
  Vec2i key(gx, gy);

  //pose周围9宫格索引
  std::set<Vec2i, less_vec<2>> surrounding_index {
    key + Vec2i(-1,  1),  key + Vec2i(0,  1), key + Vec2i(1,  1),
    key + Vec2i(-1,  0),  key + Vec2i(0,  0), key + Vec2i(1,  0),
    key + Vec2i(-1, -1),  key + Vec2i(0, -1), key + Vec2i(1, -1),
  };

  return surrounding_index;
}

CloudPtr TileManager::LoadTileFromDisk(const Vec2i& index) {
  //加载点云
  std::string file = map_tiles_root_dir_ + std::to_string(index[0]) + "_" + std::to_string(index[1]) + ".pcd";
  CloudPtr cloud(new PointCloudType);

  if (pcl::io::loadPCDFile(file, *cloud) == -1) {
    LOG(INFO) << "Failed load pcd: " << file.c_str();
    return nullptr;
  }

  return cloud;
}

void TileManager::BackgroundTileManagementLoop() {
  LOG(INFO) << "TileManager worker started.";

  while (tile_thread_should_run_.load(std::memory_order_acquire)) {
    //每50ms检查一次pose是否更新
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    //判断pose是否更新，获取当前pose
    if (!pose_update_notification_.load(std::memory_order_acquire)) continue;

    //1.获取当前pose
    SE3 current_pose;
    
    {
      std::lock_guard<std::mutex> lock(current_pose_mutex_);
      current_pose = current_pose_;
      pose_update_notification_.store(false, std::memory_order_release);
    }

    //2.获取pose周围格子索引，获取loaded_tiles副本
    Vec2i local_tile_indice = PoseToTile(current_pose);
    std::set<Vec2i, less_vec<2>> surrounding_tile_indices = PoseToSurroundTiles(current_pose);
    std::map<Vec2i, CloudPtr, less_vec<2>> loaded_tiles;

    {
      std::lock_guard<std::mutex> lock(loaded_tiles_mutex_);
      loaded_tiles = loaded_tiles_;
    }

    //3.加载/卸载地图格子计数
    bool has_loaded_tiles_changed = false;
    int cnt_new_loaded_tiles = 0;
    int cnt_unloaded_tiles = 0;

    //4.卸载loaded_tiles不需要的区域，这个稍微加大一点，不需要频繁卸载
    for (auto iter = loaded_tiles.begin(); iter != loaded_tiles.end();) {
      if ((iter->first - local_tile_indice).cast<float>().norm() > 3.0) {
        //卸载本区块
        iter = loaded_tiles.erase(iter);
        cnt_unloaded_tiles++;
        has_loaded_tiles_changed = true;
      } else {
        ++iter;
      }
    }

    //5.加载surrounding_tile_indices中需要但loaded_tiles中没有的区域
    for (auto& indice : surrounding_tile_indices) {
      //该地图数据不存在
      if (available_tile_indices_.find(indice) == available_tile_indices_.end()) continue;
    
      //已加载点云块loaded_tiles中未找到当前姿态周边点云块surrounding_tile_indices
      //从磁盘中加载地图区块
      if (loaded_tiles.find(indice) == loaded_tiles.end()) {
        CloudPtr cloud = LoadTileFromDisk(indice);
        if (cloud == nullptr) continue; //空指针直接跳过

        loaded_tiles.emplace(indice, cloud);
        has_loaded_tiles_changed = true;
        cnt_new_loaded_tiles++;
      }
    }

    if (!has_loaded_tiles_changed) continue;

    //6.重构用于NDT配准的全局点云地图ref_cloud_
    CloudPtr new_ref_cloud(new PointCloudType);
    for (auto& mp : loaded_tiles) {
      *new_ref_cloud += *mp.second;
    }

    //7.更新loaded_tiles_
    {
      std::lock_guard<std::mutex> lock(loaded_tiles_mutex_);
      loaded_tiles_.swap(loaded_tiles);
    }

    //8.更新ref_cloud_
    {
      std::lock_guard<std::mutex> lock(ref_cloud_mutex_);
      ref_cloud_.swap(new_ref_cloud);

      ref_cloud_initialized_.store(true, std::memory_order_release);
      ref_cloud_changed_.store(true, std::memory_order_release);
    }
      
    LOG(INFO) << "new loaded: " << cnt_new_loaded_tiles << ", unload: " << cnt_unloaded_tiles << "\n"
              << "rebuild global cloud, grids: " << loaded_tiles_.size();
    
  } //while loop

  LOG(INFO) << "TileManager worker stopped.";
}

} // namespace localization


