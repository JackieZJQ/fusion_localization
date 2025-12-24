#pragma once 

#include <fstream>
#include <mutex>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>

#include "common/eigen_types.hpp"
#include "common/point_types.h"

namespace localization {
class MapLoader {
public:
  MapLoader(const YAML::Node& yaml);
  ~MapLoader();

  void UpdatePose(const SE3& pose);
  bool MapChanged();
  bool MapInitialized();
  CloudPtr GetRefCloud();
  std::map<Vec2i, CloudPtr, less_vec<2>> GetMapData();

  void Stop();

  using Ptr = std::shared_ptr<localization::MapLoader>;

private:
  void LoadMapIndex();     //加载地图索引
  void WorkerThreadLoop(); //后台线程加载卸载地图

  Vec2i PoseToTile(const SE3& pose);
  std::set<Vec2i, less_vec<2>> PoseToTiles(const SE3& pose);

  CloudPtr LoadTileFromFile(const Vec2i& index);

  std::string config_yaml_;                          //config地址
  std::string data_path_;                            //点云地图区块和索引地址
  std::set<Vec2i, less_vec<2>> map_data_index_;      //哪些格子存在地图数据，所有索引的排序
  std::map<Vec2i, CloudPtr, less_vec<2>> map_data_;  //第9章建立的地图数据，一个索引对应一个点云
  mutable std::mutex map_data_mutex_;
  
  //
  CloudPtr ref_cloud_ = nullptr;                     //NDT用于参考的点云
  mutable std::mutex ref_cloud_mutex_;
  std::atomic_bool map_changed_{false};
  std::atomic_bool map_initialized_{false};
  
  double tile_size_m_ = 100.f;                       //地图格子大小
  
  //
  SE3 current_pose_;  //此时定位姿态
  mutable std::mutex current_pose_mutex_;
  std::atomic_bool current_pose_updated_{false};
  
  std::atomic_bool is_running_{false};

  std::thread worker_;
};
} // namespace localization

