#include "map/map_loader.hpp"

namespace localization {
MapLoader::MapLoader(const YAML::Node& yaml) {
  
  data_path_ = yaml["map_data"].as<std::string>();
  tile_size_m_ = yaml["tile_size_meter"].as<double>();
  
  //加载地图索引
  LoadMapIndex();

  is_running_ = true;
  worker_ = std::thread(&MapLoader::WorkerThreadLoop, this);
}

MapLoader::~MapLoader() {
  Stop();
}

void MapLoader::Stop() {
  if (!is_running_) return;

  is_running_ = false;
  if (worker_.joinable()) worker_.join();
}

void MapLoader::UpdatePose(const SE3& pose) {
  std::lock_guard<std::mutex> mutex(current_pose_mutex_);
  current_pose_ = pose;
  
  current_pose_updated_.store(true, std::memory_order_release);
}

bool MapLoader::MapChanged() {
  return map_changed_.load(std::memory_order_acquire);
}

bool MapLoader::MapInitialized() {
  return map_initialized_.load(std::memory_order_acquire);
}

CloudPtr MapLoader::GetRefCloud() {
  std::lock_guard<std::mutex> mutex(ref_cloud_mutex_);
  map_changed_.store(false, std::memory_order_release);
  
  return ref_cloud_;
}

std::map<Vec2i, CloudPtr, less_vec<2>> MapLoader::GetMapData() {
  std::lock_guard<std::mutex> mutex(map_data_mutex_);

  return map_data_;
}

void MapLoader::LoadMapIndex() {
  std::ifstream fin(data_path_ + "/map_index.txt");
  while (!fin.eof()) {
    int x, y;
    fin >> x >> y;
    map_data_index_.emplace(Vec2i(x, y));
  }

  fin.close();
}

Vec2i MapLoader::PoseToTile(const SE3& pose) {
  //计算pose所对应的点云地图索引
  int gx = floor((pose.translation().x() - tile_size_m_ * 0.5) / tile_size_m_);
  int gy = floor((pose.translation().y() - tile_size_m_ * 0.5) / tile_size_m_);
  Vec2i key(gx, gy);

  return key;
}

std::set<Vec2i, less_vec<2>> MapLoader::PoseToTiles(const SE3& pose) {
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

CloudPtr MapLoader::LoadTileFromFile(const Vec2i& index) {
  //加载点云
  std::string file = data_path_ + std::to_string(index[0]) + "_" + std::to_string(index[1]) + ".pcd";
  CloudPtr cloud(new PointCloudType);

  if (pcl::io::loadPCDFile(file, *cloud) == -1) {
    LOG(INFO) << "Failed load pcd: " << file.c_str();
    return nullptr;
  }

  return cloud;
}

void MapLoader::WorkerThreadLoop() {
  LOG(INFO) << "MapLoader worker started.";

  while (is_running_) {
    //睡眠50ms
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    //判断pose是否更新
    if (!current_pose_updated_.load(std::memory_order_acquire)) continue;

    //获取当前pose对应的格子的索引
    SE3 current_pose;
    {
      std::lock_guard<std::mutex> mutex(current_pose_mutex_);
      current_pose = current_pose_;
      current_pose_updated_.store(false, std::memory_order_release);
    }

    bool map_data_changed = false;
    int cnt_new_loaded = 0, cnt_unloaded = 0;

    //1.获取pose周围格子索引
    Vec2i local_key = PoseToTile(current_pose);
    std::set<Vec2i, less_vec<2>> surrounding_index = PoseToTiles(current_pose);

    //map_data 副本，用于重构red_cloud
    std::map<Vec2i, CloudPtr, less_vec<2>> map_data;
    {
      std::lock_guard<std::mutex> mutex(map_data_mutex_);

      //2.加载必要区域
      //map_data_为已加载的地图块
      for (auto& key : surrounding_index) {
        //该地图数据不存在
        if (map_data_index_.find(key) == map_data_index_.end()) continue;
    
        //map_data_中找到需要加载的地图区块，从磁盘中加载地图区块
        if (map_data_.find(key) == map_data_.end()) {
          CloudPtr cloud = MapLoader::LoadTileFromFile(key);
          map_data_.emplace(key, cloud);
          map_data_changed = true;
          cnt_new_loaded++;
        }
      }

      //3.卸载map_data_不需要的区域，这个稍微加大一点，不需要频繁卸载
      for (auto iter = map_data_.begin(); iter != map_data_.end();) {
        if ((iter->first - local_key).cast<float>().norm() > 3.0) {
          //卸载本区块
          iter = map_data_.erase(iter);
          cnt_unloaded++;
          map_data_changed = true;
        } else {
          iter++;
        }
      }

      //4.深拷贝map_data
      map_data.clear();
      for (const auto &[key, cloud_ptr] : map_data_) {
        if (cloud_ptr) {
          map_data[key] = cloud_ptr;
        } else {
          map_data[key] = nullptr;
        }
      }
    }

    //5.重构ndt地图
    if (map_data_changed) {
      CloudPtr new_ref_cloud(new PointCloudType);
      for (auto& mp : map_data) {
        *new_ref_cloud += *mp.second;
      }
      
      {
        std::lock_guard<std::mutex> mutex(ref_cloud_mutex_);
        ref_cloud_.swap(new_ref_cloud);
        
        map_changed_.store(true, std::memory_order_release);
        map_initialized_.store(true, std::memory_order_release);
      }
      
      LOG(INFO) << "new loaded: " << cnt_new_loaded << ", unload: " << cnt_unloaded << "\n"
                << "rebuild global cloud, grids: " << map_data_.size();
    }

  }

  LOG(INFO) << "MapLoader worker stopped.";
}

} // namespace localization


