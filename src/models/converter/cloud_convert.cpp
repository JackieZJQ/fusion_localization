#include "models/converter/cloud_convert.hpp"

namespace localization {
CloudConvert::CloudConvert(const YAML::Node& yaml)
  : yaml_(yaml) {
  
    LoadFromYAML();
}

void CloudConvert::Process(const sensor_msgs::msg::PointCloud2::ConstPtr& msg, FullCloudPtr& pcl_out) {
  switch (lidar_type_) {
    case LidarType::ROBOM1:
    case LidarType::ROBO32:
      RobosenseHandler(msg);
      break;
    default:
      LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
      break;
  }
  
  *pcl_out = cloud_out_;
}

void CloudConvert::RobosenseHandler(const sensor_msgs::msg::PointCloud2::ConstPtr& msg) {
  cloud_out_.clear();
  cloud_full_.clear();
  
  pcl::PointCloud<robosense_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();

  double time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  for (int i = 0; i < pl_orig.points.size(); i++) {
    if (i % point_filter_num_ != 0) continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

    FullPointType added_pt;
    
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.time = (pl_orig.points[i].timestamp - time) * 1000; //每个点的时间转为距离第一个点的时间 单位: ms

    cloud_out_.points.push_back(added_pt);
  }

}

void CloudConvert::LoadFromYAML() {
  time_scale_ = yaml_["preprocess"]["time_scale"].as<double>(); //todo 删除一些没用的参数
  int lidar_type = yaml_["preprocess"]["lidar_type"].as<int>();
  num_scans_ = yaml_["preprocess"]["scan_line"].as<int>();
  point_filter_num_ = yaml_["preprocess"]["point_filter_num"].as<int>();

  if (lidar_type == 1) {
    lidar_type_ = LidarType::ROBOM1;
    LOG(INFO) << "Using Robosense M1 Lidar";
  } else if (lidar_type == 2) {
    lidar_type_ = LidarType::ROBO32;
    LOG(INFO) << "Using Robosense 32 Lidar";
  } else {
    LOG(WARNING) << "unknown lidar_type";
  }
}
}  // namespace localization
