/**
 * ************************************************************************
 *
 * @file cloud_data.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 预处理雷达点云
 *        将雷达原始点云转换至FullCloud，由MessageSync类持有，
 *        负责将收到的雷达消息与IMU同步并预处理后，再交给LO/LIO算法
 *
 * ************************************************************************
 * @copyright Copyright (c) 2024
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/point_types.h"
#include "common/timer.hpp"

namespace localization {
class CloudConvert {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CloudConvert() = delete;
  CloudConvert(const YAML::Node& yaml);

  void Process(const sensor_msgs::msg::PointCloud2::ConstPtr& msg, FullCloudPtr& pcl_out);

  enum class LidarType {
    ROBOM1 = 1, //Robosense M1
    ROBO32 = 2, //Robosense 32线
  };

  using Ptr = std::shared_ptr<localization::CloudConvert>;

private:
  void LoadFromYAML();
  void RobosenseHandler(const sensor_msgs::msg::PointCloud2::ConstPtr& msg);

  FullPointCloudType cloud_full_, cloud_out_; // 输出点云
  LidarType lidar_type_ = LidarType::ROBO32;  // 雷达类型
  int point_filter_num_ = 1;                  // 跳点
  int num_scans_ = 6;                         // 扫描线数
  float time_scale_ = 1e-3;                   // 雷达点的时间字段与秒的比例
  YAML::Node yaml_;
};
}  // namespace localization