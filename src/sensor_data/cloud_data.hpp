/**
 * ************************************************************************
 * 
 * @file cloud_data.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 雷达点云数据结构
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2024 
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "common/point_types.h"

namespace localization {
struct CLOUD {
public:
  CLOUD() : timestamp_(0.0), full_cloud_ptr_(new FullPointCloudType()) { }

  using Ptr = std::shared_ptr<localization::CLOUD>;

public:
  double timestamp_;
  FullCloudPtr full_cloud_ptr_;
  //todo
  //是否需要cloud_ptr?
  //CloudPtr cloud_ptr_;
};
} //namespace localization