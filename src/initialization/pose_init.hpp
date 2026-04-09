/**
 * ************************************************************************
 * 
 * @file pose_init.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 姿态初始化函数
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once

#include <yaml-cpp/yaml.h>

namespace localization {
class PoseInit {
public:
  PoseInit() = delete;
  PoseInit(const YAML::Node& yaml);

private:
  
};
} // namespace localization