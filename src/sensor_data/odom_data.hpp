/**
 * ************************************************************************
 *
 * @file odom_data.hpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 轮速数据结构
 *
 * ************************************************************************
 * @copyright Copyright (c) 2025
 * For study and research only, no reprinting
 * ************************************************************************
 */

#pragma once

#include <memory>

namespace localization {
struct ODOM {
public:
  ODOM() = default;
  ODOM(double timestamp, double left_pulse, double right_pulse)
      : timestamp_(timestamp), left_pulse_(left_pulse), right_pulse_(right_pulse) { }

  using Ptr = std::shared_ptr<localization::ODOM>;

public:
  double timestamp_ = 0.0;
  double left_pulse_ = 0.0;  //左右轮的单位时间转过的脉冲数
  double right_pulse_ = 0.0;
};
}  // namespace localization
