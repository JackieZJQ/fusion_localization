/**
 * ************************************************************************
 * 
 * @file imu_predictor_node.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief imu 预测模块节点函数
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include <glog/logging.h>

#include "fusion/imu_predictor_flow.hpp"

int main(int argc, char** argv) {

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  rclcpp::init(argc, argv);

  auto imu_predictor_flow = std::make_shared<localization::ImuPredictorFlow>();

  // 多线程执行器，允许地图发布等与传感器回调并行
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(imu_predictor_flow);
  exec.spin();

  return 0;
}