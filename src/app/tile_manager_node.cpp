/**
 * ************************************************************************
 * 
 * @file map_server_node.cpp
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * @brief 点云地图管理服务节点
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2026
 * For study and research only, no reprinting
 * ************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include <glog/logging.h>

#include "tiles/tile_manager_flow.hpp"

int main(int argc, char** argv) {

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("tile_manager_node");
  auto tile_manager_node = std::make_shared<localization::TileManagerFlow>(node);

  // 多线程执行器，允许地图发布等与传感器回调并行
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}