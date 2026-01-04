#include <rclcpp/rclcpp.hpp>
#include <glog/logging.h>

#include "fusion/fusion_flow.hpp"
 
int main(int argc, char** argv) {

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fusion_node");
  auto fusion_node = std::make_shared<localization::FusionFlow>(node);

  // 多线程执行器，允许地图发布等与传感器回调并行
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  return 0;
}