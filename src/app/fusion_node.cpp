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

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}