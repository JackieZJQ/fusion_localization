#include "fusion/fusion_flow.hpp"

namespace localization {
FusionFlow::FusionFlow(const rclcpp::Node::SharedPtr& node) 
  : node_(node) {
  
  //todo
  //yaml文件的地址写入cmakelists文件
  const std::string config_path = "/home/jackie/robobus_localization/fusion_localization_ws/src/fusion_localization/config/mapping_robosense.yaml";
  auto yaml = YAML::LoadFile(config_path);

  fusion_ptr_ = std::make_shared<Fusion>(yaml);
  cloud_converter_ptr_ = std::make_shared<CloudConvert>(yaml);

  InitIO();
}

void FusionFlow::InitIO() {
  imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&FusionFlow::imu_callback, this, std::placeholders::_1));
  gnss_subscriber_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>("/navsatfix", 10, std::bind(&FusionFlow::gnss_callback, this, std::placeholders::_1));
  cloud_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points", 10, std::bind(&FusionFlow::cloud_callback, this, std::placeholders::_1));
}

void FusionFlow::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr) {
  
  //转换为IMU格式
  IMU::Ptr imu = std::make_shared<localization::IMU>(imu_msg_ptr);
  fusion_ptr_->ProcessIMU(imu);
}

void FusionFlow::gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg_ptr) {

  //转换为GNSS格式
  GNSS::Ptr gnss(new GNSS(gnss_msg_ptr));
  ConvertGps2UTMOnlyTrans(*gnss);

  if (std::isnan(gnss->lat_lon_alt_[2]))
    return;

  fusion_ptr_->ProcessRTK(gnss);
}

void FusionFlow::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr) {

  //转换为CLOUD格式
  //对点云数量做滤波
  CLOUD::Ptr cloud_ptr(new CLOUD);
  cloud_ptr->timestamp_ = cloud_msg_ptr->header.stamp.sec + cloud_msg_ptr->header.stamp.nanosec * 1e-9;
  cloud_converter_ptr_->Process(cloud_msg_ptr, cloud_ptr->full_cloud_ptr_);
  fusion_ptr_->ProcessPointCloud(cloud_ptr);
}
} // namespace localization