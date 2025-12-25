/**
 * @file fusion_flow.cpp
 * @brief 融合定位数据流控制实现
 * 
 * 负责ROS2接口层的数据流控制：
 * - 订阅传感器话题（IMU、GNSS、点云）
 * - 将ROS消息转换为内部数据格式
 * - 调用融合定位核心进行处理
 * - 发布定位结果
 */

#include "fusion/fusion_flow.hpp"

namespace localization {

/**
 * @brief 构造函数，初始化融合定位数据流
 * @param node ROS2节点指针
 * 
 * 工作流程：
 * 1. 加载配置文件
 * 2. 创建融合定位核心对象
 * 3. 创建点云转换器
 * 4. 初始化ROS2订阅器和发布器
 */
FusionFlow::FusionFlow(const rclcpp::Node::SharedPtr& node) 
  : node_(node) {
  
  //todo: yaml文件的地址应写入cmakelists文件或通过参数传递
  const std::string config_path = "/home/jackie/robobus_localization/fusion_localization_ws/src/fusion_localization/config/mapping_robosense.yaml";
  auto yaml = YAML::LoadFile(config_path);

  // 创建融合定位核心对象
  fusion_ptr_ = std::make_shared<Fusion>(yaml);
  
  // 创建点云转换器（ROS消息 -> PCL点云）
  cloud_converter_ptr_ = std::make_shared<CloudConvert>(yaml);

  // 初始化订阅器和发布器
  InitIO();
}

/**
 * @brief 初始化ROS2输入输出接口
 * 
 * 订阅传感器话题：
 * - /imu: IMU数据（加速度、角速度）
 * - /navsatfix: GNSS定位数据（经纬度、高度）
 * - /rslidar_points: LiDAR点云数据
 */
void FusionFlow::InitIO() {
  // 订阅IMU话题，队列长度10
  imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, std::bind(&FusionFlow::ImuCallback, this, std::placeholders::_1));
  
  // 订阅GNSS话题，队列长度10
  gnss_subscriber_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/navsatfix", 10, std::bind(&FusionFlow::GnssCallback, this, std::placeholders::_1));
  
  // 订阅点云话题，队列长度10
  cloud_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/rslidar_points", 10, std::bind(&FusionFlow::CloudCallback, this, std::placeholders::_1));
}

/**
 * @brief IMU数据回调函数
 * @param imu_msg_ptr ROS2 IMU消息指针
 * 
 * 功能：
 * 1. 将ROS2 IMU消息转换为内部IMU数据格式
 * 2. 传递给融合定位核心进行处理
 * 
 * 注意：IMU数据是高频数据（通常100-200Hz），用于状态预测
 */
void FusionFlow::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr) {
  
  // 转换为内部IMU数据格式
  IMU::Ptr imu = std::make_shared<localization::IMU>(imu_msg_ptr);
  
  // 将IMU数据传递给融合定位核心
  fusion_ptr_->ProcessIMU(imu);
}

/**
 * @brief GNSS数据回调函数
 * @param gnss_msg_ptr ROS2 NavSatFix消息指针
 * 
 * 功能：
 * 1. 将ROS2 GNSS消息转换为内部GNSS数据格式
 * 2. 将GPS坐标（经纬度）转换为UTM坐标系
 * 3. 过滤无效的GNSS数据（高度为NaN）
 * 4. 传递给融合定位核心进行处理
 * 
 * 注意：GNSS数据频率较低（通常1-10Hz），用于提供全局位置参考
 */
void FusionFlow::GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg_ptr) {

  // 转换为内部GNSS数据格式
  GNSS::Ptr gnss(new GNSS(gnss_msg_ptr));
  
  // 将GPS坐标（经纬度）转换为UTM坐标系
  ConvertGps2UTMOnlyTrans(*gnss);

  // 过滤无效数据：检查高度是否为NaN
  if (std::isnan(gnss->lat_lon_alt_[2]))
    return;

  // 将RTK数据传递给融合定位核心
  fusion_ptr_->ProcessRTK(gnss);
}

/**
 * @brief 点云数据回调函数
 * @param cloud_msg_ptr ROS2 PointCloud2消息指针
 * 
 * 功能：
 * 1. 提取时间戳（从ROS消息头）
 * 2. 将ROS2点云消息转换为PCL点云格式
 * 3. 进行点云预处理和滤波
 * 4. 传递给融合定位核心进行处理
 * 
 * 注意：点云数据频率通常为10Hz，数据量大，用于与地图配准获得准确位置
 */
void FusionFlow::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr) {

  // 创建内部点云数据结构
  CLOUD::Ptr cloud_ptr(new CLOUD);
  
  // 提取时间戳（秒 + 纳秒转换为秒）
  cloud_ptr->timestamp_ = cloud_msg_ptr->header.stamp.sec + cloud_msg_ptr->header.stamp.nanosec * 1e-9;
  
  // 将ROS2点云消息转换为PCL点云格式，同时进行滤波处理
  cloud_converter_ptr_->Process(cloud_msg_ptr, cloud_ptr->full_cloud_ptr_);
  
  // 将点云数据传递给融合定位核心
  fusion_ptr_->ProcessPointCloud(cloud_ptr);
}

} // namespace localization