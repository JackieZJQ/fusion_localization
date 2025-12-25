/**
 * @file fusion_flow.hpp
 * @brief 融合定位数据流控制器
 * @author Zhang Jiaqi (zhangiaii97@gmail.com)
 * 
 * 本类负责：
 * 1. ROS2接口层：订阅传感器话题，发布定位结果
 * 2. 数据转换：ROS消息格式 <-> 内部数据格式
 * 3. 调用融合定位核心进行数据处理
 * 
 * 数据流：
 * ROS2话题 -> 回调函数 -> 数据转换 -> Fusion核心 -> 定位结果 -> ROS2话题
 */

#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "models/converter/cloud_convert.hpp"
#include "models/converter/utm_convert.hpp"
#include "sensor_data/imu_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/cloud_data.hpp"
#include "fusion/fusion.hpp"

namespace localization {

/**
 * @class FusionFlow
 * @brief 融合定位数据流控制类
 * 
 * 职责：
 * - 订阅ROS2传感器话题（IMU、GNSS、LiDAR）
 * - 将传感器数据转换为内部格式
 * - 调用Fusion核心进行融合定位
 * - 发布定位结果到ROS2话题
 */
class FusionFlow {
public:
  /**
   * @brief 构造函数
   * @param node ROS2节点指针
   */
  FusionFlow(const rclcpp::Node::SharedPtr& node);

  /**
   * @brief 初始化输入输出接口（订阅器和发布器）
   */
  void InitIO();

  using Ptr = std::shared_ptr<localization::FusionFlow>;

private:
  // ========== 传感器回调函数 ==========
  
  /**
   * @brief IMU数据回调函数
   * @param imu_msg_ptr ROS2 IMU消息
   * 
   * 处理流程：ROS消息 -> 内部IMU格式 -> Fusion核心
   */
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr);
  
  /**
   * @brief GNSS数据回调函数
   * @param gnss_msg_ptr ROS2 NavSatFix消息
   * 
   * 处理流程：ROS消息 -> 内部GNSS格式 -> UTM坐标转换 -> Fusion核心
   */
  void GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg_ptr);
  
  /**
   * @brief 点云数据回调函数
   * @param cloud_msg_ptr ROS2 PointCloud2消息
   * 
   * 处理流程：ROS消息 -> PCL点云格式 -> 滤波 -> Fusion核心
   */
  void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr);

  // ========== 发布RViz可视化消息 ==========
  // TODO: 实现以下发布函数
  
  /**
   * @brief 发布当前帧点云到RViz
   */
  void PublishCurrentScan();
  
  /**
   * @brief 发布地图点云到RViz
   */
  void PublishMap();
  
  /**
   * @brief 发布里程计信息到RViz
   */
  void PublishOdom();
  
  /**
   * @brief 发布TF变换到RViz
   */
  void PublishTf();

  // ========== ROS2订阅器 ==========
  
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;          ///< IMU数据订阅器
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;   ///< GNSS数据订阅器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_; ///< 点云数据订阅器

  // ========== ROS2发布器 ==========
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_publisher_; ///< 当前帧点云发布器
  
  // ========== 核心组件 ==========
  
  CloudConvert::Ptr cloud_converter_ptr_;  ///< 点云转换器（ROS -> PCL）
  std::shared_ptr<Fusion> fusion_ptr_;     ///< 融合定位核心对象
  rclcpp::Node::SharedPtr node_;           ///< ROS2节点指针

  // TODO: 添加更多发布器用于RViz可视化 (*^__^*) 嘻嘻…
};

} // namespace localization

