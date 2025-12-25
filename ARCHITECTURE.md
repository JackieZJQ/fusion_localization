# 系统架构说明 (System Architecture)

本文档详细说明融合定位系统的架构设计和代码组织。

## 目录
- [系统概述](#系统概述)
- [模块划分](#模块划分)
- [数据流](#数据流)
- [状态机](#状态机)
- [关键算法](#关键算法)
- [坐标系统](#坐标系统)

## 系统概述

融合定位系统采用分层架构设计：

```
┌─────────────────────────────────────────────────┐
│          应用层 (Application Layer)              │
│         fusion_node.cpp - ROS2节点入口          │
└─────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────┐
│         接口层 (Interface Layer)                 │
│     FusionFlow - ROS2接口与数据转换              │
└─────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────┐
│         核心层 (Core Layer)                      │
│      Fusion - 多传感器融合定位核心               │
└─────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────┐
│         算法层 (Algorithm Layer)                 │
│  ┌──────────┬──────────┬──────────┬──────────┐  │
│  │   ESKF   │   NDT    │  IMU初始 │  数据同步 │  │
│  │  滤波器  │  配准    │          │          │  │
│  └──────────┴──────────┴──────────┴──────────┘  │
└─────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────┐
│         数据层 (Data Layer)                      │
│     传感器数据、地图数据、导航状态               │
└─────────────────────────────────────────────────┘
```

## 模块划分

### 1. 应用层 (src/app/)

**文件**: `fusion_node.cpp`

**职责**:
- ROS2节点的main函数入口
- 初始化Google日志系统
- 创建和管理ROS2节点生命周期

**代码示例**:
```cpp
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fusion_node");
  auto fusion_node = std::make_shared<localization::FusionFlow>(node);
  rclcpp::spin(node);
  return 0;
}
```

### 2. 接口层 (src/fusion/)

#### FusionFlow类 (`fusion_flow.hpp/cpp`)

**职责**:
- 订阅ROS2传感器话题
- ROS消息与内部数据格式转换
- 调用Fusion核心进行处理
- 发布定位结果

**主要方法**:
```cpp
class FusionFlow {
  void ImuCallback();    // IMU数据回调
  void GnssCallback();   // GNSS数据回调
  void CloudCallback();  // 点云数据回调
  void PublishOdom();    // 发布定位结果
  void PublishTf();      // 发布坐标变换
};
```

### 3. 核心层 (src/fusion/)

#### Fusion类 (`fusion.hpp/cpp`)

**职责**:
- 管理定位系统状态机
- 协调各算法模块工作
- 处理传感器数据融合

**状态机**:
```
WAITING_FOR_RTK (初始化)
    ↓ RTK搜索成功
WORKING (正常定位)
```

**核心流程**:
```cpp
void ProcessMeasurements(meas) {
  if (需要IMU初始化) {
    TryInitIMU();
  } else if (WORKING状态) {
    Predict();      // IMU预测
    Undistort();    // 点云去畸变
  }
  Align();          // 点云配准
}
```

### 4. 算法层 (src/models/)

#### 4.1 卡尔曼滤波 (models/kalmanfilter/)

**ESKF类** (`eskf.hpp`)

18维误差状态卡尔曼滤波器：
- **状态向量** (18维):
  ```
  [位置p(3), 速度v(3), 姿态R(3), 陀螺零偏bg(3), 加速零偏ba(3), 重力grav(3)]
  ```

- **预测步**: 使用IMU数据
  ```cpp
  void Predict(IMU);  // 状态预测和协方差传播
  ```

- **更新步**: 使用GNSS或配准结果
  ```cpp
  void ObserveGNSS(GNSS);        // GNSS观测更新
  void ObserveSE3(SE3, noise);   // 位姿观测更新
  ```

#### 4.2 点云配准 (models/registration/)

**RegistrationManager类** (`registration_manager.hpp/cpp`)

管理NDT配准算法：
- 设置目标点云（地图）
- 执行点云匹配
- 返回配准位姿和得分

**NDT算法**:
- Normal Distributions Transform（正态分布变换）
- 使用OpenMP并行加速
- 输出配准位姿和匹配得分

#### 4.3 IMU初始化 (models/initialization/)

**StaticIMUInit类** (`static_imu_init.hpp/cpp`)

静态IMU初始化：
- 收集静止时的IMU数据
- 估计陀螺仪零偏
- 估计加速度计零偏
- 估计重力方向

#### 4.4 数据同步 (models/synchronization/)

**MessageSync类** (`measure_sync.hpp/cpp`)

同步IMU和LiDAR数据：
- 缓存IMU数据
- 等待点云数据
- 提取与点云对应的IMU数据
- 回调用户处理函数

```cpp
struct MeasureGroup {
  double lidar_begin_time_;  // 点云开始时间
  FullCloudPtr lidar_;       // 点云数据
  std::deque<IMU::Ptr> imu_; // 对应的IMU数据
};
```

#### 4.5 坐标转换 (models/converter/)

**CloudConvert** (`cloud_convert.hpp/cpp`)
- ROS2点云 → PCL点云
- 点云滤波和预处理

**UTMConvert** (`utm_convert.hpp/cpp`)
- GPS坐标（经纬度） → UTM坐标
- UTM坐标 → GPS坐标

### 5. 数据层 (src/sensor_data/)

#### 数据结构

**IMU数据** (`imu_data.hpp`)
```cpp
struct IMU {
  double timestamp_;    // 时间戳
  Vec3d gyro_;         // 角速度
  Vec3d acce_;         // 加速度
};
```

**GNSS数据** (`gnss_data.hpp`)
```cpp
struct GNSS {
  double timestamp_;
  Vec3d lat_lon_alt_;  // 纬度、经度、高度
  SE3 utm_pose_;       // UTM坐标位姿
};
```

**点云数据** (`cloud_data.hpp`)
```cpp
struct CLOUD {
  double timestamp_;
  FullCloudPtr full_cloud_ptr_;  // 完整点云
};
```

**导航状态** (`nav_state.hpp`)
```cpp
struct NavState {
  double timestamp_;
  Vec3d p_;   // 位置
  Vec3d v_;   // 速度
  SO3 R_;     // 姿态
  Vec3d bg_;  // 陀螺零偏
  Vec3d ba_;  // 加速零偏
  Vec3d g_;   // 重力
};
```

### 6. 地图管理 (src/map/)

**MapLoader类** (`map_loader.hpp/cpp`)

功能：
- 加载点云地图文件
- 分块管理大范围地图
- 根据当前位置动态加载/卸载地图

```cpp
class MapLoader {
  void UpdatePose(SE3);         // 更新当前位置
  CloudPtr GetRefCloud();       // 获取参考点云
  bool MapInitialized();        // 地图是否已初始化
  bool MapChanged();            // 地图是否有变化
};
```

### 7. 可视化 (src/ui/)

**PangolinWindow类** (`pangolin_window.hpp/cpp`)

3D可视化界面：
- 显示当前扫描点云
- 显示地图点云
- 显示车辆位置和轨迹
- 显示导航状态信息

## 数据流

### 1. 传感器数据流

```
传感器 → ROS2话题 → FusionFlow → Fusion → 算法模块 → 定位结果
```

详细流程：

**IMU数据流**:
```
/imu (sensor_msgs/Imu)
  ↓ ImuCallback
ROS消息转换 (IMU::Ptr)
  ↓ ProcessIMU
MessageSync (数据同步)
  ↓ 同步后触发
ProcessMeasurements
  ↓ Predict
ESKF预测
```

**GNSS数据流**:
```
/navsatfix (sensor_msgs/NavSatFix)
  ↓ GnssCallback
ROS消息转换 (GNSS::Ptr)
  ↓ UTM坐标转换
ProcessRTK
  ↓ 保存最新GNSS
用于初始化或观测更新
```

**点云数据流**:
```
/rslidar_points (sensor_msgs/PointCloud2)
  ↓ CloudCallback
ROS消息转换 + 滤波 (CLOUD::Ptr)
  ↓ ProcessPointCloud
MessageSync (数据同步)
  ↓ 同步后触发
ProcessMeasurements
  ↓ Undistort → Align
点云去畸变 → NDT配准
```

### 2. 融合定位处理流程

```
┌─────────────────────────────────────────┐
│  1. 数据同步 (MessageSync)               │
│     - 收集对应时间段的IMU数据            │
│     - 与点云数据打包                     │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│  2. IMU预测 (Predict)                    │
│     - 逐个处理IMU数据                    │
│     - ESKF状态预测                       │
│     - 保存预测轨迹 (用于去畸变)          │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│  3. 点云去畸变 (Undistort)               │
│     - 根据点时间戳插值位姿               │
│     - 运动补偿每个点                     │
│     - 统一到扫描结束时刻                 │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│  4. 点云配准 (Align)                     │
│     - 坐标转换 (LiDAR → IMU)             │
│     - 降采样 (VoxelGrid)                 │
│     - NDT配准与地图                      │
│     - 获得配准位姿和得分                 │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│  5. 状态更新                             │
│     - 使用配准结果更新ESKF               │
│     - 发布定位结果                       │
│     - 更新可视化                         │
└─────────────────────────────────────────┘
```

## 状态机

### 系统状态转换

```
┌──────────────────┐
│   系统启动        │
└────────┬─────────┘
         ↓
┌──────────────────┐
│ WAITING_FOR_RTK  │  ← 等待RTK初始化
│                  │
│ 等待IMU初始化    │
│ 等待RTK数据      │
│ 执行网格搜索     │
└────────┬─────────┘
         ↓ RTK搜索成功
         │ (配准得分 > 阈值)
┌────────▼─────────┐
│    WORKING       │  ← 正常工作
│                  │
│ IMU预测          │
│ 点云去畸变       │
│ NDT配准          │
│ 状态更新         │
└──────────────────┘
```

### RTK初始化详细流程

```
收到RTK数据
  ↓
生成搜索网格
  位置: RTK周围 ±几米
  角度: 0-360度，步长10度
  ↓
并行处理每个网格点:
  ├─ 设置初始位姿
  ├─ NDT配准
  └─ 记录配准得分
  ↓
选择最高得分位姿
  ↓
得分 > 阈值？
  ├─ 是: 初始化ESKF → 切换到WORKING
  └─ 否: 继续等待下一个RTK
```

## 关键算法

### 1. ESKF预测

**输入**: IMU测量 (角速度ω, 加速度a)

**输出**: 更新后的状态和协方差

**步骤**:
```
1. 状态预测:
   p = p + v*dt + 0.5*a*dt^2
   v = v + a*dt
   R = R * Exp(ω*dt)
   
2. 协方差预测:
   P = F*P*F' + G*Q*G'
   
   其中:
   F: 状态转移矩阵
   G: 噪声驱动矩阵
   Q: 过程噪声协方差
```

### 2. NDT配准

**输入**: 
- 源点云 (当前扫描)
- 目标点云 (地图)
- 初始位姿估计

**输出**:
- 配准后位姿
- 配准得分

**原理**:
```
1. 目标点云体素化
   - 划分为规则网格
   - 每个体素用正态分布表示
   
2. 优化目标函数
   maximize Σ exp(-0.5 * (p-μ)'*Σ^-1*(p-μ))
   
   其中:
   p: 源点云点经过位姿变换后的坐标
   μ, Σ: 体素的均值和协方差
   
3. 使用牛顿法或高斯-牛顿法优化
```

### 3. 点云去畸变

**问题**: LiDAR扫描一帧需要时间（如100ms），期间车辆在运动

**解决方案**: 运动补偿

**算法**:
```
对点云中的每个点 p_i，时间戳 t_i:
1. 在IMU预测轨迹中插值得到 T(t_i)
2. 计算补偿变换:
   p_compensated = T_end^-1 * T(t_i) * p_i
3. 所有点统一到扫描结束时刻 T_end
```

**实现**:
```cpp
// 并行处理每个点
std::for_each(std::execution::par_unseq, 
  cloud->points.begin(), cloud->points.end(), 
  [&](auto& pt) {
    // 根据点时间戳插值位姿
    SE3 Ti;
    math::PoseInterp(pt.time, imu_states_, Ti);
    
    // 运动补偿
    Vec3d p_compensate = 
      TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pt;
    
    // 更新点坐标
    pt.xyz = p_compensate;
  });
```

## 坐标系统

### 坐标系定义

系统中使用多个坐标系：

```
1. 世界坐标系 (World Frame) - W
   - UTM坐标系
   - 地图原点为参考
   
2. IMU坐标系 (IMU Frame) - I
   - 车体坐标系
   - 前-右-下 (FRD) 或 前-左-上 (FLU)
   
3. LiDAR坐标系 (LiDAR Frame) - L
   - LiDAR传感器坐标系
   
4. 地图坐标系 (Map Frame) - M
   - 与世界坐标系可能有偏移
```

### 坐标变换

**外参标定**:
```
TIL: LiDAR → IMU 变换
- 通过离线标定获得
- 保存在配置文件中
```

**状态表示**:
```
ESKF估计的状态在IMU坐标系中
配准得到的位姿在世界坐标系中
```

**点云处理流程中的坐标变换**:
```
1. 点云采集 (LiDAR坐标系)
   ↓
2. 去畸变 (转换到IMU坐标系)
   p_imu = TIL * p_lidar
   ↓
3. 配准 (世界坐标系)
   p_world = T_world_imu * p_imu
```

### 变换矩阵

使用Sophus库表示SE3变换：
```cpp
SE3 T;  // 4x4变换矩阵
T.translation();  // 平移部分 (3x1)
T.so3();         // 旋转部分 (SO3)
T.matrix();      // 完整矩阵 (4x4)
T.inverse();     // 逆变换
```

## 性能优化

### 1. 并行计算

**点云去畸变**:
```cpp
std::for_each(std::execution::par_unseq, ...);
```

**NDT配准**:
- 使用ndt_omp (OpenMP并行)
- 多线程加速最近邻搜索

### 2. 数据结构优化

**点云降采样**:
```cpp
VoxelGrid滤波，体素大小0.5m
减少点云数量，加速配准
```

**地图分块管理**:
```
只加载当前位置附近的地图
动态卸载远处的地图
减少内存占用
```

### 3. 算法优化

**ESKF预测**:
- 只在有IMU数据时预测
- 批量处理IMU数据

**配准初值**:
- 使用ESKF预测位姿作为初值
- 减少配准迭代次数

## 扩展性设计

### 添加新传感器

1. 定义传感器数据结构 (src/sensor_data/)
2. 添加ROS消息转换 (FusionFlow)
3. 在Fusion中添加处理函数
4. 在ESKF中添加观测模型（如需要）

### 更换配准算法

实现RegistrationInterface接口：
```cpp
class MyRegistration : public RegistrationInterface {
  bool SetInputTarget(CloudPtr);
  bool ScanMatch(CloudPtr, pose_init, result);
  float GetFitnessScore();
};
```

### 添加新的可视化

继承或扩展PangolinWindow类，添加新的显示元素。

## 调试和监控

### 日志系统

使用Google Glog：
```cpp
LOG(INFO) << "信息日志";
LOG(WARNING) << "警告";
LOG(ERROR) << "错误";
```

### 可视化调试

Pangolin界面显示：
- 实时点云
- 地图
- 车辆轨迹
- 导航状态数值

RViz可视化：
- /current_scan: 当前扫描
- /map: 地图
- /odometry: 里程计
- /tf: 坐标变换

## 常见问题

### 1. 初始化失败

**可能原因**:
- RTK信号不好
- 地图与实际环境不匹配
- 配准得分低于阈值

**解决方法**:
- 检查RTK定位精度
- 调整搜索范围和步长
- 降低rtk_search_min_score_阈值

### 2. 定位漂移

**可能原因**:
- IMU零偏估计不准
- 配准失败
- 地图质量差

**解决方法**:
- 重新进行IMU初始化
- 检查配准得分
- 更新地图

### 3. 性能问题

**优化方向**:
- 增加点云降采样
- 减少NDT分辨率
- 优化地图分块大小

---

**维护者**: Zhang Jiaqi (zhangiaii97@gmail.com)

**最后更新**: 2025-12-25
