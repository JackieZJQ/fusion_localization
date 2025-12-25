# Fusion Localization (融合定位)

## 项目概述

这是一个高精度的自动驾驶定位模块，通过融合多种传感器数据实现精确的车辆定位。该系统采用**误差状态卡尔曼滤波器（Error-State Kalman Filter, ESKF）**融合以下传感器数据：

- **IMU（惯性测量单元）**：提供高频率的加速度和角速度测量
- **RTK-GNSS（实时动态差分GPS）**：提供高精度的全球定位
- **LiDAR（激光雷达）**：通过点云配准与预先构建的地图进行匹配定位

## 核心功能

### 1. 多传感器融合定位
- 使用18维ESKF进行状态估计（位置、速度、姿态、IMU零偏、重力）
- 支持IMU静态初始化，自动估计初始零偏和重力方向
- RTK初始化：在RTK附近进行网格搜索找到准确的初始位置
- 点云配准：使用NDT（Normal Distributions Transform）算法进行激光点云与地图的配准

### 2. 数据同步与处理
- IMU、GNSS、LiDAR数据的时间同步
- 点云畸变矫正（基于IMU预测的运动补偿）
- 坐标系转换（UTM坐标系、车体坐标系、LiDAR坐标系）

### 3. 可视化
- 基于Pangolin的3D可视化界面
- 实时显示当前扫描、地图、车辆轨迹
- 支持ROS2的rviz可视化

## 系统架构

```
融合定位系统架构：

传感器输入层：
├── IMU数据 (sensor_msgs/Imu)
├── GNSS数据 (sensor_msgs/NavSatFix) 
└── LiDAR点云 (sensor_msgs/PointCloud2)
         ↓
数据同步层 (MessageSync)：
├── 时间对齐
└── 数据缓存
         ↓
融合定位核心 (Fusion)：
├── 状态：WAITING_FOR_RTK → WORKING
├── IMU初始化 (StaticIMUInit)
├── RTK搜索 (SearchRTK - 网格搜索)
├── 预测 (Predict - ESKF预测)
├── 去畸变 (Undistort - 点云运动补偿)
└── 配准 (Align - NDT点云配准)
         ↓
输出层：
├── 位姿估计 (NavState)
├── ROS2话题发布 (Odometry, TF)
└── 可视化 (Pangolin窗口)
```

## 主要模块说明

### Fusion（融合定位核心）
位于 `src/fusion/fusion.hpp` 和 `fusion.cpp`

**主要职责：**
- 管理整个定位系统的状态机
- 处理同步后的传感器数据
- 执行ESKF预测、点云去畸变、配准等核心算法

**关键方法：**
- `ProcessMeasurements()`: 处理同步后的IMU和LiDAR数据
- `TryInitIMU()`: 使用静态数据初始化IMU零偏和重力
- `SearchRTK()`: 在RTK附近搜索准确的车辆位置
- `Predict()`: 利用IMU进行状态预测
- `Undistort()`: 对点云进行畸变矫正
- `Align()`: 执行点云配准并更新状态

### ESKF（误差状态卡尔曼滤波器）
位于 `src/models/kalmanfilter/eskf.hpp`

**18维状态向量：**
- 位置 (p): 3维
- 速度 (v): 3维
- 旋转 (R): 3维（so3表示）
- 陀螺仪零偏 (bg): 3维
- 加速度计零偏 (ba): 3维
- 重力 (grav): 3维

**主要功能：**
- IMU运动预测
- GNSS观测更新
- 状态协方差传播

### RegistrationManager（配准管理器）
位于 `src/models/registration/`

**功能：**
- 管理NDT配准算法
- 设置目标点云（地图）
- 执行点云匹配，返回配准位姿和得分

### MapLoader（地图加载器）
位于 `src/map/map_loader.hpp`

**功能：**
- 加载预先构建的点云地图
- 管理地图分块，支持大范围地图
- 根据当前位置动态加载/卸载地图区块

### FusionFlow（数据流控制）
位于 `src/fusion/fusion_flow.hpp`

**功能：**
- 订阅ROS2传感器话题
- 将ROS消息转换为内部数据结构
- 发布定位结果到ROS2话题

## 依赖项

### 必需依赖
- **ROS2**: 机器人操作系统框架
- **Eigen3**: 线性代数库
- **PCL**: 点云处理库
- **Sophus**: 李群库（用于SO3、SE3表示）
- **Glog**: Google日志库
- **yaml-cpp**: YAML配置文件解析
- **OpenCV**: 计算机视觉库
- **Pangolin**: 3D可视化库
- **ndt_omp**: NDT配准算法的OpenMP并行实现
- **TBB**: Intel线程构建模块

### 可选依赖
- **OpenMP**: 并行计算

## 构建说明

### 前置条件
1. 安装ROS2（推荐Humble或更新版本）
2. 安装所有依赖库

### 构建步骤
```bash
# 进入ROS2工作空间
cd ~/ros2_ws/src

# 克隆仓库
git clone https://github.com/JackieZJQ/fusion_localization.git

# 返回工作空间根目录
cd ~/ros2_ws

# 构建
colcon build --packages-select fusion_localization

# 设置环境
source install/setup.bash
```

## 运行说明

### 配置文件
配置文件位于 `config/` 目录：
- `localization_robosense.yaml`: 定位模式配置
- `mapping_robosense.yaml`: 建图模式配置
- `robosense.yaml`: 传感器参数配置

### 运行节点
```bash
# 运行融合定位节点
ros2 run fusion_localization fusion_localization_node
```

### 话题订阅
节点订阅以下话题：
- `/imu/data`: IMU数据
- `/gnss/fix`: GNSS定位数据
- `/lidar/points`: LiDAR点云数据

### 话题发布
节点发布以下话题：
- `/current_scan`: 当前帧点云
- `/map`: 地图点云
- `/odometry`: 定位结果
- `/tf`: 坐标变换

## 目录结构

```
fusion_localization/
├── config/                    # 配置文件
│   ├── localization_robosense.yaml
│   ├── mapping_robosense.yaml
│   └── robosense.yaml
├── src/
│   ├── app/                   # 应用程序入口
│   │   └── fusion_node.cpp    # ROS2节点主程序
│   ├── common/                # 通用工具
│   │   ├── eigen_types.hpp    # Eigen类型定义
│   │   ├── lidar_utils.hpp    # LiDAR工具函数
│   │   ├── math_utils.h       # 数学工具函数
│   │   ├── point_types.h      # 点云类型定义
│   │   └── timer.hpp          # 计时器
│   ├── fusion/                # 融合定位核心
│   │   ├── fusion.hpp/cpp     # 融合定位主类
│   │   └── fusion_flow.hpp/cpp # 数据流控制
│   ├── map/                   # 地图管理
│   │   └── map_loader.hpp/cpp # 地图加载器
│   ├── models/                # 算法模型
│   │   ├── converter/         # 数据转换
│   │   │   ├── cloud_convert.hpp/cpp  # 点云转换
│   │   │   └── utm_convert.hpp/cpp    # UTM坐标转换
│   │   ├── initialization/    # 初始化
│   │   │   └── static_imu_init.hpp/cpp # IMU静态初始化
│   │   ├── kalmanfilter/      # 卡尔曼滤波
│   │   │   └── eskf.hpp       # 误差状态卡尔曼滤波器
│   │   ├── registration/      # 点云配准
│   │   │   ├── registration_interface.hpp    # 配准接口
│   │   │   ├── registration_manager.hpp/cpp  # 配准管理器
│   │   │   ├── ndt_manager.hpp/cpp           # NDT配准
│   │   │   └── ndtomp_registration.hpp/cpp   # NDT OMP实现
│   │   └── synchronization/   # 数据同步
│   │       └── measure_sync.hpp/cpp # 测量同步器
│   ├── sensor_data/           # 传感器数据结构
│   │   ├── cloud_data.hpp     # 点云数据
│   │   ├── gnss_data.hpp      # GNSS数据
│   │   ├── imu_data.hpp       # IMU数据
│   │   ├── nav_state.hpp      # 导航状态
│   │   └── odom_data.hpp      # 里程计数据
│   └── ui/                    # 用户界面
│       ├── pangolin_window.hpp/cpp      # Pangolin窗口
│       ├── pangolin_window_impl.hpp/cpp # 窗口实现
│       ├── ui_car.hpp/cpp               # 车辆显示
│       ├── ui_cloud.hpp/cpp             # 点云显示
│       └── ui_trajectory.hpp/cpp        # 轨迹显示
├── third_party/               # 第三方库
│   └── sophus/                # Sophus李群库
├── CMakeLists.txt             # CMake构建文件
├── package.xml                # ROS2包描述文件
└── README.md                  # 本文件
```

## 工作流程

### 1. 初始化阶段（WAITING_FOR_RTK）
```
1. 启动节点
2. 等待IMU数据进行静态初始化
   - 收集静态IMU数据
   - 估计陀螺仪和加速度计零偏
   - 估计重力方向
3. 等待第一个RTK数据
4. 在RTK附近进行网格搜索
   - 生成搜索网格
   - 对每个网格点进行NDT配准
   - 选择配准得分最高的位置作为初始位置
5. 切换到WORKING状态
```

### 2. 正常工作阶段（WORKING）
```
循环处理每一帧数据：
1. 预测（Predict）
   - 使用IMU数据进行ESKF预测
   - 更新位置、速度、姿态
   - 传播协方差矩阵

2. 去畸变（Undistort）
   - 利用预测的运动轨迹
   - 对点云进行运动补偿
   - 生成去畸变后的点云

3. 配准（Align）
   - 使用NDT算法配准点云与地图
   - 获得配准位姿和得分
   - 使用配准结果更新ESKF状态
   
4. 发布结果
   - 发布当前位姿
   - 发布TF变换
   - 更新可视化
```

## 关键算法

### NDT配准
使用正态分布变换（Normal Distributions Transform）进行点云配准：
- 将目标点云（地图）划分为体素网格
- 每个体素用正态分布表示
- 通过优化目标函数找到最佳配准位姿

### ESKF融合
误差状态卡尔曼滤波器融合多传感器数据：
- **预测步**：使用IMU数据预测状态和协方差
- **更新步**：使用GNSS和点云配准结果校正状态

### 点云去畸变
补偿LiDAR扫描过程中的运动：
- LiDAR扫描需要一定时间（如100ms）
- 在扫描期间车辆在运动
- 使用IMU预测的运动轨迹对每个点进行补偿

## 性能特点

- **高精度**：融合多传感器，定位精度可达厘米级
- **鲁棒性**：即使某个传感器暂时失效，系统仍能工作
- **实时性**：使用NDT OMP并行加速，支持实时定位
- **可扩展**：模块化设计，易于添加新的传感器或算法

## 注意事项

1. **地图要求**：需要预先构建点云地图
2. **RTK信号**：初始化需要良好的RTK信号
3. **坐标系**：正确配置传感器外参（LiDAR相对IMU的位置和姿态）
4. **配置文件**：根据实际传感器调整配置参数

## 参考资料

本项目基于《自动驾驶中的SLAM技术》一书第10章的内容实现。

## 作者

Zhang Jiaqi (zhangiaii97@gmail.com)

## 许可证

仅供学习和研究使用，禁止商业用途。

---

**注**：此项目为教学示例项目，展示了多传感器融合定位的完整实现流程。
