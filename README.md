# fusion_localization
自动驾驶定位模块

## 简介

融合定位模块，基于LiDAR点云、IMU和GNSS实现高精度定位。采用NDT配准和ESKF滤波器融合多传感器数据。

## 快速开始

### 编译

```bash
cd fusion_localization_ws
colcon build --packages-select fusion_localization
source install/setup.bash
```

### 运行

使用默认配置启动：
```bash
ros2 launch fusion_localization launch.py
```

使用自定义配置：
```bash
ros2 launch fusion_localization launch.py config_path:=/path/to/your/config.yaml
```

启用rosbag回放：
```bash
ros2 launch fusion_localization launch.py use_bag:=true bag_path:=/path/to/your/bag
```

## 配置

配置文件示例位于 `config/localization_robosense.yaml`。主要参数包括：

- **map_data**: 点云地图数据目录
- **origin**: 地图原点坐标（UTM）
- **tile_size_meter**: 地图瓦片大小
- **ndt**: NDT配准参数
- **imu**: IMU初始化参数

## 主要特性

- ✅ 多传感器融合（LiDAR + IMU + GNSS）
- ✅ NDT点云配准定位
- ✅ ESKF滤波器
- ✅ 地图瓦片动态加载
- ✅ 高效的缓存机制
- ✅ 线程安全设计

## 最近优化

查看详细的优化说明：
- [OPTIMIZATION_NOTES.md](OPTIMIZATION_NOTES.md) - 优化详情和使用指南
- [CHANGES_SUMMARY.md](CHANGES_SUMMARY.md) - 完整变更总结

主要改进包括：
- 🚀 消除硬编码路径，支持灵活部署
- 🚀 优化地图加载性能（缓存机制）
- 🚀 减少不必要的NDT重新初始化
- 🚀 改进错误处理和鲁棒性
- 🚀 提升代码质量和可维护性

## 依赖

- ROS2 (Humble或更新版本)
- PCL (Point Cloud Library)
- Eigen3
- OpenCV
- yaml-cpp
- glog
- ndt_omp
- TBB

## 许可证

查看 [LICENSE](LICENSE) 文件了解详情。

## 贡献

欢迎提交问题和改进建议！
