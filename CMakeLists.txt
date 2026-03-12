cmake_minimum_required(VERSION 3.5)
project(fusion_localization)

# 允许 *_ROOT 变量，并让子目录沿用
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)          # 当前目录
  set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)  # 传递给子目录
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG   "-g ${CMAKE_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ---------------- deps ----------------
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(beidou_ins_driver REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenMP REQUIRED)

# ---------------- sources ----------------
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE UTM_CONVERT_SRCS "*.cc")
file(GLOB_RECURSE NODE_SRCS "src/app/fusion_node.cpp")
file(GLOB_RECURSE INS_NODE_SRCS "src/app/ins_node.cpp")
file(GLOB_RECURSE IMU_PRED_NODE_SRCS "src/app/imu_predictor_node.cpp")

list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${INS_NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${IMU_PRED_NODE_SRCS})  # ★ 新增
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/third_party/fast_gicp/.*")

# fast_gicp 选项
set(BUILD_apps OFF CACHE BOOL "" FORCE)
set(BUILD_test OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(BUILD_VGICP_CUDA OFF CACHE BOOL "" FORCE)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/fast_gicp)

# ---------------- core library ----------------
add_library(fusion_localization_core SHARED
  ${ALL_SRCS}
  ${UTM_CONVERT_SRCS}
)

target_include_directories(fusion_localization_core PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/third_party
  ${PROJECT_SOURCE_DIR}/third_party/sophus
  ${PROJECT_SOURCE_DIR}/third_party/fast_gicp/include
  ${EIGEN3_INCLUDE_DIR}
  ${Glog_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(fusion_localization_core
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  localization_msgs
  message_filters
  beidou_ins_driver
  tf2
  tf2_ros
  pcl_conversions
)

target_link_libraries(fusion_localization_core
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
  TBB::tbb
  fast_gicp
  OpenMP::OpenMP_CXX
)

# ---------------- nodes ----------------
add_executable(fusion_localization_node src/app/fusion_node.cpp)
ament_target_dependencies(fusion_localization_node rclcpp)
target_link_libraries(fusion_localization_node fusion_localization_core)

add_executable(ins_localization_node src/app/ins_node.cpp)
ament_target_dependencies(ins_localization_node rclcpp)
target_link_libraries(ins_localization_node fusion_localization_core)

add_executable(imu_predictor_node src/app/imu_predictor_node.cpp)
ament_target_dependencies(imu_predictor_node rclcpp)
target_link_libraries(imu_predictor_node fusion_localization_core)

# ---------------- install ----------------
install(TARGETS fusion_localization_core
  EXPORT export_fusion_localization_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS fusion_localization_node ins_localization_node imu_predictor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()