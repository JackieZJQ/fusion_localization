cmake_minimum_required(VERSION 3.5)
project(fusion_localization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
# todo
# 有bug
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -flto ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenMP REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}
                    ${PROJECT_SOURCE_DIR}/src
                    ${PROJECT_SOURCE_DIR}/third_party
                    ${PROJECT_SOURCE_DIR}/third_party/sophus
                    ${PROJECT_SOURCE_DIR}/third_party/fast_gicp/include
                    ${EIGEN3_INCLUDE_DIR}
                    ${Glog_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})              

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE UTM_CONVERT_SRCS "*.cc")
file(GLOB_RECURSE NODE_SRCS "src/app/fusion_node.cpp")

list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/third_party/fast_gicp/.*")

# 关闭fast_gicp的 apps/test/python，避免多余目标
set(BUILD_apps OFF CACHE BOOL "" FORCE)
set(BUILD_test OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
# 如果要 CUDA，就开 ON，否则 OFF
set(BUILD_VGICP_CUDA OFF CACHE BOOL "" FORCE)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/fast_gicp)

add_executable(fusion_localization_node "src/app/fusion_node.cpp" ${ALL_SRCS} ${UTM_CONVERT_SRCS})
ament_target_dependencies(fusion_localization_node 
  rclcpp 
  sensor_msgs 
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs 
  pcl_conversions 
  OpenMP)

target_link_libraries(fusion_localization_node 
  ${PCL_LIBRARIES} 
  ${GLOG_LIBRARIES} 
  ${OpenCV_LIBS} 
  TBB::tbb 
  yaml-cpp
  fast_gicp)

install(TARGETS
  fusion_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
