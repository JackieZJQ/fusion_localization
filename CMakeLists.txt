cmake_minimum_required(VERSION 3.5)
project(fusion_localization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}
                    ${PROJECT_SOURCE_DIR}/src
                    ${PROJECT_SOURCE_DIR}/third_party
                    ${PROJECT_SOURCE_DIR}/third_party/sophus
                    ${EIGEN3_INCLUDE_DIR}
                    ${Glog_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS})              

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE UTM_CONVERT_SRCS "*.cc")
file(GLOB_RECURSE NODE_SRCS "src/app/fusion_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(fusion_localization_node "src/app/fusion_node.cpp" ${ALL_SRCS} ${UTM_CONVERT_SRCS})
ament_target_dependencies(fusion_localization_node rclcpp sensor_msgs nav_msgs pcl_conversions ndt_omp OpenMP)
target_link_libraries(fusion_localization_node ${PCL_LIBRARIES} ${GLOG_LIBRARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES} TBB::tbb yaml-cpp)

install(TARGETS
  fusion_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
