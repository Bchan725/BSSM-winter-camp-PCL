cmake_minimum_required(VERSION 3.10)
project(hello_ply)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# PCL dependencies (Linux only: pcl_ros 포함)
if(UNIX AND NOT APPLE)
  find_package(pcl_conversions REQUIRED)
  find_package(pcl_ros REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common io filters)
else()
  find_package(PCL REQUIRED COMPONENTS common io filters io)
endif()

# 실행 파일
add_executable(ply_publisher
  src/main.cpp
  src/ply_helper.cpp
)

# include 경로
target_include_directories(ply_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# link libraries
target_link_libraries(ply_publisher
  ${PCL_LIBRARIES}
  yaml-cpp
)

# ament dependencies 연결
ament_target_dependencies(ply_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  ament_index_cpp
)

# 설치
install(TARGETS ply_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# YAML config 설치
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
)

ament_package()
