cmake_minimum_required(VERSION 3.10)

# CMake 정책 설정 (경고 제거) - project() 전에 설정
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)  # find_package uses <PackageName>_ROOT variables
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)  # find_package uses upper-case <PACKAGENAME>_ROOT variables
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)  # FindBoost module compatibility
endif()

# PCL 내부 경고 억제를 위한 설정
set(CMAKE_WARN_DEPRECATED OFF)

project(hello_ply)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# PCL 관련 패키지 찾기 전에 정책 재설정 (PCL 내부 경고 억제)
# PCL이 사용하는 FindBoost 모듈을 위해 OLD 정책 유지
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()
# FLANN_ROOT 변수 호환성을 위해 NEW 정책 설정
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# 개발자 경고 억제 (PCL 내부 경고 제거)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.25")
  set(CMAKE_POLICY_DEFAULT_CMP0167 OLD)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

# PCL 관련 패키지 찾기
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 실행 파일 추가
add_executable(ply_publisher 
  src/main.cpp
  src/ply_helper.cpp
)
target_include_directories(ply_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS})
target_link_libraries(ply_publisher ${PCL_LIBRARIES} yaml-cpp)
ament_target_dependencies(ply_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  ament_index_cpp)

# 실행 파일 설치
install(TARGETS ply_publisher
  DESTINATION lib/${PROJECT_NAME})

# config 파일 설치
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
