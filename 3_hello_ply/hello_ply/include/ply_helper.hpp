#pragma once

// ROS2 & 메시지
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// PCL Core
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL 필터 및 알고리즘
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>

// 기타
#include <ctime>
#include <algorithm>
#include <vector>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

#define LOG_INFO(x) RCLCPP_INFO_STREAM(this->get_logger(), x);
#define LOG_WARN(x) RCLCPP_WARN_STREAM(this->get_logger(), x);
#define LOG_ERROR(x) RCLCPP_ERROR_STREAM(this->get_logger(), x);

#define NODE_NAME "ply_helper"

using CloudT = pcl::PointXYZRGB; // 포인트 클라우드 타입
using CloudPtr = pcl::PointCloud<CloudT>::Ptr; // 포인트 클라우드 포인터

class PlyHelper : public rclcpp::Node // rclcpp::Node를 상속받음
{
public:
    PlyHelper();
    ~PlyHelper();

public:
    void loadPlyFile(const std::string &file_path);
    void center_and_scale_pointcloud();
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampling();
    std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, std::vector<pcl::Vertices>> extract_boundary_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_cloud);
    void change_pointcloud_color(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int red, int green, int blue);
    std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::ModelCoefficients::Ptr> find_top_plane_ransac(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_cloud);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr get_center_hole(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& boundary_cloud, const std::vector<pcl::Vertices>& polygons);
    std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, geometry_msgs::msg::TransformStamped, geometry_msgs::msg::TransformStamped> create_center_square_box(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const std::string& frame_id);
    std::pair<geometry_msgs::msg::TransformStamped, geometry_msgs::msg::TransformStamped> create_tf(const Eigen::Vector3f& centroid, const Eigen::Vector3f& z_axis, const Eigen::Vector3f& x_axis, const Eigen::Vector3f& y_axis, const std::string& frame_id);

private:
    void process(); // 스레드에서 실행되는 함수
    void loadConfig(); // config.yaml 파일 로드
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::PublisherOptions publisher_options_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr original_publisher_;     // 원본 포인트 클라우드 publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_publisher_;  // 다운샘플링된 포인트 클라우드 publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_publisher_;     // 외곽 포인트 클라우드 publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_publisher_;       // 평면 포인트 클라우드 publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr center_hole_publisher_;  // 중간 구멍 포인트 클라우드 publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr center_square_box_publisher_; // 중간 사각형 포인트 클라우드 publisher
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // tf 브로드캐스터

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr original_cloud_; // 원본 포인트 클라우드

    std::string file_path_; // 파일 경로
    std::string topic_name_; // topic name
    std::thread thread_; // 스레드
    
    // Config 값들
    float downsample_radius_; // 다운샘플링 반경
    float scale_factor_; // 스케일 팩터
    float boundary_alpha_; // Boundary extraction alpha 값
    int ransac_max_iterations_; // RANSAC 최대 반복 횟수
    float ransac_distance_threshold_; // RANSAC 거리 임계값
    int normal_k_search_; // Normal estimation K search
    float square_box_spacing_; // Square box 생성 간격
    float up_tf_offset_; // 위로 올린 TF 오프셋 (m)

// public:
//     void loadPlyFile(const std::string& config_path);
//     void downsamplePointCloud();
//     void centerToOrigin();

//     void find_top_plane();
//     void find_hole_center();
    
//     // Publisher 메서드
//     void publishOriginCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);
//     void publishBaseCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);
};
