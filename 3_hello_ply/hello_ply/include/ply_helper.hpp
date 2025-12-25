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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>

// 기타
#include <thread>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#define LOG_INFO(x) RCLCPP_INFO_STREAM(this->get_logger(), x);
#define LOG_WARN(x) RCLCPP_WARN_STREAM(this->get_logger(), x);
#define LOG_ERROR(x) RCLCPP_ERROR_STREAM(this->get_logger(), x);

#define NODE_NAME "ply_helper"

using CloudT = pcl::PointXYZRGBNormal; // 포인트 클라우드 타입
using CloudPtr = pcl::PointCloud<CloudT>::Ptr; // 포인트 클라우드 포인터

class PlyHelper : public rclcpp::Node // rclcpp::Node를 상속받음
{
public:
    PlyHelper();
    ~PlyHelper();

public:
    void loadPlyFile(const std::string &file_path);
    void center_and_scale_pointcloud();

    void downsampling(CloudPtr &output_cloud); // 다운샘플링

    void extract_boundary_cloud( // 외곽 경계 추출
        const CloudPtr &input_cloud,
        CloudPtr &output_cloud,
        std::vector<pcl::Vertices> &output_polygons); 

    void find_top_plane_ransac( // RANSAC 평면 추출
        const CloudPtr &input_cloud,
        CloudPtr &output_cloud,
        pcl::ModelCoefficients::Ptr &output_coefficients); 
    
    void get_center_hole( // 중간 구멍 추출
        const CloudPtr& boundary_cloud,
        const std::vector<pcl::Vertices>& polygons,
        CloudPtr& output_cloud
    );

    void create_center_square_box( // 중간 사각형 생성
        const CloudPtr &cloud,
        const std::string &frame_id,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
        geometry_msgs::msg::TransformStamped &output_tf,
        geometry_msgs::msg::TransformStamped &output_up_tf);

    void create_tf( // TF 생성
        const Eigen::Vector3f &centroid,
        const Eigen::Vector3f &z_axis, const Eigen::Vector3f &x_axis,
        const Eigen::Vector3f &y_axis, const std::string &frame_id,
        geometry_msgs::msg::TransformStamped &output_tf,
        geometry_msgs::msg::TransformStamped &output_up_tf);    

    void extract_offset_z( // z offset 기준으로 위쪽 포인트 클라우드 추출
        const CloudPtr& input_cloud,
        CloudPtr& output_cloud,
        float z_offset = 0.0f
    );

    void change_pointcloud_color( // 포인트 클라우드 색상 변경
        CloudPtr cloud, 
        int red, 
        int green, 
        int blue
    ); 

private:
    void process(); // 스레드에서 실행되는 함수
    void loadConfig(); // config.yaml 파일 로드
    
    // 포인트 클라우드 publish 헬퍼 (템플릿)
    template<typename PointT>
    void publishPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, 
                          rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                          const std::string& frame_id = "map") {
        if (!cloud || cloud->points.empty() || !publisher) {
            return;
        }
        
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg<PointT>(*cloud, msg);
        msg.header.frame_id = frame_id;
        msg.header.stamp = this->get_clock()->now();
        publisher->publish(msg);
    }
    
    // CloudPtr를 위한 오버로드 (타입 추론을 위해)
    void publishPointCloud(const CloudPtr& cloud, 
                          rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                          const std::string& frame_id = "map") {
        publishPointCloud<CloudT>(cloud, publisher, frame_id);
    }

private:
    // ROS2 Callback group 및 publisher 옵션
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::PublisherOptions publisher_options_;

    // 포인트 클라우드 Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr original_publisher_;           // 원본 포인트클라우드
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_publisher_;        // 다운샘플링 포인트클라우드
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr top_cloud_publisher_;          // 상단 필터링 포인트클라우드
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_publisher_;           // 외곽 경계 포인트클라우드
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_publisher_;              // 평면 포인트클라우드
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr center_hole_publisher_;        // 중심 구멍 포인트클라우드
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr center_square_box_publisher_;  // 중심 사각형 포인트클라우드

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 포인트 클라우드 데이터
    CloudPtr original_cloud_;

    // 내부 변수
    std::string file_path_;    // 파일 경로
    std::string topic_name_;   // 토픽 이름
    std::thread thread_;       // 백그라운드 처리 스레드

    // config.yaml 로드 파라미터
    float downsample_radius_;       // 다운샘플링 반경
    float scale_factor_;            // 스케일 팩터
    float boundary_alpha_;          // 외곽 경계 추출 alpha 값
    int   ransac_max_iterations_;   // RANSAC 최대 반복 횟수
    float ransac_distance_threshold_; // RANSAC 거리 임계값
    int   normal_k_search_;         // 노말 계산 K search
    float square_box_spacing_;      // 중심 사각형 생성 시 간격
    float up_tf_offset_;            // 상단 TF 오프셋 (m)
};
