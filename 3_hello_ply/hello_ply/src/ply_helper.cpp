#include "ply_helper.hpp"

PlyHelper::PlyHelper() : rclcpp::Node(NODE_NAME) {
	callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher_options_.callback_group = callback_group_;

    topic_name_ = "pointcloud"; 

    original_cloud_ = std::make_shared<pcl::PointCloud<CloudT>>();
    
    loadConfig();

    original_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_,
                                                                                      1,
                                                                                      publisher_options_);

    downsampled_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_ + "_downsampled", 
                                                                                      1,
                                                                                      publisher_options_);

    offset_z_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_ + "_top", 
                                                                                      1,
                                                                                      publisher_options_);

    boundary_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_ + "_boundary", 
                                                                                      1,
                                                                                      publisher_options_);

    plane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_ + "_plane", 
                                                                                      1,
                                                                                      publisher_options_);

    center_hole_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_ + "_center_hole", 
                                                                                      1,
                                                                                      publisher_options_);

    center_square_box_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_ + "_center_square_box", 
                                                                                      1,
                                                                                      publisher_options_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    thread_ = std::thread(&PlyHelper::process, &(*this));
    thread_.detach();
}

PlyHelper::~PlyHelper()
{
}

void PlyHelper::loadConfig() {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("hello_ply");
    std::string config_file_path = package_share_directory + "/config/config.yaml";
    
    YAML::Node config = YAML::LoadFile(config_file_path);
    YAML::Node configs = config["configs"];
    
    std::string relative_path = configs["file_path"].as<std::string>();
    if (relative_path[0] == '.' && relative_path[1] == '/') {
        file_path_ = package_share_directory + "/" + relative_path.substr(2);
    } else {
        file_path_ = relative_path;
    }
    
    downsample_radius_ = configs["downsample_radius"].as<float>();
    scale_factor_ = configs["scale_factor"].as<float>();
    boundary_alpha_ = configs["boundary_alpha"].as<float>();
    ransac_max_iterations_ = configs["ransac_max_iterations"].as<int>();
    ransac_distance_threshold_ = configs["ransac_distance_threshold"].as<float>();
    normal_k_search_ = configs["normal_k_search"].as<int>();
    square_box_spacing_ = configs["square_box_spacing"].as<float>();
    up_tf_offset_ = configs["up_tf_offset"].as<float>();
    
    LOG_INFO("Config loaded successfully.");
}

void PlyHelper::loadPlyFile(const std::string &file_path) {
	if (file_path.empty()) {
		LOG_ERROR("The path to read the ply file is empty.");
		return;
	}
  
	file_path_ = file_path;
	int load_result = pcl::io::loadPLYFile<CloudT>(file_path_, *original_cloud_);

	if (load_result != 0) {
		LOG_ERROR("Failed to load ply file.");
		return;
	}

	LOG_INFO("Ply file loaded successfully.");
}

void PlyHelper::center_and_scale_pointcloud() {
    if (!original_cloud_ || original_cloud_->points.empty()) {
        LOG_ERROR("Original cloud is empty. Load PLY file first.");
        return;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*original_cloud_, centroid);

    for (auto& point : original_cloud_->points) {
        point.x = (point.x - centroid[0]) * scale_factor_;
        point.y = (point.y - centroid[1]) * scale_factor_;
        point.z = (point.z - centroid[2]) * scale_factor_;
    }

    LOG_INFO("Point cloud centered to origin and scaled by " << scale_factor_ << ".");
    LOG_INFO("Centroid: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")");
}

void PlyHelper::downsampling(CloudPtr& output_cloud) {
    if (!original_cloud_ || original_cloud_->points.empty()) {
        LOG_ERROR("Original cloud is empty. Load PLY file first.");
        output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
        return;
    }

    output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();

    pcl::UniformSampling<CloudT> uniform_sampling;

    uniform_sampling.setInputCloud(original_cloud_);
    uniform_sampling.setRadiusSearch(downsample_radius_);
    uniform_sampling.filter(*output_cloud);

    LOG_INFO("Downsampled point cloud successfully. Original: " << original_cloud_->points.size()
            << " points, Downsampled: " << output_cloud->points.size() << " points");
}

void PlyHelper::extract_boundary_cloud(
	const CloudPtr& input_cloud,
	CloudPtr& output_cloud,
	std::vector<pcl::Vertices>& output_polygons
)
{   
	if (!input_cloud || input_cloud->points.empty()) {
		LOG_ERROR("Input cloud is empty for boundary extraction.");
		output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
		output_polygons.clear();
		return;
	}

	if (input_cloud->points.size() < 3) {
		LOG_ERROR("Input cloud has less than 3 points. Cannot extract boundary.");
		output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
		output_polygons.clear();
		return;
	}

	LOG_INFO("Extracting boundary from " << input_cloud->points.size() << " points");

	output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
	output_polygons.clear();

	pcl::ConcaveHull<CloudT> concave_hull;
	concave_hull.setInputCloud(input_cloud);  
	concave_hull.setAlpha(boundary_alpha_);

	concave_hull.reconstruct(*output_cloud, output_polygons);

	if (output_cloud && !output_cloud->points.empty() && !output_polygons.empty()) {
		LOG_INFO("Boundary extraction successful with alpha=" << boundary_alpha_ 
				<< ": " << output_cloud->points.size() << " points, " << output_polygons.size() << " polygons");
	} else {
		LOG_ERROR("Boundary extraction failed with alpha=" << boundary_alpha_ 
				<< " (points: " << (output_cloud ? output_cloud->points.size() : 0) 
				<< ", polygons: " << output_polygons.size() << ")");
		output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
		output_polygons.clear();
	}
}

void PlyHelper::find_top_plane_ransac(
    const CloudPtr& input_cloud,
    CloudPtr& output_cloud,
    pcl::ModelCoefficients::Ptr& output_coefficients
) {
    if (!input_cloud || input_cloud->points.empty()) {
        LOG_ERROR("Input cloud is empty for RANSAC plane detection.");
        output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
        output_coefficients = std::make_shared<pcl::ModelCoefficients>();
        return;
    }

    if (input_cloud->points.size() < 3) {
        LOG_ERROR("Input cloud has less than 3 points. Cannot find plane.");
        output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
        output_coefficients = std::make_shared<pcl::ModelCoefficients>();
        return;
    }

    LOG_INFO("Finding top plane using RANSAC from " << input_cloud->points.size() << " points...");

    pcl::SACSegmentation<CloudT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    output_coefficients = std::make_shared<pcl::ModelCoefficients>();

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_max_iterations_);
    seg.setDistanceThreshold(ransac_distance_threshold_);
    
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *output_coefficients);

    if (inliers->indices.empty()) {
        LOG_ERROR("RANSAC failed to find a plane.");
        output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
        return;
    }

    output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
    for (int idx : inliers->indices) {
        output_cloud->points.push_back(input_cloud->points[idx]);
    }
    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
    output_cloud->is_dense = true;

    LOG_INFO("RANSAC plane found: " << output_cloud->points.size() << " inlier points");
    if (output_coefficients->values.size() >= 4) {
        LOG_INFO("Plane equation: " << output_coefficients->values[0] << "x + " 
                << output_coefficients->values[1] << "y + " 
                << output_coefficients->values[2] << "z + " 
                << output_coefficients->values[3] << " = 0");
    }
}

void PlyHelper::get_center_hole(
    const CloudPtr& boundary_cloud, 
    const std::vector<pcl::Vertices>& polygons,
    CloudPtr& output_cloud
) {
    if (!boundary_cloud || boundary_cloud->points.empty() || polygons.empty()) {
        LOG_ERROR("Input boundary cloud or polygons is empty.");
        output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
        return;
    }

    if (polygons.size() < 2) {
        LOG_WARN("Not enough polygons to find center hole. Using first polygon.");
        if (polygons.empty()) {
            output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
            return;
        }
    }

    std::vector<std::pair<int, size_t>> polygon_sizes;
    for (size_t i = 0; i < polygons.size(); i++) {
        polygon_sizes.emplace_back(i, polygons[i].vertices.size());
    }

    std::sort(polygon_sizes.begin(), polygon_sizes.end(),
            [](const auto& a, const auto& b) {
                return a.second < b.second;
            });

    int smallest_idx = polygon_sizes[0].first;
    const auto& target_vertices = polygons[smallest_idx].vertices;

    LOG_INFO("Found " << polygons.size() << " polygons. Selecting smallest polygon (index " 
            << smallest_idx << ", size: " << target_vertices.size() << " vertices)");

    output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
    for (int idx : target_vertices) {
        if (idx >= 0 && static_cast<size_t>(idx) < boundary_cloud->points.size()) {
            output_cloud->points.push_back(boundary_cloud->points[idx]);
        }
    }

    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
    output_cloud->is_dense = true;

    LOG_INFO("Center hole extracted: " << output_cloud->points.size() << " points");
}

void PlyHelper::create_center_square_box(
    const CloudPtr& cloud, 
    const std::string& frame_id,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
    geometry_msgs::msg::TransformStamped& output_tf,
    geometry_msgs::msg::TransformStamped& output_up_tf
) {
    if (!cloud || cloud->points.empty()) {
        LOG_ERROR("Input cloud is empty for square box creation.");
        output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output_tf = geometry_msgs::msg::TransformStamped();
        output_up_tf = geometry_msgs::msg::TransformStamped();
        return;
    }

    pcl::NormalEstimation<CloudT, pcl::Normal> ne;
    pcl::search::KdTree<CloudT>::Ptr tree(new pcl::search::KdTree<CloudT>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(normal_k_search_);
    ne.compute(*normals);

    Eigen::Vector3f avg_normal(0, 0, 0);
    for (const auto& n : normals->points) {
        avg_normal += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
    }
    avg_normal.normalize();
    
    if (avg_normal.z() < 0) {
        avg_normal = -avg_normal;
        LOG_INFO("Normal flipped to point upward (z was negative)");
    }

    Eigen::Vector4f centroid4f;
    pcl::compute3DCentroid(*cloud, centroid4f);
    Eigen::Vector3f centroid = centroid4f.head<3>();

    Eigen::Vector3f z_axis = avg_normal;
    Eigen::Vector3f x_axis = z_axis.unitOrthogonal();
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);

    Eigen::Matrix3f R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();

    for (const auto& pt : cloud->points) {
        Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);
        Eigen::Vector3f local = R.transpose() * (pt_vec - centroid);

        if (local.x() < x_min) x_min = local.x();
        if (local.x() > x_max) x_max = local.x();
        if (local.y() < y_min) y_min = local.y();
        if (local.y() > y_max) y_max = local.y();
    }

    output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    for (float lx = x_min; lx <= x_max; lx += square_box_spacing_) {
        for (float ly = y_min; ly <= y_max; ly += square_box_spacing_) {
            Eigen::Vector3f local_pt(lx, ly, 0);
            Eigen::Vector3f world_pt = R * local_pt + centroid;
            output_cloud->points.emplace_back(world_pt.x(), world_pt.y(), world_pt.z());
        }
    }

    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
    output_cloud->is_dense = true;

    LOG_INFO("Square box created: " << output_cloud->points.size() << " points");

    create_tf(centroid, z_axis, x_axis, y_axis, frame_id, output_tf, output_up_tf);
}

void PlyHelper::create_tf(
    const Eigen::Vector3f& centroid, 
    const Eigen::Vector3f& z_axis, 
    const Eigen::Vector3f& x_axis, 
    const Eigen::Vector3f& y_axis, 
    const std::string& frame_id,
    geometry_msgs::msg::TransformStamped& output_tf,
    geometry_msgs::msg::TransformStamped& output_up_tf
) {
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
    Eigen::Quaternionf q(rotation_matrix);

    output_tf.header.stamp = this->get_clock()->now();
    output_tf.header.frame_id = "map";
    output_tf.child_frame_id = frame_id;

    output_tf.transform.translation.x = centroid.x();
    output_tf.transform.translation.y = centroid.y();
    output_tf.transform.translation.z = centroid.z();
    output_tf.transform.rotation.x = q.x();
    output_tf.transform.rotation.y = q.y();
    output_tf.transform.rotation.z = q.z();
    output_tf.transform.rotation.w = q.w();

    output_up_tf.header.stamp = this->get_clock()->now();
    output_up_tf.header.frame_id = "map";
    output_up_tf.child_frame_id = "up_" + frame_id;

    Eigen::Matrix3f rot = q.toRotationMatrix();
    Eigen::Vector3f z_dir = rot.col(2);
    Eigen::Vector3f original_pos(centroid.x(), centroid.y(), centroid.z());
    Eigen::Vector3f offset_pos = original_pos + z_dir * up_tf_offset_;

    output_up_tf.transform.translation.x = offset_pos.x();
    output_up_tf.transform.translation.y = offset_pos.y();
    output_up_tf.transform.translation.z = offset_pos.z();
    output_up_tf.transform.rotation = output_tf.transform.rotation;
}

void PlyHelper::change_pointcloud_color(CloudPtr cloud, int red, int green, int blue) {
    if (!cloud || cloud->points.empty()) {
        LOG_ERROR("Cloud is empty. Cannot change color.");
        return;
    }

    // 값 범위 검증 (0~255)
    red = std::max(0, std::min(255, red));
    green = std::max(0, std::min(255, green));
    blue = std::max(0, std::min(255, blue));

    // 모든 포인트의 색상 변경
    for (auto& point : cloud->points) {
        point.r = static_cast<uint8_t>(red);
        point.g = static_cast<uint8_t>(green);
        point.b = static_cast<uint8_t>(blue);
    }

    LOG_INFO("Point cloud color changed to RGB(" << red << ", " << green << ", " << blue << ")");
}

void PlyHelper::extract_offset_z(
    const CloudPtr& input_cloud,
    CloudPtr& output_cloud,
    float z_offset
) {
    if (!input_cloud || input_cloud->points.empty()) {
        LOG_ERROR("Input cloud is empty for offset z extraction.");
        output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
        return;
    }

    output_cloud = std::make_shared<pcl::PointCloud<CloudT>>();
    
    // z 범위 확인
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& point : input_cloud->points) {
        if (point.z < min_z) min_z = point.z;
        if (point.z > max_z) max_z = point.z;
        
        if (point.z >= z_offset) {
            output_cloud->points.push_back(point);
        }
    }
    
    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
    output_cloud->is_dense = true;
    
    LOG_INFO("Input cloud z range: min=" << min_z << ", max=" << max_z);
    LOG_INFO("Extracted cloud: " << output_cloud->points.size() << " points (z >= " << z_offset << ") out of " << input_cloud->points.size() << " total");
}

void PlyHelper::process() {
	/*
	전체 흐름 설명

	1. 포인트 클라우드 로드
	2. 포인트 클라우드 중심 이동 및 스케일링
	3. 포인트 클라우드 다운샘플링
	4. 다운샘플링된 포인트 클라우드에서 Z 축이 0 이상인 포인트만 필터링 ( 노란색 )
	5. RANSAC으로 위쪽 평면 찾기 ( 초록색 )
	6. 외곽선 추출 (평면 포인트에서 추출) ( 빨간색 )
	7. 찾은 외곽 포인트들 중 가장 작은 포인트 클라우드를 중심 포인트 클라우드로 설정 ( 파란색 )
	8. 중앙 포인트 클라우드를 기준으로 사각형 생성 ( 흰색 )
	*/

	// 1. 포인트 클라우드 로드
  	loadPlyFile(file_path_);

    // 2. 포인트 클라우드 중심 이동 및 스케일링
	center_and_scale_pointcloud();

    // 3. 포인트 클라우드 다운샘플링
    CloudPtr downsampled_cloud;
    downsampling(downsampled_cloud);
    change_pointcloud_color(downsampled_cloud, 255, 255, 255);
    publishPointCloud(downsampled_cloud, downsampled_publisher_);

    // 4. 다운샘플링된 포인트 클라우드에서 Z 축이 0 이상인 포인트만 필터링 ( 노란색 )
    CloudPtr offset_z_cloud;
    extract_offset_z(downsampled_cloud, offset_z_cloud, 0.0f);
    change_pointcloud_color(offset_z_cloud, 255, 255, 0);
    publishPointCloud(offset_z_cloud, offset_z_cloud_publisher_);

	// 5. RANSAC으로 위쪽 평면 찾은 후 초록색으로 변경
	CloudPtr plane_cloud;
    pcl::ModelCoefficients::Ptr plane_coefficients;
    find_top_plane_ransac(offset_z_cloud, plane_cloud, plane_coefficients);
    change_pointcloud_color(plane_cloud, 0, 255, 0);
    publishPointCloud(plane_cloud, plane_publisher_);

    // 6. 외곽선 추출 (평면 포인트에서 추출) ( 빨간색 )
    CloudPtr boundary_cloud;
    std::vector<pcl::Vertices> polygons;
    extract_boundary_cloud(plane_cloud, boundary_cloud, polygons);
    change_pointcloud_color(boundary_cloud, 255, 0, 0);
    publishPointCloud(boundary_cloud, boundary_publisher_);

    // 7. 찾은 외곽 포인트들 중 가장 작은 포인트 클라우드를 중심 포인트 클라우드로 설정 ( 파란색 )
    CloudPtr center_hole_cloud;
    get_center_hole(boundary_cloud, polygons, center_hole_cloud);
    publishPointCloud(center_hole_cloud, center_hole_publisher_);

    // 8. 중앙 포인트 클라우드를 기준으로 사각형 생성 ( 흰색 )
    pcl::PointCloud<pcl::PointXYZ>::Ptr square_cloud;
    geometry_msgs::msg::TransformStamped tf_msg;
    geometry_msgs::msg::TransformStamped up_tf_msg;
    create_center_square_box(center_hole_cloud, "hole_center", square_cloud, tf_msg, up_tf_msg);
   
	publishPointCloud<pcl::PointXYZ>(square_cloud, center_square_box_publisher_);
	LOG_INFO("Square box published: " << square_cloud->points.size() << " points");

	tf_msg.header.stamp = this->get_clock()->now();
	up_tf_msg.header.stamp = this->get_clock()->now();
	tf_broadcaster_->sendTransform(tf_msg);
	tf_broadcaster_->sendTransform(up_tf_msg);

    // 메시지 준비 (while 루프에서 사용)
    sensor_msgs::msg::PointCloud2 downsampled_msg;
    sensor_msgs::msg::PointCloud2 offset_z_pcl_msg;
    sensor_msgs::msg::PointCloud2 plane_msg;
    sensor_msgs::msg::PointCloud2 boundary_msg;
    sensor_msgs::msg::PointCloud2 center_hole_msg;
    sensor_msgs::msg::PointCloud2 square_msg;
    
    pcl::toROSMsg<CloudT>(*downsampled_cloud, downsampled_msg);
    downsampled_msg.header.frame_id = "map";

    pcl::toROSMsg<CloudT>(*offset_z_cloud, offset_z_pcl_msg);
    offset_z_pcl_msg.header.frame_id = "map";

    pcl::toROSMsg<CloudT>(*plane_cloud, plane_msg);
    plane_msg.header.frame_id = "map";
    
    pcl::toROSMsg<CloudT>(*boundary_cloud, boundary_msg);
    boundary_msg.header.frame_id = "map";
    
    pcl::toROSMsg<CloudT>(*center_hole_cloud, center_hole_msg);
    center_hole_msg.header.frame_id = "map";
    
    if (square_cloud && !square_cloud->points.empty()) {
        pcl::toROSMsg<pcl::PointXYZ>(*square_cloud, square_msg);
        square_msg.header.frame_id = "map";
    }

    int publish_count = 1;
    
    while (rclcpp::ok()) {
        if (std::getchar() == '\n') {
            auto now = this->get_clock()->now();
            
            downsampled_msg.header.stamp = now;
            downsampled_publisher_->publish(downsampled_msg);

            offset_z_pcl_msg.header.stamp = now;
            offset_z_cloud_publisher_->publish(offset_z_pcl_msg);
            
            plane_msg.header.stamp = now;
            plane_publisher_->publish(plane_msg);
            
            boundary_msg.header.stamp = now;
            boundary_publisher_->publish(boundary_msg);
            
            center_hole_msg.header.stamp = now;
            center_hole_publisher_->publish(center_hole_msg);
            
            if (square_msg.width > 0) {
                square_msg.header.stamp = now;
                center_square_box_publisher_->publish(square_msg);
            }
            
            tf_msg.header.stamp = now;
            up_tf_msg.header.stamp = now;
            tf_broadcaster_->sendTransform(tf_msg);
            tf_broadcaster_->sendTransform(up_tf_msg);
            
            LOG_INFO("Publish count: " << publish_count++);
        }
    }

    LOG_INFO("Process thread finished.");
}