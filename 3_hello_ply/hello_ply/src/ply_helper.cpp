#include "ply_helper.hpp"

PlyHelper::PlyHelper() : rclcpp::Node(NODE_NAME) {
	callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher_options_.callback_group = callback_group_;

    topic_name_ = "pointcloud"; // topic name 은 노드 이름으로 설정

    // original_cloud_ 초기화
    original_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    
    // Config 파일 로드
    loadConfig();

    original_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_,
                                                                                      1,
                                                                                      publisher_options_);

    downsampled_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_ + "_downsampled", 
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
    try {
        // 패키지 경로 가져오기
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("hello_ply");
        std::string config_file_path = package_share_directory + "/config/config.yaml";
        
        // YAML 파일 로드
        YAML::Node config = YAML::LoadFile(config_file_path);
        
        if (config["configs"]) {
            YAML::Node configs = config["configs"];
            
            // file_path 읽기 (상대 경로를 절대 경로로 변환)
            if (configs["file_path"]) {
                std::string relative_path = configs["file_path"].as<std::string>();
                if (relative_path[0] == '.' && relative_path[1] == '/') {
                    // ./data/nut.ply 같은 상대 경로를 절대 경로로 변환
                    file_path_ = package_share_directory + "/" + relative_path.substr(2);
                } else {
                    file_path_ = relative_path;
                }
                LOG_INFO("Config loaded - file_path: " << file_path_);
            }
            
            // downsample_radius 읽기
            if (configs["downsample_radius"]) {
                downsample_radius_ = configs["downsample_radius"].as<float>();
                LOG_INFO("Config loaded - downsample_radius: " << downsample_radius_);
            } else {
                downsample_radius_ = 0.01f; // 기본값
                LOG_WARN("downsample_radius not found in config, using default: " << downsample_radius_);
            }
            
            // scale_factor 읽기
            if (configs["scale_factor"]) {
                scale_factor_ = configs["scale_factor"].as<float>();
                LOG_INFO("Config loaded - scale_factor: " << scale_factor_);
            } else {
                scale_factor_ = 1.0f; // 기본값
                LOG_WARN("scale_factor not found in config, using default: " << scale_factor_);
            }
            
            // boundary_alpha 읽기
            if (configs["boundary_alpha"]) {
                boundary_alpha_ = configs["boundary_alpha"].as<float>();
                LOG_INFO("Config loaded - boundary_alpha: " << boundary_alpha_);
            } else {
                boundary_alpha_ = 0.005f;
                LOG_WARN("boundary_alpha not found in config, using default: " << boundary_alpha_);
            }
            
            // ransac_max_iterations 읽기
            if (configs["ransac_max_iterations"]) {
                ransac_max_iterations_ = configs["ransac_max_iterations"].as<int>();
                LOG_INFO("Config loaded - ransac_max_iterations: " << ransac_max_iterations_);
            } else {
                ransac_max_iterations_ = 1000;
                LOG_WARN("ransac_max_iterations not found in config, using default: " << ransac_max_iterations_);
            }
            
            // ransac_distance_threshold 읽기
            if (configs["ransac_distance_threshold"]) {
                ransac_distance_threshold_ = configs["ransac_distance_threshold"].as<float>();
                LOG_INFO("Config loaded - ransac_distance_threshold: " << ransac_distance_threshold_);
            } else {
                ransac_distance_threshold_ = 0.01f;
                LOG_WARN("ransac_distance_threshold not found in config, using default: " << ransac_distance_threshold_);
            }
            
            // normal_k_search 읽기
            if (configs["normal_k_search"]) {
                normal_k_search_ = configs["normal_k_search"].as<int>();
                LOG_INFO("Config loaded - normal_k_search: " << normal_k_search_);
            } else {
                normal_k_search_ = 20;
                LOG_WARN("normal_k_search not found in config, using default: " << normal_k_search_);
            }
            
            // square_box_spacing 읽기
            if (configs["square_box_spacing"]) {
                square_box_spacing_ = configs["square_box_spacing"].as<float>();
                LOG_INFO("Config loaded - square_box_spacing: " << square_box_spacing_);
            } else {
                square_box_spacing_ = 0.001f;
                LOG_WARN("square_box_spacing not found in config, using default: " << square_box_spacing_);
            }
            
            // up_tf_offset 읽기
            if (configs["up_tf_offset"]) {
                up_tf_offset_ = configs["up_tf_offset"].as<float>();
                LOG_INFO("Config loaded - up_tf_offset: " << up_tf_offset_);
            } else {
                up_tf_offset_ = 0.1f;
                LOG_WARN("up_tf_offset not found in config, using default: " << up_tf_offset_);
            }
        } else {
            LOG_ERROR("Config file structure invalid. Using default values.");
            downsample_radius_ = 0.01f;
            scale_factor_ = 1.0f;
            boundary_alpha_ = 0.005f;
            ransac_max_iterations_ = 1000;
            ransac_distance_threshold_ = 0.01f;
            normal_k_search_ = 20;
            square_box_spacing_ = 0.001f;
            up_tf_offset_ = 0.1f;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to load config file: " << e.what() << ". Using default values.");
        downsample_radius_ = 0.01f;
        scale_factor_ = 1.0f;
        boundary_alpha_ = 0.005f;
        ransac_max_iterations_ = 1000;
        ransac_distance_threshold_ = 0.01f;
        normal_k_search_ = 20;
        square_box_spacing_ = 0.001f;
        up_tf_offset_ = 0.1f;
    }
}

void PlyHelper::loadPlyFile(const std::string &file_path) {
	if (file_path.empty()) {
		LOG_ERROR("The path to read the ply file is empty.");
		return;
	}
  
	file_path_ = file_path; // 파일 경로 업데이트
	int load_result = pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(file_path_, *original_cloud_);

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

    // 포인트 클라우드의 중심(centroid) 계산
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*original_cloud_, centroid);

    // 모든 포인트를 중심으로 이동하고 스케일 적용
    for (auto& point : original_cloud_->points) {
        // 중심으로 이동 후 스케일 적용: (point - centroid) * scale_factor
        point.x = (point.x - centroid[0]) * scale_factor_;
        point.y = (point.y - centroid[1]) * scale_factor_;
        point.z = (point.z - centroid[2]) * scale_factor_;
    }

    LOG_INFO("Point cloud centered to origin and scaled by " << scale_factor_ 
            << ". Centroid was: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")");
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PlyHelper::downsampling() {
    if (!original_cloud_ || original_cloud_->points.empty()) {
        LOG_ERROR("Original cloud is empty. Load PLY file first.");
        return nullptr;
    }

    auto downsampled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();

    pcl::UniformSampling<pcl::PointXYZRGBNormal> uniform_sampling;

    uniform_sampling.setInputCloud(original_cloud_);
    uniform_sampling.setRadiusSearch(downsample_radius_);
    uniform_sampling.filter(*downsampled_cloud);

    LOG_INFO("Downsampled point cloud successfully. Original: " << original_cloud_->points.size()
            << " points, Downsampled: " << downsampled_cloud->points.size() << " points");

    return downsampled_cloud;
}

std::pair<
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, std::vector<pcl::Vertices>
> PlyHelper::extract_boundary_cloud(
	const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_cloud
)
{   
	if (!input_cloud || input_cloud->points.empty()) {
		LOG_ERROR("Input cloud is empty for boundary extraction.");
		return { std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(), std::vector<pcl::Vertices>() };
	}

	if (input_cloud->points.size() < 3) {
		LOG_ERROR("Input cloud has less than 3 points. Cannot extract boundary.");
		return { std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(), std::vector<pcl::Vertices>() };
	}

	LOG_INFO("Extracting boundary from " << input_cloud->points.size() << " points");

	auto boundary_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
	std::vector<pcl::Vertices> polygons;

	pcl::ConcaveHull<pcl::PointXYZRGBNormal> concave_hull;
	concave_hull.setInputCloud(input_cloud);  
	concave_hull.setAlpha(boundary_alpha_);

	concave_hull.reconstruct(*boundary_cloud, polygons);

	// 성공적으로 boundary를 추출했는지 확인
	if (boundary_cloud && !boundary_cloud->points.empty() && !polygons.empty()) {
		LOG_INFO("Boundary extraction successful with alpha=" << boundary_alpha_ 
				<< ": " << boundary_cloud->points.size() << " points, " << polygons.size() << " polygons");
		return { boundary_cloud, polygons };
	} else {
		LOG_ERROR("Boundary extraction failed with alpha=" << boundary_alpha_ 
				<< " (points: " << (boundary_cloud ? boundary_cloud->points.size() : 0) 
				<< ", polygons: " << polygons.size() << ")");
		return { std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(), std::vector<pcl::Vertices>() };
	}
}

void PlyHelper::change_pointcloud_color(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int red, int green, int blue) {
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

std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::ModelCoefficients::Ptr> 
PlyHelper::find_top_plane_ransac(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_cloud) {
    if (!input_cloud || input_cloud->points.empty()) {
        LOG_ERROR("Input cloud is empty for RANSAC plane detection.");
        return { std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(), 
                 std::make_shared<pcl::ModelCoefficients>() };
    }

    if (input_cloud->points.size() < 3) {
        LOG_ERROR("Input cloud has less than 3 points. Cannot find plane.");
        return { std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(), 
                 std::make_shared<pcl::ModelCoefficients>() };
    }

    LOG_INFO("Finding top plane using RANSAC from " << input_cloud->points.size() << " points...");

    // RANSAC 평면 추출
    pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_max_iterations_);
    seg.setDistanceThreshold(ransac_distance_threshold_);
    
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        LOG_ERROR("RANSAC failed to find a plane.");
        return { std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(), coefficients };
    }

    // 평면에 속한 포인트들 추출
    auto plane_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    for (int idx : inliers->indices) {
        plane_cloud->points.push_back(input_cloud->points[idx]);
    }
    plane_cloud->width = plane_cloud->points.size();
    plane_cloud->height = 1;
    plane_cloud->is_dense = true;

    LOG_INFO("RANSAC plane found: " << plane_cloud->points.size() << " inlier points");
    if (coefficients->values.size() >= 4) {
        LOG_INFO("Plane equation: " << coefficients->values[0] << "x + " 
                << coefficients->values[1] << "y + " 
                << coefficients->values[2] << "z + " 
                << coefficients->values[3] << " = 0");
    }

    return { plane_cloud, coefficients };
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr 
PlyHelper::get_center_hole(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& boundary_cloud, 
                            const std::vector<pcl::Vertices>& polygons) {
    if (!boundary_cloud || boundary_cloud->points.empty() || polygons.empty()) {
        LOG_ERROR("Input boundary cloud or polygons is empty.");
        return std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    }

    if (polygons.size() < 2) {
        LOG_WARN("Not enough polygons to find center hole. Using first polygon.");
        if (polygons.empty()) {
            return std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
        }
    }

    // 각 polygon의 크기를 저장
    std::vector<std::pair<int, size_t>> polygon_sizes;
    for (size_t i = 0; i < polygons.size(); i++) {
        polygon_sizes.emplace_back(i, polygons[i].vertices.size());
    }

    // 크기 기준으로 오름차순 정렬 (가장 작은 것이 첫 번째)
    std::sort(polygon_sizes.begin(), polygon_sizes.end(),
            [](const auto& a, const auto& b) {
                return a.second < b.second;
            });

    // 가장 작은 polygon 선택
    int smallest_idx = polygon_sizes[0].first;
    const auto& target_vertices = polygons[smallest_idx].vertices;

    LOG_INFO("Found " << polygons.size() << " polygons. Selecting smallest polygon (index " 
            << smallest_idx << ", size: " << target_vertices.size() << " vertices)");

    // 선택된 polygon의 vertices를 사용해서 포인트 클라우드 생성
    auto result = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    for (int idx : target_vertices) {
        if (idx >= 0 && static_cast<size_t>(idx) < boundary_cloud->points.size()) {
            result->points.push_back(boundary_cloud->points[idx]);
        }
    }

    result->width = result->points.size();
    result->height = 1;
    result->is_dense = true;

    LOG_INFO("Center hole extracted: " << result->points.size() << " points");

    return result;
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, geometry_msgs::msg::TransformStamped, geometry_msgs::msg::TransformStamped>
PlyHelper::create_center_square_box(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const std::string& frame_id) {
    if (!cloud || cloud->points.empty()) {
        LOG_ERROR("Input cloud is empty for square box creation.");
        return { std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(), 
                 geometry_msgs::msg::TransformStamped(),
                 geometry_msgs::msg::TransformStamped() };
    }

    // 1. normal, centroid, 좌표축 생성
    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
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
    
    // z축이 아래로 향하는 경우 반전 (z 성분이 음수면 반전)
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

    // 2. 평면 좌표계로 변환
    Eigen::Matrix3f R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    // 포인트의 각 x, y축에서 min, max 포인트를 찾음
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

    // 3. local 좌표 기준 네모 그리드 생성
    auto filled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    for (float lx = x_min; lx <= x_max; lx += square_box_spacing_) {
        for (float ly = y_min; ly <= y_max; ly += square_box_spacing_) {
            Eigen::Vector3f local_pt(lx, ly, 0);
            Eigen::Vector3f world_pt = R * local_pt + centroid;
            filled_cloud->points.emplace_back(world_pt.x(), world_pt.y(), world_pt.z());
        }
    }

    filled_cloud->width = filled_cloud->points.size();
    filled_cloud->height = 1;
    filled_cloud->is_dense = true;

    LOG_INFO("Square box created: " << filled_cloud->points.size() << " points");

    auto [tf_msg, up_tf_msg] = create_tf(centroid, z_axis, x_axis, y_axis, frame_id);

    return {filled_cloud, tf_msg, up_tf_msg};
}

std::pair<geometry_msgs::msg::TransformStamped, geometry_msgs::msg::TransformStamped>
PlyHelper::create_tf(const Eigen::Vector3f& centroid, const Eigen::Vector3f& z_axis, 
                     const Eigen::Vector3f& x_axis, const Eigen::Vector3f& y_axis, 
                     const std::string& frame_id) {
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
    Eigen::Quaternionf q(rotation_matrix);

    // TF 메시지 생성
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = frame_id;

    tf_msg.transform.translation.x = centroid.x();
    tf_msg.transform.translation.y = centroid.y();
    tf_msg.transform.translation.z = centroid.z();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    // 위로 100mm 올린 TF 생성
    geometry_msgs::msg::TransformStamped up_tf_msg;
    up_tf_msg.header.stamp = this->get_clock()->now();
    up_tf_msg.header.frame_id = "map";
    up_tf_msg.child_frame_id = "up_" + frame_id;

    Eigen::Matrix3f rot = q.toRotationMatrix();
    Eigen::Vector3f z_dir = rot.col(2);
    Eigen::Vector3f original_pos(centroid.x(), centroid.y(), centroid.z());
    Eigen::Vector3f offset_pos = original_pos + z_dir * up_tf_offset_;

    up_tf_msg.transform.translation.x = offset_pos.x();
    up_tf_msg.transform.translation.y = offset_pos.y();
    up_tf_msg.transform.translation.z = offset_pos.z();
    up_tf_msg.transform.rotation = tf_msg.transform.rotation;

    return {tf_msg, up_tf_msg};
}

void PlyHelper::process() {
	std::string file_path = "/Users/byungchan/Desktop/BSSM-winter-camp-PCL/3_hello_ply/data/nut.ply";
  	loadPlyFile(file_path);

	center_and_scale_pointcloud();

    auto downsampled_cloud = downsampling();

    if (!downsampled_cloud) {
        LOG_ERROR("Downsampling failed.");
        return;
    }

    // 1. 위쪽만 필터링 (z >= 0 조건만 사용)
    auto top_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    
    // z 범위 확인
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& point : downsampled_cloud->points) {
        if (point.z < min_z) min_z = point.z;
        if (point.z > max_z) max_z = point.z;
        
        // z >= 0 조건만 사용
        if (point.z >= 0.0f) {
            top_cloud->points.push_back(point);
        }
    }
    top_cloud->width = top_cloud->points.size();
    top_cloud->height = 1;
    top_cloud->is_dense = true;
    
    LOG_INFO("Downsampled cloud z range: min=" << min_z << ", max=" << max_z);
    LOG_INFO("Top cloud filtered: " << top_cloud->points.size() << " points (z >= 0) out of " << downsampled_cloud->points.size() << " total");

    if (top_cloud->points.empty()) {
        LOG_ERROR("Top cloud is empty after filtering.");
        return;
    }

    if (top_cloud->points.size() < 3) {
        LOG_ERROR("Top cloud has less than 3 points. Cannot extract boundary.");
        return;
    }

    // 2. RANSAC으로 위쪽 평면 찾기
    auto [plane_cloud, plane_coefficients] = find_top_plane_ransac(top_cloud);
    
    if (!plane_cloud || plane_cloud->points.empty()) {
        LOG_ERROR("RANSAC plane detection failed.");
        return;
    }
    
    LOG_INFO("RANSAC plane found: " << plane_cloud->points.size() << " points");

    // 3. 평면 포인트 색상을 초록색으로 변경
    change_pointcloud_color(plane_cloud, 0, 255, 0);

    // 4. Plane cloud publish
    sensor_msgs::msg::PointCloud2 plane_msg;
    pcl::toROSMsg<pcl::PointXYZRGBNormal>(*plane_cloud, plane_msg);
    plane_msg.header.frame_id = "map";
    plane_msg.header.stamp = this->get_clock()->now();
    plane_publisher_->publish(plane_msg);
    LOG_INFO("Plane cloud published with green color.");

    // 5. 외곽선 추출 (평면 포인트에서 추출)
    LOG_INFO("Attempting boundary extraction from plane_cloud with " << plane_cloud->points.size() << " points...");
    auto [boundary_cloud, polygons] = extract_boundary_cloud(plane_cloud);
    
    if (!boundary_cloud || boundary_cloud->points.empty()) {
        LOG_ERROR("Boundary cloud is empty after extraction. Plane cloud had " << plane_cloud->points.size() << " points.");
        return;
    }
    
    LOG_INFO("Boundary extracted: " << boundary_cloud->points.size() << " points, " << polygons.size() << " polygons");

    // 6. 포인트 색상을 빨간색으로 변경
    change_pointcloud_color(boundary_cloud, 255, 0, 0);

    // 7. Boundary cloud publish
    sensor_msgs::msg::PointCloud2 boundary_msg;
    pcl::toROSMsg<pcl::PointXYZRGBNormal>(*boundary_cloud, boundary_msg);
    boundary_msg.header.frame_id = "map";
    boundary_msg.header.stamp = this->get_clock()->now();
    boundary_publisher_->publish(boundary_msg);
    LOG_INFO("Boundary cloud published with red color.");

    // 8. Center hole 추출 (가장 작은 polygon 사용)
    auto center_hole_cloud = get_center_hole(boundary_cloud, polygons);
    
    if (!center_hole_cloud || center_hole_cloud->points.empty()) {
        LOG_WARN("Center hole cloud is empty.");
    } else {
        // 9. 포인트 색상을 파란색으로 변경
        change_pointcloud_color(center_hole_cloud, 0, 0, 255);

        // 10. Center hole cloud publish
        sensor_msgs::msg::PointCloud2 center_hole_msg;
        pcl::toROSMsg<pcl::PointXYZRGBNormal>(*center_hole_cloud, center_hole_msg);
        center_hole_msg.header.frame_id = "map";
        center_hole_msg.header.stamp = this->get_clock()->now();
        center_hole_publisher_->publish(center_hole_msg);
        LOG_INFO("Center hole cloud published with blue color.");

        // 11. Center square box 생성 및 TF publish
        auto [square_cloud, tf_msg, up_tf_msg] = create_center_square_box(center_hole_cloud, "hole_center");
        
        if (square_cloud && !square_cloud->points.empty()) {
            // Square box publish
            sensor_msgs::msg::PointCloud2 square_msg;
            pcl::toROSMsg<pcl::PointXYZ>(*square_cloud, square_msg);
            square_msg.header.frame_id = "map";
            square_msg.header.stamp = this->get_clock()->now();
            center_square_box_publisher_->publish(square_msg);
            LOG_INFO("Square box published: " << square_cloud->points.size() << " points");

            // TF publish
            tf_broadcaster_->sendTransform(tf_msg);
            tf_broadcaster_->sendTransform(up_tf_msg);
            LOG_INFO("TF published: " << tf_msg.child_frame_id << " and " << up_tf_msg.child_frame_id);
        }
    }

    // 11. Downsampled cloud publish
    sensor_msgs::msg::PointCloud2 pointcloud2_msg;
    pcl::toROSMsg<pcl::PointXYZRGBNormal>(*downsampled_cloud, pointcloud2_msg);
    pointcloud2_msg.header.frame_id = "map";

    // 초기 publish
    pointcloud2_msg.header.stamp = this->get_clock()->now();
    downsampled_publisher_->publish(pointcloud2_msg);
    LOG_INFO("Initial publish complete. Press Enter to republish...");

    // Enter 키를 누를 때마다 다시 publish
    int publish_count = 1;
    sensor_msgs::msg::PointCloud2 center_hole_msg;
    sensor_msgs::msg::PointCloud2 square_msg;
    geometry_msgs::msg::TransformStamped tf_msg;
    geometry_msgs::msg::TransformStamped up_tf_msg;
    
    if (center_hole_cloud && !center_hole_cloud->points.empty()) {
        pcl::toROSMsg<pcl::PointXYZRGBNormal>(*center_hole_cloud, center_hole_msg);
        center_hole_msg.header.frame_id = "map";
        
        // Square box와 TF도 미리 준비
        auto [square_cloud, tf_msg_temp, up_tf_msg_temp] = create_center_square_box(center_hole_cloud, "hole_center");
        if (square_cloud && !square_cloud->points.empty()) {
            pcl::toROSMsg<pcl::PointXYZ>(*square_cloud, square_msg);
            square_msg.header.frame_id = "map";
            tf_msg = tf_msg_temp;
            up_tf_msg = up_tf_msg_temp;
        }
    }
    
    while (rclcpp::ok()) {
        if (std::getchar() == '\n') {
            pointcloud2_msg.header.stamp = this->get_clock()->now();
            downsampled_publisher_->publish(pointcloud2_msg);
            LOG_INFO("Publish count: " << publish_count++);

            plane_msg.header.stamp = this->get_clock()->now();
            plane_publisher_->publish(plane_msg);

            boundary_msg.header.stamp = this->get_clock()->now();
            boundary_publisher_->publish(boundary_msg);

            if (center_hole_cloud && !center_hole_cloud->points.empty()) {
                center_hole_msg.header.stamp = this->get_clock()->now();
                center_hole_publisher_->publish(center_hole_msg);
                
                if (square_msg.width > 0) {
                    square_msg.header.stamp = this->get_clock()->now();
                    center_square_box_publisher_->publish(square_msg);
                    
                    tf_msg.header.stamp = this->get_clock()->now();
                    up_tf_msg.header.stamp = this->get_clock()->now();
                    tf_broadcaster_->sendTransform(tf_msg);
                    tf_broadcaster_->sendTransform(up_tf_msg);
                }
            }
        }
    }

    LOG_INFO("Process thread finished.");
}