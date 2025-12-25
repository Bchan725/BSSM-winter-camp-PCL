#include "ply_helper.hpp"

/*
1. Ply 데이터를 불러온다 .

2. 다운샘플링 진행

3. 다운샘플링된 Pointcloud 원점으로 이동 ( 0,0,0 )

4. 원점으로 이동된 Pointcloud Publihser ( topic name: base )

5. 윗 면의 평면을 찾아 RANSAC 알고리즘을 사용하여 평면을 찾는다.

6. 평면을 찾은 후 중간 구멍의 원점을 찾음 ( 이때 기울기도 같이 ) ( topic name : hole_center_ply )

7. 중간 구멍의 위치를 Publihser ( topic name: hole_center_tf )
*/


int main(int argc, char * argv[])
{
  	rclcpp::init(argc, argv);

  	rclcpp::executors::MultiThreadedExecutor executor;

	auto node_ptr = std::make_shared<PlyHelper>();
	executor.add_node(node_ptr);
	executor.spin();

	rclcpp::shutdown();
        
	return 0;
}
