#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/moment_of_inertia_estimation.h>

int main(int argc, char* argv[]) {
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("table_scene_lms400_downsampled.pcd", *cloud) == -1)
		return -1;

	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();  // 直接开始特征计算，不用传对象去接收结果

	std::vector<float> moment_of_inertia;  // 存放惯性矩的特征向量
	std::vector<float> eccentricity;  // 存放偏心率的特征向量
	float major_value, middle_value, minor_value;  // 三个特征值
	Eigen::Vector3f major_vector, middle_vector, minor_vector;  // 三个特征向量
	Eigen::Vector3f mass_center;  // 质心

	// 这四个是OBB对应的参数
	pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	// 这是AABB对应的参数
	pcl::PointXYZ min_point_AABB, max_point_AABB;

	feature_extractor.getMomentOfInertia(moment_of_inertia);  // 得到惯性矩
	feature_extractor.getEccentricity(eccentricity);  // 得到偏心率

	// 获取OBB盒子
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	// 获取AABB盒子
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);

	// 获取主轴major_vector，中轴middle_vector，辅助轴minor_vector
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);  // 获取质心

	// 可视化（先把初始点云添加进去）
	pcl::visualization::PCLVisualizer viewer("viewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	viewer.addPointCloud<pcl::PointXYZ>(cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 255, 0), "original cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original cloud");
	
	// 添加OBB包容盒
	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);  // 中心位置
	Eigen::Quaternionf quat(rotational_matrix_OBB);  // 旋转矩阵
	std::cout << "position_OBB: " << position_OBB << std::endl;
	std::cout << "mass_center: " << mass_center << std::endl;
	viewer.addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");  // x宽度、y高度、z深度
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "OBB"); // 设置为蓝色
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "OBB");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "OBB");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

	// 添加AABB包容盒
	viewer.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");  // 这应该是addCube的重载，添加的是正的立方体，所以不要旋转矩阵(1.0,1.0,0.0 这三个代表的是黄色)
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "AABB");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
	
	// 这步很关键，将所有的actor的可视化改为线框表示（这样上面的63、68行应该是一个作用）
	viewer.setRepresentationToWireframeForAllActors();


	pcl::PointXYZ center(mass_center(0), mass_center[1], mass_center(2));  // []就是重载的，一个意思
	pcl::PointXYZ x_axis(major_vector[0] + mass_center[0], major_vector[1] + mass_center(1), major_vector[2] + mass_center[2]);  
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center[0], middle_vector[1] + mass_center[1], middle_vector(2) + mass_center[2]);
	pcl::PointXYZ z_axis(minor_vector[0] + mass_center[0], minor_vector[1] + mass_center[1], minor_vector[2] + mass_center[2]);
	viewer.addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer.addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer.addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

	std::cout << "size of cloud: " << cloud->points.size() << std::endl;
	std::cout << "moment_of_inertia size: " << moment_of_inertia.size() << std::endl;
	std::cout << "eccentricity size: " << eccentricity.size() << std::endl;
	float height = max_point_OBB.z - min_point_OBB.z;
	float width = max_point_OBB.y - min_point_OBB.y;
	float depth = max_point_OBB.x - min_point_OBB.x;
	std::cout << "长：" << depth << std::endl;  // 可能这里的长宽高定义有些不一样。
	std::cout << "宽：" << width << std::endl;
	std::cout << "高：" << height << std::endl;

	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
	return 0;
}