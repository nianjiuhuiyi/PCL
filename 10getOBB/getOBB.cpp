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
	feature_extractor.compute();  // ֱ�ӿ�ʼ�������㣬���ô�����ȥ���ս��

	std::vector<float> moment_of_inertia;  // ��Ź��Ծص���������
	std::vector<float> eccentricity;  // ���ƫ���ʵ���������
	float major_value, middle_value, minor_value;  // ��������ֵ
	Eigen::Vector3f major_vector, middle_vector, minor_vector;  // ������������
	Eigen::Vector3f mass_center;  // ����

	// ���ĸ���OBB��Ӧ�Ĳ���
	pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	// ����AABB��Ӧ�Ĳ���
	pcl::PointXYZ min_point_AABB, max_point_AABB;

	feature_extractor.getMomentOfInertia(moment_of_inertia);  // �õ����Ծ�
	feature_extractor.getEccentricity(eccentricity);  // �õ�ƫ����

	// ��ȡOBB����
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	// ��ȡAABB����
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);

	// ��ȡ����major_vector������middle_vector��������minor_vector
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);  // ��ȡ����

	// ���ӻ����Ȱѳ�ʼ������ӽ�ȥ��
	pcl::visualization::PCLVisualizer viewer("viewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	viewer.addPointCloud<pcl::PointXYZ>(cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 255, 0), "original cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original cloud");
	
	// ���OBB���ݺ�
	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);  // ����λ��
	Eigen::Quaternionf quat(rotational_matrix_OBB);  // ��ת����
	std::cout << "position_OBB: " << position_OBB << std::endl;
	std::cout << "mass_center: " << mass_center << std::endl;
	viewer.addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");  // x��ȡ�y�߶ȡ�z���
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "OBB"); // ����Ϊ��ɫ
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "OBB");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "OBB");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

	// ���AABB���ݺ�
	viewer.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");  // ��Ӧ����addCube�����أ���ӵ������������壬���Բ�Ҫ��ת����(1.0,1.0,0.0 ������������ǻ�ɫ)
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "AABB");
	//viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
	
	// �ⲽ�ܹؼ��������е�actor�Ŀ��ӻ���Ϊ�߿��ʾ�����������63��68��Ӧ����һ�����ã�
	viewer.setRepresentationToWireframeForAllActors();


	pcl::PointXYZ center(mass_center(0), mass_center[1], mass_center(2));  // []�������صģ�һ����˼
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
	std::cout << "����" << depth << std::endl;  // ��������ĳ���߶�����Щ��һ����
	std::cout << "��" << width << std::endl;
	std::cout << "�ߣ�" << height << std::endl;

	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
	return 0;
}