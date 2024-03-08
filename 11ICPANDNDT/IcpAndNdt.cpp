#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/ndt.h>  // ndt��׼ͷ�ļ�
#include <pcl/filters/approximate_voxel_grid.h>>  // �˲�ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>

void ICP() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

	// �������������
	cloud_in->width = 5;
	cloud_in->height = 1;  // 1��������
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);
	for (auto &point : cloud_in->points) {
		point.x = 1 * rand() / (RAND_MAX + 1.0f);
		point.y = 1 * rand() / (RAND_MAX + 1.0f);
		point.z = 1 * rand() / (RAND_MAX + 1.0f);
	}

	// �����ɵĵ��Ƶ����ݸ��Ƹ��������
	*cloud_out = *cloud_in;
	// ִ�м򵥵ĸ��Ա仯��x��0.7�׵�ƫ��
	for (auto &point : cloud_out->points) point.x += 0.7f;


	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
		cloud_in->points[i].z << std::endl;
	std::cout << "\n";
	for (size_t i = 0; i < cloud_out->points.size(); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
	std::cout << "\n";

	// ����IterativeClosestPointʵ��
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);  // �������
	icp.setInputTarget(cloud_out);  // ƽ�ƺ�ĵ�����ΪĿ�����
	// ����һ�����������洢�任���Դ����
	// Ӧ��icp�㷨���ܹ����������ƣ��������������ƥ����ȷ�Ļ�(����������һ��Ӧ��ĳ�ָ���任���Ϳ��Եõ�������ͬһ����ϵ����ͬ�ĵ���)����ô icp.hasConverged() = 1��true��.Ȼ�����������ձ任�����ƥ������ͱ任�������Ϣ��
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);   // Final �����ӡ���������ݸ� cloud_out��һ���ģ������ط���һ�������������ȫƥ���ˣ�
	std::cout << "has converged: " << icp.hasConverged() << " socre: " << icp.getFitnessScore() << std::endl;
	const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
	std::cout << matrix << std::endl;

	for (auto &point : Final.points) {
		std::cout << "    " <<
			point.x << " " << point.y << " " << point.z << std::endl;
	}
}

int main(int argc, char* argv[]) {
	// �����״εķ���ɨ������ΪĿ�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("./room_scan1.pcd", *target_cloud) == -1) {
		std::cout << "Couldn`t read file room_scan1.pcd" << std::endl;
		return -1;
	}
	// ���ش���һ�����ӽǵĵ�����Ϊ�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("./room_scan2.pcd", *input_cloud) == -1) {
		std::cout << "Couldn`t read file room_scan2.pcd" << std::endl;
		return -1;
	}
	std::cout << "points of room_scan1.pcd: " << target_cloud->size() << std::endl;
	std::cout << "points of room_scan2.pcd: " << input_cloud->size() << std::endl;
	// ������ǿ�ʼ���[Դ����input_cloud]��[Ŀ�����target_cloud]�ı任����


	// Filtering input scan to roughly 10% of original size to increase speed of registration.
	// �ⲽ���ǽ������Դ���ƹ��˵�Լ10%��ԭʼ��С���������׼�ٶȡ��������κ��������ȹ����������ԣ���ע��Ŀ�����target_cloud����Ҫ�����˲�������ΪNDT�㷨��Ŀ����ƶ�Ӧ������Voxel�������ݼ���ʱ����ʹ�õ����㣬����ʹ�����صĵ�(�������˽���������)��
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filter_cloud);
	std::cout << "Filtered cloud contains " << filter_cloud->size() << " points from room_scan2.pcd\n";

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.01);  // ������ֹ��������С�任����
	ndt.setStepSize(0.1);  // ���� More-Thuente ����������󲽳�
	ndt.setResolution(1.0);  // ����NDT����ṹ�ķֱ��� (VoxelGridCovariance).
	// ����3�в��������ÿ�����ע��һ:
	ndt.setMaximumIterations(35);  // ���� ƥ����� ������������ע�Ͷ���
	ndt.setInputSource(filter_cloud);  // Setting point cloud to be aligned.
	ndt.setInputTarget(target_cloud);  // Setting point cloud to be aligned to.

	// ����ʹ�û����˲�෨�õ��Ĵ��Գ�ʼ�任���󣨼�ע������
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	// ��������ĸ��Ա任��ʹ��������Ŀ���ƶ��롣����ע���ģ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	ndt.align(*output_cloud, init_guess);
	std::cout << "Normal Distributions Transform has converged: " << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << std::endl;
	
	// ʹ���ҵ��ı任��������δ���˵������ƽ��б任������ndt�еĵõ�����任����
	pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
	// ����ת����ĵ���
	pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

	// ���ӻ�
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	// Ŀ����ƣ�������1��red��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color_handler(target_cloud, 255, 0, 0);
	viewer.addPointCloud(target_cloud, target_color_handler, "target cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	// ����ת����ĵ��ƣ�green��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color_handler(output_cloud, 0, 255, 0);
	viewer.addPointCloud(output_cloud, output_color_handler, "output cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");
	
	viewer.addCoordinateSystem(1.0, "global");
	viewer.initCameraParameters();
	while (!viewer.wasStopped()) {
		//viewer.spinOnce(100);
		//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		viewer.spinOnce();
	}
	return 0;
}