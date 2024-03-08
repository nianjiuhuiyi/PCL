#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/ndt.h>  // ndt配准头文件
#include <pcl/filters/approximate_voxel_grid.h>>  // 滤波头文件
#include <pcl/visualization/pcl_visualizer.h>

void ICP() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

	// 随机填充无序点云
	cloud_in->width = 5;
	cloud_in->height = 1;  // 1代表无序
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);
	for (auto &point : cloud_in->points) {
		point.x = 1 * rand() / (RAND_MAX + 1.0f);
		point.y = 1 * rand() / (RAND_MAX + 1.0f);
		point.z = 1 * rand() / (RAND_MAX + 1.0f);
	}

	// 把生成的点云的数据复制给输出点云
	*cloud_out = *cloud_in;
	// 执行简单的刚性变化，x加0.7米的偏移
	for (auto &point : cloud_out->points) point.x += 0.7f;


	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
		cloud_in->points[i].z << std::endl;
	std::cout << "\n";
	for (size_t i = 0; i < cloud_out->points.size(); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
	std::cout << "\n";

	// 创建IterativeClosestPoint实例
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);  // 输入点云
	icp.setInputTarget(cloud_out);  // 平移后的点云作为目标点云
	// 创建一个对象用来存储变换后的源点云
	// 应用icp算法后，能够保存结果点云，如果这两个点云匹配正确的话(即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云)，那么 icp.hasConverged() = 1（true）.然后可以输出最终变换矩阵的匹配分数和变换矩阵等信息，
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);   // Final 这里打印出来的数据跟 cloud_out是一样的（其它地方不一样，这里可能完全匹配了）
	std::cout << "has converged: " << icp.hasConverged() << " socre: " << icp.getFitnessScore() << std::endl;
	const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
	std::cout << matrix << std::endl;

	for (auto &point : Final.points) {
		std::cout << "    " <<
			point.x << " " << point.y << " " << point.z << std::endl;
	}
}

int main(int argc, char* argv[]) {
	// 加载首次的房间扫描数据为目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("./room_scan1.pcd", *target_cloud) == -1) {
		std::cout << "Couldn`t read file room_scan1.pcd" << std::endl;
		return -1;
	}
	// 加载从另一个新视角的点云作为输入点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("./room_scan2.pcd", *input_cloud) == -1) {
		std::cout << "Couldn`t read file room_scan2.pcd" << std::endl;
		return -1;
	}
	std::cout << "points of room_scan1.pcd: " << target_cloud->size() << std::endl;
	std::cout << "points of room_scan2.pcd: " << input_cloud->size() << std::endl;
	// 下面就是开始求从[源点云input_cloud]到[目标点云target_cloud]的变换矩阵


	// Filtering input scan to roughly 10% of original size to increase speed of registration.
	// 这步就是将输入的源点云过滤到约10%的原始大小，以提高配准速度。这里用任何其它均匀过滤器都可以，但注意目标点云target_cloud不需要进行滤波处理，因为NDT算法在目标点云对应的体素Voxel网格数据计算时，不使用单个点，而是使用体素的点(即已做了降采样处理)。
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filter_cloud);
	std::cout << "Filtered cloud contains " << filter_cloud->size() << " points from room_scan2.pcd\n";

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.01);  // 设置终止条件的最小变换差异
	ndt.setStepSize(0.1);  // 设置 More-Thuente 线搜索的最大步长
	ndt.setResolution(1.0);  // 设置NDT网格结构的分辨率 (VoxelGridCovariance).
	// 上面3行参数的设置看具体注释一:
	ndt.setMaximumIterations(35);  // 设置 匹配迭代 的最大次数（见注释二）
	ndt.setInputSource(filter_cloud);  // Setting point cloud to be aligned.
	ndt.setInputTarget(target_cloud);  // Setting point cloud to be aligned to.

	// 设置使用机器人测距法得到的粗略初始变换矩阵（见注释三）
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	// 计算所需的刚性变换以使输入云与目标云对齐。（见注释四）
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	ndt.align(*output_cloud, init_guess);
	std::cout << "Normal Distributions Transform has converged: " << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << std::endl;
	
	// 使用找到的变换矩阵，来对未过滤的输入云进行变换。（从ndt中的得到这个变换矩阵）
	pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
	// 保存转换后的点云
	pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

	// 可视化
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	// 目标点云，（场景1，red）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color_handler(target_cloud, 255, 0, 0);
	viewer.addPointCloud(target_cloud, target_color_handler, "target cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	// 输入转换后的点云（green）
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