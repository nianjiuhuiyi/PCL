#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  // 这个头文件
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>  // pcl::VoxelGrid 类需要
#include <pcl/filters/uniform_sampling.h>  // pcl::UniformSampling 类需要
#include <pcl/filters/statistical_outlier_removal.h>  // pcl::StatisticalOutlierRemoval 类需要

#define PRINT_POINT( point ) \
	std::cout << "(" << point.x  << ", "<< point.y << ", " << point.z << ")" << std::endl

void pass_through() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (auto &point : cloud->points) {
		point.x = 2 * rand() / (RAND_MAX + 1.0f);
		point.y = 2 * rand() / (RAND_MAX + 1.0f);
		point.z = 2 * rand() / (RAND_MAX + 1.0f);
	}

	std::cout << "filtering before filtering: " << std::endl;
	for (auto &point : cloud->points) {
		PRINT_POINT(point);
	}

	// 创建一个过滤的对象
	pcl::PassThrough<pcl::PointXYZ> passThrough;
	passThrough.setInputCloud(cloud);  // 1.设置输入源
	passThrough.setFilterFieldName("z");  // 2.设置过滤时所需要点云类型的Z字段
	passThrough.setFilterLimits(0.0, 1.0);  // 3.设置过滤范围(在这个范围内的保留)
	passThrough.filter(*cloud_filtered);  // 4.执行过滤，结果输出到cloud_filtered
	std::cout << "\nCloud after filtering" << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
		PRINT_POINT(cloud_filtered->points.at(i));
	}

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	// 这里会一直阻塞直到点云被渲染
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {}
}

void voxel_grid() {
	// 这里用pcl::PointCloud<pcl::PointXYZ>::Ptr 这个类型应该也是可以的
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// 从文件读取点云图
	pcl::PCDReader reader;
	reader.read("C:\\Users\\Administrator\\Downloads\\table_scene_lms400.pcd", *cloud);
	std::cout << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

	// 创建一个长宽高分别是1cm的体素过滤器，cloud作为输入数据，cloud_filtered作为输出数据
	float leftSize = 0.01f;  // 代表1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leftSize, leftSize, leftSize);
	sor.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;;

	pcl::PCDWriter writer;
	writer.write("./table_scene_lms400_downsampled.pcd", *cloud_filtered);
}

void uniform_sampleing() {
	// 读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("C:\\Users\\Administrator\\Downloads\\table_scene_lms400.pcd", *cloud);
	std::cout << "original cloud size: " << cloud->size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_uniform(new pcl::PointCloud<pcl::PointXYZ>());
	// 使用unifromSampling下采样
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	double radius = 0.005f;
	uniform_sampling.setRadiusSearch(radius);
	uniform_sampling.filter(*cloud_uniform);
	std::cout << "UniformSampling size: " << cloud_uniform->size() << std::endl;
}


int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("C:\\Users\\Administrator\\Downloads\\table_scene_lms400.pcd", *cloud);
	std::cout << "Cloud before filtering: " << std::endl;
	std::cout << *cloud << std::endl;

	/*
	创建过滤器，每个点分析计算时考虑最近邻居个数为50个；
	设置标准差阈值为1，这意味着所有距离查询点的平均距离的标准偏差均大于1个标准偏差的所有点都将被标记为离群值并删除
	*/
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);  // 设置平均距离估计的最近邻居的数量k
	sor.setStddevMulThresh(1.0);  // 设置标准差阈值系数
	sor.filter(*cloud_filtered);  // 过滤并进结果进行保存
	std::cout << "Cloud after filtering: " << std::endl;
	std::cout << *cloud_filtered << std::endl;

	pcl::PCDWriter writer;
	// 这是将保留下来的点存为 _inliers.pcd 
	writer.write<pcl::PointXYZ>("./table_scene_lms400_inliers.pcd", *cloud_filtered);

	// 使用的是相通的过滤器，但是对输出结果取反，得到滤去的那些点，存为 _outliers.pcd
	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("./table_scene_lms400_outliers.pcd", *cloud_filtered);

	system("pause");
	return 0;
}