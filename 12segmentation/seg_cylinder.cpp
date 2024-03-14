#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>  // pcl::ModelCoefficients
#include <pcl/segmentation/sac_segmentation.h>  // pcl::SACSegmentation、pcl::SACSegmentationFromNormals

#include <pcl/filters/passthrough.h>  // pcl::PassThrough
#include <pcl/features/normal_3d.h>  // pcl::NormalEstimation, 带_omp的可以用OpenMP
#include <pcl/filters/extract_indices.h>  // pcl::ExtractIndices


int main(int argc, char* argv[]) {
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PassThrough<pcl::PointXYZ> pass;  // 直通滤波器
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;  // 法线估计对象
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;  // 分割器
	pcl::ExtractIndices<pcl::PointXYZ> extract;  // 点提取对象（即从输入点云中提取符合要求的点）
	pcl::ExtractIndices<pcl::Normal> extract_normals;  // 法线提取对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

	reader.read("./table_scene_mug_stereo_textured.pcd", *cloud);
	std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	// 创建一个 passthrough 过滤器去掉 spurious NaNs
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");  // 必须是小写的
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points" << std::endl;

	// 法线估计
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// 创建分割对象，为了 planar model 设置全部的参数
	seg.setOptimizeCoefficients(true);  // 可选配置，是否优化模型系数
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers_plane, *coefficients_plane);  // 得到 plane inliers 和 coefficients
	std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;

	// 从输入云提取平面的inliers
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);  // 如果为true，就是保留要过滤的点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	extract.filter(*cloud_plane);
	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points" << std::endl;
	if (!cloud_plane->points.empty()) 
		writer.write("./table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

	/* ------------------------------------------------------------------- */
	
	// 去掉 planar inliers，提取之外的部分
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);  // 全部法线
	extract_normals.setIndices(inliers_plane);  // 这里给的索引是平面的法线索引，所以前面才会有extract_normals.setNegative(true);来取反，这样cloud_normals2就是除平面外的所有法线
	extract_normals.filter(*cloud_normals2);

	// 设置圆柱体分割对象参数
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);  // 设置分割模型为圆柱体
	seg.setMethodType(pcl::SAC_RANSAC);  // 采用RANSAC算法进行参数估计
	seg.setNormalDistanceWeight(0.1);  // 表面法线权重系数
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);  // 内点到模型的最大距离 0.05m
	seg.setRadiusLimits(0, 0.1);  // 圆柱半径范围为0 -> 0.1m
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
	// 写进硬盘
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else {
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
		writer.write("./table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder);
	}
	system("pause");
	return 0;
}