#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>  // pcl::ModelCoefficients
#include <pcl/segmentation/sac_segmentation.h>  // pcl::SACSegmentation

int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 生成15个无序点云，x、y为随机数，z为1.0
	// 将points中0、3、6索引位置的z值进行修改，将其作为离群值
	cloud->width = 15;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1.0;
	}
	cloud->points[0].z = 2.0; cloud->points[3].z = -2.0; cloud->points[6].z = 4.0;
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;
	for (std::size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;

	/*
	- 创建分割时所需要的模型稀疏对象 coefficients 及存储内点的点索引集合对象 inliers。
	- 指定“阈值距离DistanceThreshold”的地方，该距离阈值确定点必须与模型有多远才能被是视为离群点。
	- 这里距离阈值是0.01m，即只要点到z=1平面距离小于该阈值的带你都作为内部点看待，而大于该阈值的则看作离群点。
	- 将使用RANSAC方法(pcl::SAC_RANSAC)作为可靠的估计器，
	*/
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;  // 分割对象
	seg.setOptimizeCoefficients(true);  // 可选配置，是否优化模型系数
	// 必选配置：设置分割的模型类型、分割算法、距离阈值、输入点云
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud);
	// 执行分割操作，并存储分割结果保存到点集合 inliers 即存储平明模型稀疏 coefficients 
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return -1;
	}
	// 用来打印出估算的平面模型的参数（以 ax+by+ca+d=0 形式）,详见RANSAC采样一致性算法的SACMODEL_PLANE平面模型
	auto &values = coefficients->values;
	std::cout << "Model Coefficients: " << values[0] << " " << values[1] << " " << values[2] << " " << values[3] << std::endl;

	std::cout << "Model inliers: " << inliers->indices.size() << std::endl;
	for (std::size_t i = 0; i < inliers->indices.size(); ++i)
		std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
		<< cloud->points[inliers->indices[i]].y << " "
		<< cloud->points[inliers->indices[i]].z << std::endl;

	system("pause");
	return 0;
}