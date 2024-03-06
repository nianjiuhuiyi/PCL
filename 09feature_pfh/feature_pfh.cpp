#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>


int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("./rabbit.pcd", *cloud);

	// 计算法向量
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);  // 估算法向量半径3cm

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	normalEstimation.setSearchMethod(kd_tree);
	normalEstimation.compute(*normals);

	// 计算 PFH 直方图
	// 如果传入的cloud类型是 PointNormal，可以直接 pfh.setInputNormals(cloud)。上面法线估计有说这俩区别
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(cloud);
	pfh.setInputNormals(normals);
	// 创建一个空的kdtree给PFH对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pfh.setSearchMethod(tree);
	// 设置搜索领域（重点：这个半径一定要要比上面的法线估计的索引半径要大）
	pfh.setRadiusSearch(0.08);
	// 用于存输出数据
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh.compute(*pfhs);

	for (unsigned long i = 0; i < pfhs->points.size(); ++i) {
		pcl::PFHSignature125 &singature125 = pfhs->points[i];
		float *hist = singature125.histogram;
		printf("%d: %f,%f,%f\n", i, hist[1], hist[2], hist[3]);
	}

	pcl::visualization::PCLVisualizer viewer("Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.01, "normals");
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	return 0;
}