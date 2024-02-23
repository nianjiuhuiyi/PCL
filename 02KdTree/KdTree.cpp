#include <iostream>
#include <vector>
#include <ctime>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char **argv) {
	// 用系统时间初始化随机种子
	std::srand((unsigned int)time(NULL));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// 生成1000个点云数据
	cloud->width = 1000;
	cloud->height = 1;     // height为1代表无序点云，前面数据格式介绍过
	cloud->points.resize(cloud->width * cloud->height);

	// 给点云填充数据 0 - 1023 (也是可以读取一个pcd文件，然后来使用的吧)
	std::uniform_int_distribution<unsigned> u(0, 102300);
	std::default_random_engine e;
	for (auto &point : cloud->points) {
		//point.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		point.x = static_cast<float>(u(e)) / 100.0f;
		point.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		point.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	//for (auto &point : cloud->points) {
	//	std::cout << point.x << "  " << point.y << "  " << point.z << std::endl;
	//}
	
	// 创建KdTree的实现类KdTreeFLANN
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//pcl::search::KdTree<pcl::PointXYZ> kdtree;  // 一个意思，需要<pcl/search/kdtree.h>头文件，或许也还要<pcl/search/impl/search.hpp>
	// 设置搜索空间，把cloud作为输入
	kdtree.setInputCloud(cloud);

	// 初始化一个随机的点，作为查询点
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	// 方式一：搜索K个最近邻居
	int K = 10;  // 表示搜索10个临近点
	std::vector<int> pointIdxNKNSearch(K);  // 保存搜索到的临近点的索引
	std::vector<float> pointNKNSquareDistance(K);  // 保存对应临近点的距离的平方
	// 返回大于了0个近邻，就将其打印出来
	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquareDistance) > 0) {
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
			int idx = pointIdxNKNSearch.at(i);
			auto &points = cloud->points;
			std::cout << "  " << points.at(idx).x << " " << points[idx].y << " " << points[idx].z << " (距离平方: " << pointNKNSquareDistance.at(i) << ")" << std::endl;
		}
	}
	std::cout << "\nhello\n" << std::endl;

	// 方式二：通过指定半径搜索
	std::vector<int> pointIxdRadiusSearch;
	std::vector<float> pointRadiusSquareDistance;

	float radius = 256.0f;  // 半径过小后，可能找不到的。
	// 这里的返回值是一个整型，就代表找到的个数。
	if (kdtree.radiusSearch(searchPoint, radius, pointIxdRadiusSearch, pointRadiusSquareDistance) > 0) {

		for (size_t i = 0; i < pointIxdRadiusSearch.size(); ++i) {
			int idx = pointIxdRadiusSearch.at(i);
			auto &points = cloud->points;
			std::cout << "  " << points[idx].x << " " << points[idx].y << " " << points[idx].z << " (距离平方: " << pointRadiusSquareDistance[i] << std::endl;
		}
	}

	pcl::visualization::PCLVisualizer viewer("PCLViewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

	pcl::PointXYZ originalPoint(0.0, 0.0, 0.0);
	// 添加从原点到搜索点的线段（后面是指定颜色，可以不要）
	viewer.addLine(originalPoint, searchPoint, 0.9, 0.9, 0.0);
	// 添加一个以搜索点为圆心，搜索半径为半径的球体 (颜色参数也可以不要)
	viewer.addSphere(searchPoint, radius, 0.0, 0.5, 0.0, "sphere", 0);
	// 添加一个放大200倍后的坐标系
	viewer.addCoordinateSystem(200);


	// 按q退出，这三行也等同于  viewer.spin();
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	return 0;
}