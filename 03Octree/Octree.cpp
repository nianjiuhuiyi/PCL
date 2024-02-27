#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_cloud.h>  // 点云头文件
#include <pcl/octree/octree.h>  // 八叉树头文件
#include <pcl/visualization/cloud_viewer.h>  // 可视化头文件

#define PRINT_POINT_DISTANCE( point, distance ) \
	std::cout << "(" << point.x  << ", "<< point.y << ", " << point.z << ")." \
		<< " squared distance: " << distance << std::endl

#define PRINT_POINT( point ) \
	std::cout << "(" << point.x  << ", "<< point.y << ", " << point.z << ")" << std::endl


int main(int argc, char* argv[]) {
	std::srand((unsigned int)time(NULL));  // 用系统时间初始化随机种子
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 创建点云数据
	cloud->width = 1000;
	cloud->height = 1;  // 无序
	cloud->points.resize(cloud->height * cloud->width);
	// 随机循环产生点云的坐标值(产生0-1023的随机值，上面有写到过，用的是for循环)
	// 它本质就是一个STL容器，可以用pcl::PointCloud<pcl::PointXYZ>::iterator iter = cloud->begin()进行循环
	for(auto &point : cloud->points) {
		point.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		point.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		point.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	/*
	创建一个octree实例，用设置分辨率进行初始化，该octree用它的页结点存放点索引向量，
	分辨率参数描述最低一级octree的最小体素的尺寸，因此octree的深度是分辨率和点云空间维度的函数，
	如果知道点云的边界框，应该用defineBoundingbox方法把它分配给octree然后通过点云指针把所有点增加到octree中。
	*/
	// 该参数描述了octree叶子leaf节点的最小体素尺寸
	float resolution = 128.0;  // 设置分辨率为128
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	// 设置输入点云
	octree.setInputCloud(cloud);  // 这两句是最关键的建立PointCloud和octree之间的联系
	octree.addPointsFromInputCloud();  // 构建octree

	pcl::PointXYZ searchPoint;  // 设置搜索点
	searchPoint.x = 1024.f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.f * rand() / (RAND_MAX + 1.0f);
	//pcl::PointXYZ searchPoint = cloud->points.at(10);

	/* Neighbors within voxel search
	方式一：“体素近邻搜索”，它把查询点所在的体素中其它点的索引作为查询结果返回，
		结果以点索引向量的形式保存，因此搜索点和搜索结果之间的距离取决于八叉树的分辨率参数
	*/
	std::vector<int> pointIdxVec;
	if (octree.voxelSearch(searchPoint, pointIdxVec)) {
		std::cout << "Neighbors within voxel search at ";
		PRINT_POINT(searchPoint);
		std::vector<int>::const_iterator iter = pointIdxVec.cbegin();
		for (; iter != pointIdxVec.cend(); ++iter) {
			std::cout << "  ";
			PRINT_POINT(cloud->points.at(*iter));
		}
	}
	std::cout << pointIdxVec.size() << std::endl;

	/*  K nearest neighbor search
	方式二：K近邻搜索，本例中k设为10，这会把搜索结果写到两个分开的向量中
	*/
	int K = 10;
	std::vector<int> pointIdxNKNSearch;  // 包含搜索结果，即结果点的索引的向量
	std::vector<float> pointNKNSquareDistance;  // 保存相应的搜索点和近邻之间的距离平方

	if (octree.nearestKSearch(searchPoint,K, pointIdxNKNSearch, pointNKNSquareDistance) > 0) {
		std::cout << "\nK nearest neighbor search at ";
		PRINT_POINT(searchPoint);
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
			PRINT_POINT_DISTANCE(cloud->points.at(pointIdxNKNSearch[i]), pointNKNSquareDistance.at(i));
		}
	}

	/* Neighbors within radius search
	方式三：半径内近邻搜索，和方式二类似
	*/
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquareDistance;

	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) > 0) {
		std::cout << "\nNeighbors within radius search at ";
		PRINT_POINT(searchPoint);
		std::cout << "radius: " << radius; 
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
			PRINT_POINT_DISTANCE(cloud->points.at(pointIdxRadiusSearch.at(i)), pointRadiusSquareDistance[i]);
		}
	}

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

	pcl::PointXYZ originPoint(0.0, 0.0, 0.0);
	viewer.addLine(originPoint, searchPoint);  // 添加原点到搜索点的线
	viewer.addSphere(searchPoint, radius, "sphere", 0);  // 添加一个球
	viewer.addCoordinateSystem(200);  // 添加一个放到200倍后的坐标系

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	return 0;
}