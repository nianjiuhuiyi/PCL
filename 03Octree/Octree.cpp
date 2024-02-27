#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_cloud.h>  // ����ͷ�ļ�
#include <pcl/octree/octree.h>  // �˲���ͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>  // ���ӻ�ͷ�ļ�

#define PRINT_POINT_DISTANCE( point, distance ) \
	std::cout << "(" << point.x  << ", "<< point.y << ", " << point.z << ")." \
		<< " squared distance: " << distance << std::endl

#define PRINT_POINT( point ) \
	std::cout << "(" << point.x  << ", "<< point.y << ", " << point.z << ")" << std::endl


int main(int argc, char* argv[]) {
	std::srand((unsigned int)time(NULL));  // ��ϵͳʱ���ʼ���������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// ������������
	cloud->width = 1000;
	cloud->height = 1;  // ����
	cloud->points.resize(cloud->height * cloud->width);
	// ���ѭ���������Ƶ�����ֵ(����0-1023�����ֵ��������д�������õ���forѭ��)
	// �����ʾ���һ��STL������������pcl::PointCloud<pcl::PointXYZ>::iterator iter = cloud->begin()����ѭ��
	for(auto &point : cloud->points) {
		point.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		point.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		point.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	/*
	����һ��octreeʵ���������÷ֱ��ʽ��г�ʼ������octree������ҳ����ŵ�����������
	�ֱ��ʲ����������һ��octree����С���صĳߴ磬���octree������Ƿֱ��ʺ͵��ƿռ�ά�ȵĺ�����
	���֪�����Ƶı߽��Ӧ����defineBoundingbox�������������octreeȻ��ͨ������ָ������е����ӵ�octree�С�
	*/
	// �ò���������octreeҶ��leaf�ڵ����С���سߴ�
	float resolution = 128.0;  // ���÷ֱ���Ϊ128
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	// �����������
	octree.setInputCloud(cloud);  // ����������ؼ��Ľ���PointCloud��octree֮�����ϵ
	octree.addPointsFromInputCloud();  // ����octree

	pcl::PointXYZ searchPoint;  // ����������
	searchPoint.x = 1024.f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.f * rand() / (RAND_MAX + 1.0f);
	//pcl::PointXYZ searchPoint = cloud->points.at(10);

	/* Neighbors within voxel search
	��ʽһ�������ؽ��������������Ѳ�ѯ�����ڵ��������������������Ϊ��ѯ������أ�
		����Ե�������������ʽ���棬�����������������֮��ľ���ȡ���ڰ˲����ķֱ��ʲ���
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
	��ʽ����K����������������k��Ϊ10�������������д�������ֿ���������
	*/
	int K = 10;
	std::vector<int> pointIdxNKNSearch;  // �������������������������������
	std::vector<float> pointNKNSquareDistance;  // ������Ӧ��������ͽ���֮��ľ���ƽ��

	if (octree.nearestKSearch(searchPoint,K, pointIdxNKNSearch, pointNKNSquareDistance) > 0) {
		std::cout << "\nK nearest neighbor search at ";
		PRINT_POINT(searchPoint);
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
			PRINT_POINT_DISTANCE(cloud->points.at(pointIdxNKNSearch[i]), pointNKNSquareDistance.at(i));
		}
	}

	/* Neighbors within radius search
	��ʽ�����뾶�ڽ����������ͷ�ʽ������
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
	viewer.addLine(originPoint, searchPoint);  // ���ԭ�㵽���������
	viewer.addSphere(searchPoint, radius, "sphere", 0);  // ���һ����
	viewer.addCoordinateSystem(200);  // ���һ���ŵ�200���������ϵ

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	return 0;
}