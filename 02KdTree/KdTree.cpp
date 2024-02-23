#include <iostream>
#include <vector>
#include <ctime>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char **argv) {
	// ��ϵͳʱ���ʼ���������
	std::srand((unsigned int)time(NULL));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// ����1000����������
	cloud->width = 1000;
	cloud->height = 1;     // heightΪ1����������ƣ�ǰ�����ݸ�ʽ���ܹ�
	cloud->points.resize(cloud->width * cloud->height);

	// ������������� 0 - 1023 (Ҳ�ǿ��Զ�ȡһ��pcd�ļ���Ȼ����ʹ�õİ�)
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
	
	// ����KdTree��ʵ����KdTreeFLANN
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//pcl::search::KdTree<pcl::PointXYZ> kdtree;  // һ����˼����Ҫ<pcl/search/kdtree.h>ͷ�ļ�������Ҳ��Ҫ<pcl/search/impl/search.hpp>
	// ���������ռ䣬��cloud��Ϊ����
	kdtree.setInputCloud(cloud);

	// ��ʼ��һ������ĵ㣬��Ϊ��ѯ��
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	// ��ʽһ������K������ھ�
	int K = 10;  // ��ʾ����10���ٽ���
	std::vector<int> pointIdxNKNSearch(K);  // �������������ٽ��������
	std::vector<float> pointNKNSquareDistance(K);  // �����Ӧ�ٽ���ľ����ƽ��
	// ���ش�����0�����ڣ��ͽ����ӡ����
	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquareDistance) > 0) {
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
			int idx = pointIdxNKNSearch.at(i);
			auto &points = cloud->points;
			std::cout << "  " << points.at(idx).x << " " << points[idx].y << " " << points[idx].z << " (����ƽ��: " << pointNKNSquareDistance.at(i) << ")" << std::endl;
		}
	}
	std::cout << "\nhello\n" << std::endl;

	// ��ʽ����ͨ��ָ���뾶����
	std::vector<int> pointIxdRadiusSearch;
	std::vector<float> pointRadiusSquareDistance;

	float radius = 256.0f;  // �뾶��С�󣬿����Ҳ����ġ�
	// ����ķ���ֵ��һ�����ͣ��ʹ����ҵ��ĸ�����
	if (kdtree.radiusSearch(searchPoint, radius, pointIxdRadiusSearch, pointRadiusSquareDistance) > 0) {

		for (size_t i = 0; i < pointIxdRadiusSearch.size(); ++i) {
			int idx = pointIxdRadiusSearch.at(i);
			auto &points = cloud->points;
			std::cout << "  " << points[idx].x << " " << points[idx].y << " " << points[idx].z << " (����ƽ��: " << pointRadiusSquareDistance[i] << std::endl;
		}
	}

	pcl::visualization::PCLVisualizer viewer("PCLViewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

	pcl::PointXYZ originalPoint(0.0, 0.0, 0.0);
	// ��Ӵ�ԭ�㵽��������߶Σ�������ָ����ɫ�����Բ�Ҫ��
	viewer.addLine(originalPoint, searchPoint, 0.9, 0.9, 0.0);
	// ���һ����������ΪԲ�ģ������뾶Ϊ�뾶������ (��ɫ����Ҳ���Բ�Ҫ)
	viewer.addSphere(searchPoint, radius, 0.0, 0.5, 0.0, "sphere", 0);
	// ���һ���Ŵ�200���������ϵ
	viewer.addCoordinateSystem(200);


	// ��q�˳���������Ҳ��ͬ��  viewer.spin();
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	return 0;
}