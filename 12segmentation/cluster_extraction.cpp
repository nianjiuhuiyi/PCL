#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>  // �����˲�
#include <pcl/features/normal_3d.h>  // ���߹���
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>  // ����ŷʽ����
#include <pcl/console/time.h>


// ��������1��intensity���롣ǿ��������
bool enforceIntensitySimilarity(const pcl::PointXYZINormal &point_a, const pcl::PointXYZINormal &point_b, float squared_distance) {
	return (fabs(point_a.intensity - point_b.intensity) < 5.0f) ? true : false;
}

// ��������2��intensity���� & �������� ����ǿ��������
bool enforceCurvatureOrIntensitySimilarity(const pcl::PointXYZINormal &point_a, const pcl::PointXYZINormal &point_b, float squared_distance) {
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap();
	Eigen::Map<const Eigen::Vector3f> point_b_normal = point_b.getNormalVector3fMap();
	if (fabs(point_a.intensity - point_b.intensity) < 5.0f)
		return true;
	if (fabs(point_a_normal.dot(point_b_normal)) < 0.05)
		return true;
	return false;
}

// ��������3��
bool customRegionGrowing(const pcl::PointXYZINormal &point_a, const pcl::PointXYZINormal &point_b, float squared_distance) {
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap();
	Eigen::Map<const Eigen::Vector3f> point_b_normal = point_b.getNormalVector3fMap();
	if (squared_distance < 10000) {
		if (fabs(point_a.intensity - point_b.intensity) < 8.0f)  // ��a�͵�b���ܶȲ�
			return true;
		if (fabs(point_a_normal.dot(point_b_normal)) < 0.06)
			return true;
	}
	else {
		if (fabs(point_a.intensity - point_b.intensity) < 3.0f)
			return true;
	}
	return false;
}

// ����ŷʽ����
int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	// ��������
	// using IndicesClusters = std::vector<pcl::PointIndices>;
	// using IndicesClustersPtr = std::shared_ptr<std::vector<pcl::PointIndices> >;
	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);  // ��z������������using������
	pcl::IndicesClustersPtr small_clusters(new pcl::IndicesClusters);
	pcl::IndicesClustersPtr large_clusters(new pcl::IndicesClusters);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::console::TicToc tt;

	pcl::io::loadPCDFile("./Statues_4.pcd", *cloud);
	std::cout << "original: " << cloud->points.size() << " points data.\n" << std::endl;
	// �Ե��ƽ��������˲�
	std::cout << "DownSampleing..\n", tt.tic();
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(80, 80, 80);  // ����Ҷ�ߴ�
	vg.setDownsampleAllData(true);  // �����Ƿ�����е���������˲�
	vg.filter(*cloud_filtered);
	std::cout << ">> Done: " << tt.toc() << " ms," << cloud_filtered->points.size() << " points.\n" << std::endl;

	// ���з��߹���
	std::cout << "Computing normals...\n", tt.tic();
	pcl::copyPointCloud(*cloud_filtered, *cloud_normals);  // ���Ƶ��ƣ�������Ϊ���Ͳ�һ�������ƾͲ���ǰ���*cloud_normals = *cloud_filtered;
	// ������Ҫ���Ԥ����������cmakelist�� add_definitions(-DPCL_NO_PRECOMPILE)
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;  
	// pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::PointXYZINormal> ne;  // һ����˼��������OpenMP����
	ne.setInputCloud(cloud_filtered);
	ne.setSearchMethod(search_tree);
	ne.setRadiusSearch(300.0);  // ���������뾶
	ne.compute(*cloud_normals);
	std::cout << ">> Done: " << tt.toc() << " ms.\n"  << std::endl;

	// ��������ŷʽ�������
	std::cout << "Segmenting to clusters...\n", tt.tic();
	pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec(true);
	cec.setInputCloud(cloud_normals);
	// �����ò�ͬ�����������������������ֶ�����������֣�
	int method = 3;
	switch (method) {
	case 1:
		cec.setConditionFunction(enforceIntensitySimilarity);
		break;
	case 2:
		cec.setConditionFunction(enforceCurvatureOrIntensitySimilarity);
		break;
	case 3:
		cec.setConditionFunction(&customRegionGrowing);  // ���Ǻ���ָ�룬Ҫ��Ҫȡַ��&����
		break;
	default:
		cec.setConditionFunction(customRegionGrowing);  // ���� &����Ҳ���Զ�תΪ����ָ��
		break;
	}

	cec.setClusterTolerance(500);  //  ���þ���ο�������Ѿ���
	cec.setMinClusterSize(cloud_normals->points.size() / 1000);  // ���ù�С����ı�׼
	cec.setMaxClusterSize(cloud_normals->points.size() / 5);  // ���ù������ı�׼
	cec.segment(*clusters);  // ��ȡ����Ľ�����ָ��������ڵ���������������
	cec.getRemovedClusters(small_clusters, large_clusters);  // ��ȡ��Ч�ߴ�ľ���
	std::cout << ">> Done: " << tt.toc() << " ms\n" << std::endl;;

	// ʹ��ǿ��ͨ������������ӳٿ��ӻ�
	for (size_t i = 0; i < small_clusters->size(); ++i) {
		for (size_t j = 0; j < small_clusters->at(i).indices.size(); ++j)
			cloud_filtered->points[small_clusters->at(i).indices[j]].intensity = -2.0;
	}
	for (size_t i = 0; i < large_clusters->size(); ++i) {
		for (size_t j = 0; j < large_clusters->at(i).indices.size(); ++j)
			cloud_filtered->points[large_clusters->at(i).indices[j]].intensity = +10.0;
	}
	for (size_t i = 0; i < clusters->size(); ++i) {
		for (size_t j = 0; j < clusters->at(i).indices.size(); ++j) {
			cloud_filtered->points[clusters->at(i).indices[j]].intensity = rand() % 8;
		}
	}
	std::cout << "Saving...\n", tt.tic();
	std::stringstream ss;
	ss << "outputMethod_" << method << ".pcd";
	pcl::io::savePCDFile(ss.str(), *cloud_filtered);
	std::cout << ">> Done: " << tt.toc() << " ms.\n" << std::endl;
	system("pause");
	return 0;
}