#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>  // 体素滤波
#include <pcl/features/normal_3d.h>  // 法线估计
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>  // 条件欧式聚类
#include <pcl/console/time.h>


// 条件函数1：intensity距离。强度相似性
bool enforceIntensitySimilarity(const pcl::PointXYZINormal &point_a, const pcl::PointXYZINormal &point_b, float squared_distance) {
	return (fabs(point_a.intensity - point_b.intensity) < 5.0f) ? true : false;
}

// 条件函数2：intensity聚类 & 法向量。 曲率强度相似性
bool enforceCurvatureOrIntensitySimilarity(const pcl::PointXYZINormal &point_a, const pcl::PointXYZINormal &point_b, float squared_distance) {
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap();
	Eigen::Map<const Eigen::Vector3f> point_b_normal = point_b.getNormalVector3fMap();
	if (fabs(point_a.intensity - point_b.intensity) < 5.0f)
		return true;
	if (fabs(point_a_normal.dot(point_b_normal)) < 0.05)
		return true;
	return false;
}

// 条件函数3：
bool customRegionGrowing(const pcl::PointXYZINormal &point_a, const pcl::PointXYZINormal &point_b, float squared_distance) {
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap();
	Eigen::Map<const Eigen::Vector3f> point_b_normal = point_b.getNormalVector3fMap();
	if (squared_distance < 10000) {
		if (fabs(point_a.intensity - point_b.intensity) < 8.0f)  // 点a和点b的密度差
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

// 条件欧式聚类
int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	// 创建索引
	// using IndicesClusters = std::vector<pcl::PointIndices>;
	// using IndicesClustersPtr = std::shared_ptr<std::vector<pcl::PointIndices> >;
	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);  // 这倆类名是类里用using得来的
	pcl::IndicesClustersPtr small_clusters(new pcl::IndicesClusters);
	pcl::IndicesClustersPtr large_clusters(new pcl::IndicesClusters);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::console::TicToc tt;

	pcl::io::loadPCDFile("./Statues_4.pcd", *cloud);
	std::cout << "original: " << cloud->points.size() << " points data.\n" << std::endl;
	// 对点云进行体素滤波
	std::cout << "DownSampleing..\n", tt.tic();
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(80, 80, 80);  // 设置叶尺寸
	vg.setDownsampleAllData(true);  // 设置是否对所有点进行体素滤波
	vg.filter(*cloud_filtered);
	std::cout << ">> Done: " << tt.toc() << " ms," << cloud_filtered->points.size() << " points.\n" << std::endl;

	// 进行法线估计
	std::cout << "Computing normals...\n", tt.tic();
	pcl::copyPointCloud(*cloud_filtered, *cloud_normals);  // 复制点云，可能因为类型不一样，复制就不是前面的*cloud_normals = *cloud_filtered;
	// 这里需要添加预处理器：在cmakelist中 add_definitions(-DPCL_NO_PRECOMPILE)
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;  
	// pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::PointXYZINormal> ne;  // 一个意思，这是用OpenMP加速
	ne.setInputCloud(cloud_filtered);
	ne.setSearchMethod(search_tree);
	ne.setRadiusSearch(300.0);  // 设置搜索半径
	ne.compute(*cloud_normals);
	std::cout << ">> Done: " << tt.toc() << " ms.\n"  << std::endl;

	// 建立条件欧式聚类对象
	std::cout << "Segmenting to clusters...\n", tt.tic();
	pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec(true);
	cec.setInputCloud(cloud_normals);
	// 可设置不同的搜索函数（其它方法就手动来改这个数字）
	int method = 3;
	switch (method) {
	case 1:
		cec.setConditionFunction(enforceIntensitySimilarity);
		break;
	case 2:
		cec.setConditionFunction(enforceCurvatureOrIntensitySimilarity);
		break;
	case 3:
		cec.setConditionFunction(&customRegionGrowing);  // 这是函数指针，要不要取址符&都行
		break;
	default:
		cec.setConditionFunction(customRegionGrowing);  // 不加 &，它也会自动转为函数指针
		break;
	}

	cec.setClusterTolerance(500);  //  设置聚类参考点的所搜距离
	cec.setMinClusterSize(cloud_normals->points.size() / 1000);  // 设置过小聚类的标准
	cec.setMaxClusterSize(cloud_normals->points.size() / 5);  // 设置过大聚类的标准
	cec.segment(*clusters);  // 获取聚类的结果，分割结果保存在点云索引的向量中
	cec.getRemovedClusters(small_clusters, large_clusters);  // 获取无效尺寸的聚类
	std::cout << ">> Done: " << tt.toc() << " ms\n" << std::endl;;

	// 使用强度通道对输出进行延迟可视化
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