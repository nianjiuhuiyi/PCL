#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>  // pcl::ExtractIndices
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>  // pcl::EuclideanClusterExtraction 
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	
	//pcl::io::loadPCDFile("./results.pcd", *cloud);
	pcl::io::loadPCDFile("./table_scene_lms400_downsampled.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;

	// 降采样滤波，叶子大小1cm
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

	// 创建平面模型分割器并初始化参数
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliners(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.3 * nr_points) {
		// 移除点云中最大的平面（应该是非必须，但有平面的可能会把所有连在一起，影响就很大）
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliners, *coefficients);  // 将分割出来的平面点云索引保存在inliners中
		if (inliners->indices.size() == 0) {
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// 从输入点云中取出平面内点
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliners);
		extract.setNegative(false);
		extract.filter(*cloud_plane);  // 得到平面相关的点cloud_plane
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// 下面一行设为true，得到除平面外剩下的点并保存到cloud_tmp,并重新赋值给cloud_filtered
		extract.setNegative(true);
		extract.filter(*cloud_tmp);
		*cloud_filtered = *cloud_tmp;
	}

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);
	
	// 这个方法这里的核心，创建欧式聚类对象
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setClusterTolerance(0.02);  // 使劲儿中临近搜索的搜索半径（搜搜容差）为2cm
	ece.setMinClusterSize(100);  // 每个簇（集群）的最小值
	ece.setMaxClusterSize(25000);  // 每个簇（集群）的最大值
	ece.setSearchMethod(tree);  // 设置点云搜索算法
	ece.setInputCloud(cloud_filtered);
	ece.extract(cluster_indices);  // 每个簇以索引的形式存到cluster_indices。cluster_indices[0]就是第一个cluster(簇)的所有索引

	pcl::visualization::PCLVisualizer viewer("3D viewer");
	// 为了从点云索引向量中分割出每个簇，必须迭代访问点云索引
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator iter = cluster_indices.cbegin(); iter != cluster_indices.cend(); ++iter) {
		// 每次创建一个新的点云数据集，并且将当前簇的点写进到点云数据中
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = iter->indices.cbegin(); pit != iter->indices.cend(); ++pit) {
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;

		/*  看存不存
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		pcl::io::savePCDFile(ss.str(), *cloud_cluster);
		*/

		std::stringstream ss;
		ss << "cloud_cluster_" << j;
		// PointCloudColorHandlerRandom 自己会指定一个随机的颜色(都会是亮色)
		pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> single_color(cloud_cluster);
		viewer.addPointCloud(cloud_cluster, single_color, ss.str());
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
		j++;
	}

	std::cout << "cloud size: " << cluster_indices.size() << std::endl;
	viewer.spin();
	return 0;
}