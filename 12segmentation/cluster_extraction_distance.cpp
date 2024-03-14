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

	// �������˲���Ҷ�Ӵ�С1cm
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

	// ����ƽ��ģ�ͷָ�������ʼ������
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
		// �Ƴ�����������ƽ�棨Ӧ���ǷǱ��룬����ƽ��Ŀ��ܻ����������һ��Ӱ��ͺܴ�
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliners, *coefficients);  // ���ָ������ƽ���������������inliners��
		if (inliners->indices.size() == 0) {
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// �����������ȡ��ƽ���ڵ�
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliners);
		extract.setNegative(false);
		extract.filter(*cloud_plane);  // �õ�ƽ����صĵ�cloud_plane
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// ����һ����Ϊtrue���õ���ƽ����ʣ�µĵ㲢���浽cloud_tmp,�����¸�ֵ��cloud_filtered
		extract.setNegative(true);
		extract.filter(*cloud_tmp);
		*cloud_filtered = *cloud_tmp;
	}

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);
	
	// �����������ĺ��ģ�����ŷʽ�������
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setClusterTolerance(0.02);  // ʹ�������ٽ������������뾶�������ݲΪ2cm
	ece.setMinClusterSize(100);  // ÿ���أ���Ⱥ������Сֵ
	ece.setMaxClusterSize(25000);  // ÿ���أ���Ⱥ�������ֵ
	ece.setSearchMethod(tree);  // ���õ��������㷨
	ece.setInputCloud(cloud_filtered);
	ece.extract(cluster_indices);  // ÿ��������������ʽ�浽cluster_indices��cluster_indices[0]���ǵ�һ��cluster(��)����������

	pcl::visualization::PCLVisualizer viewer("3D viewer");
	// Ϊ�˴ӵ������������зָ��ÿ���أ�����������ʵ�������
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator iter = cluster_indices.cbegin(); iter != cluster_indices.cend(); ++iter) {
		// ÿ�δ���һ���µĵ������ݼ������ҽ���ǰ�صĵ�д��������������
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = iter->indices.cbegin(); pit != iter->indices.cend(); ++pit) {
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;

		/*  ���治��
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		pcl::io::savePCDFile(ss.str(), *cloud_cluster);
		*/

		std::stringstream ss;
		ss << "cloud_cluster_" << j;
		// PointCloudColorHandlerRandom �Լ���ָ��һ���������ɫ(��������ɫ)
		pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> single_color(cloud_cluster);
		viewer.addPointCloud(cloud_cluster, single_color, ss.str());
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
		j++;
	}

	std::cout << "cloud size: " << cluster_indices.size() << std::endl;
	viewer.spin();
	return 0;
}