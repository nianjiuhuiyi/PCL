#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>


int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("./rabbit.pcd", *cloud);

	// ���㷨����
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);  // ���㷨�����뾶3cm

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	normalEstimation.setSearchMethod(kd_tree);
	normalEstimation.compute(*normals);

	// ���� PFH ֱ��ͼ
	// ��������cloud������ PointNormal������ֱ�� pfh.setInputNormals(cloud)�����淨�߹�����˵��������
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(cloud);
	pfh.setInputNormals(normals);
	// ����һ���յ�kdtree��PFH����
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pfh.setSearchMethod(tree);
	// �������������ص㣺����뾶һ��ҪҪ������ķ��߹��Ƶ������뾶Ҫ��
	pfh.setRadiusSearch(0.08);
	// ���ڴ��������
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