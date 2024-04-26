#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>  // pcl::MovingLeastSquares 
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char* argv[]) {
	std::cout << "hello" << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("./table_scene_lms400_downsampled.pcd", *cloud);

	// ����һ��Kd-tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);  // �����ع�ʱ��Ҫ���Ƶ��Ƶķ�����������MLS�ṩ��һ�ַ��������Ƶ��Ʒ�����
	mls.setInputCloud(cloud);  // ���ڷ��ߵĹ������ж���ʽ���ǽ�����������
	mls.setPolynomialOrder(2);  // MLS������ߵĽ׶Σ�����׶��ڹ��캯����Ĭ����2�����ǲο����׸������ѡ��3��4
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);

	// Reconstruct
	mls.process(mls_points);
	
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {}  // ����û����ʾ����
	pcl::io::savePCDFile("./table_scene_lms400_downsampled-mls.pcd", mls_points);

	return 0;
}
