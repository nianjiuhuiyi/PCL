#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>  // pcl::MovingLeastSquares 
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char* argv[]) {
	std::cout << "hello" << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("./table_scene_lms400_downsampled.pcd", *cloud);

	// 创建一个Kd-tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);  // 表面重构时需要估计点云的法向量，这里MLS提供了一种方法来估计点云法向量
	mls.setInputCloud(cloud);  // 对于法线的估计是有多项式还是仅仅依靠切线
	mls.setPolynomialOrder(2);  // MLS拟合曲线的阶段，这个阶段在构造函数里默认是2，但是参考文献给出最好选择3或4
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);

	// Reconstruct
	mls.process(mls_points);
	
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {}  // 好像没有显示出来
	pcl::io::savePCDFile("./table_scene_lms400_downsampled-mls.pcd", mls_points);

	return 0;
}
