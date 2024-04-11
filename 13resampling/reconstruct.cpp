//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>  // pcl::MovingLeastSquares 
//#include <pcl/visualization/cloud_viewer.h>
//
//int main(int argc, char* argv[]) {
//	std::cout << "hello" << std::endl;
//	
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::io::loadPCDFile("./table_scene_lms400_downsampled.pcd", *cloud);
//
//	// 创建一个Kd-tree
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//
//	// Output has the PointNormal type in order to store the normals calculated by MLS
//	pcl::PointCloud<pcl::PointNormal> mls_points;
//	// Init object (second point type is for the normals, even if unused)
//	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//	mls.setComputeNormals(true);
//	mls.setInputCloud(cloud);
//	mls.setPolynomialOrder(2);
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(0.03);
//
//	// Reconstruct
//	mls.process(mls_points);
//	
//	pcl::visualization::CloudViewer viewer("Cloud Viewer");
//	viewer.showCloud(cloud);
//	while (!viewer.wasStopped()) {}  // 好像没有显示出来
//	pcl::io::savePCDFile("./table_scene_lms400_downsampled-mls.pcd", mls_points);
//
//	return 0;
//}



#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>  // 为了 computeCloudResolution 函数中一些API
#include <pcl/surface/gp3.h>  // pcl::GreedyProjectionTriangulation 类
#include <pcl/surface/poisson.h>   // pcl::Poisson
#include <pcl/surface/marching_cubes_hoppe.h>  // 好像还有 marching_cubes_rbf.h 这些

#include <pcl/visualization/pcl_visualizer.h>


// 用于计算分辨率
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squareDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	// 这俩是一模一样的，(因为cloud中重载了[]，然后里面函数返回的就是cloud->points[])
	//std::cout << cloud->points[0] << std::endl;
	//std::cout << "\n\n" << (*cloud)[0] << std::endl;

	for (size_t i = 0; i < cloud->size(); ++i) {
		// pcl::isFinite是判断一个浮点数是否是有限的（下面两个写法是一个意思）
		//if (!pcl::isFinite((*cloud)[i])) continue;
		if (!pcl::isFinite(cloud->points[i])) continue;
		// 考虑第二个邻居，因为第一个点是它本身
		nres = tree.nearestKSearch(i, 2, indices, squareDistances);
		if (nres == 2) {
			resolution += std::sqrt(squareDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;
	return resolution;
}

int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("./ism_train_cat.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	double resolution = computeCloudResolution(cloud);

	// 法线估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(20);
	ne.compute(*normals);

	// normals should not contain the point normals + surface curvatures
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::PolygonMesh mesh;


	/*  --------------贪婪投影三角化--------------
	*/
	// 初始化对象
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	// 设置连接点之间的最大距离(最大边长)用于确定k近邻的球半径，默认值为零
	gp3.setSearchRadius(5 * resolution);
	gp3.setMu(2.5);  // 设置最近邻距离的乘子，以得到每个点的最终搜索半径，默认值为零
	gp3.setMaximumNearestNeighbors(100);  // 设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle(M_PI_4);  // 45 degree (pi)最大平面角
	gp3.setMinimumAngle(M_PI / 18);  // 10 degree 每个三角的最小角度
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees 每个三角的最大角度
	gp3.setNormalConsistency(false);  // 如果法向量一直，设置为true
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(mesh);

	// 添加顶点信息（vertex information）
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	/*   --------------泊松曲面重建--------------

	pcl::Poisson<pcl::PointNormal> pn;
	pn.setConfidence(false);
	pn.setDegree(2);
	pn.setDepth(8);
	pn.setManifold(false);
	pn.setOutputPolygons(false);
	pn.setSamplesPerNode(3.0);
	pn.setScale(1.25);
	pn.setSolverDivide(8);
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
	pn.performReconstruction(mesh);
	
	*/

	/*   --------------移动立方体--------------
	
	// 这里似乎用到了多态，父类指针指向了子类对象
	pcl::MarchingCubes<pcl::PointNormal>::Ptr mc(new pcl::MarchingCubesHoppe<pcl::PointNormal>);
	mc->setIsoLevel(0.0f);
	mc->setGridResolution(5, 5, 5);
	mc->setPercentageExtendGrid(0.0f);
	mc->setSearchMethod(tree2);
	mc->setInputCloud(cloud_with_normals);
	mc->reconstruct(mesh);
	
	*/

	pcl::visualization::PCLVisualizer viewer;
	viewer.addPolygonMesh(mesh, "mesh");
	viewer.spin();

	return 0;
}

