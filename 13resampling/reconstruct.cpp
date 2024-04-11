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
//	// ����һ��Kd-tree
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
//	while (!viewer.wasStopped()) {}  // ����û����ʾ����
//	pcl::io::savePCDFile("./table_scene_lms400_downsampled-mls.pcd", mls_points);
//
//	return 0;
//}



#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>  // Ϊ�� computeCloudResolution ������һЩAPI
#include <pcl/surface/gp3.h>  // pcl::GreedyProjectionTriangulation ��
#include <pcl/surface/poisson.h>   // pcl::Poisson
#include <pcl/surface/marching_cubes_hoppe.h>  // ������ marching_cubes_rbf.h ��Щ

#include <pcl/visualization/pcl_visualizer.h>


// ���ڼ���ֱ���
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squareDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	// ������һģһ���ģ�(��Ϊcloud��������[]��Ȼ�����溯�����صľ���cloud->points[])
	//std::cout << cloud->points[0] << std::endl;
	//std::cout << "\n\n" << (*cloud)[0] << std::endl;

	for (size_t i = 0; i < cloud->size(); ++i) {
		// pcl::isFinite���ж�һ���������Ƿ������޵ģ���������д����һ����˼��
		//if (!pcl::isFinite((*cloud)[i])) continue;
		if (!pcl::isFinite(cloud->points[i])) continue;
		// ���ǵڶ����ھӣ���Ϊ��һ������������
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

	// ���߹���
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


	/*  --------------̰��ͶӰ���ǻ�--------------
	*/
	// ��ʼ������
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	// �������ӵ�֮���������(���߳�)����ȷ��k���ڵ���뾶��Ĭ��ֵΪ��
	gp3.setSearchRadius(5 * resolution);
	gp3.setMu(2.5);  // ��������ھ���ĳ��ӣ��Եõ�ÿ��������������뾶��Ĭ��ֵΪ��
	gp3.setMaximumNearestNeighbors(100);  // ��������������ڵ���������
	gp3.setMaximumSurfaceAngle(M_PI_4);  // 45 degree (pi)���ƽ���
	gp3.setMinimumAngle(M_PI / 18);  // 10 degree ÿ�����ǵ���С�Ƕ�
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees ÿ�����ǵ����Ƕ�
	gp3.setNormalConsistency(false);  // ���������һֱ������Ϊtrue
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(mesh);

	// ��Ӷ�����Ϣ��vertex information��
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	/*   --------------���������ؽ�--------------

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

	/*   --------------�ƶ�������--------------
	
	// �����ƺ��õ��˶�̬������ָ��ָ�����������
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

