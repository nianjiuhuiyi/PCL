#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>  // ISS()
#include <pcl/keypoints/harris_3d.h>  // Harris
#include <pcl/keypoints/sift_keypoint.h>  // SIFT
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

void ISS() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("./table_scene_lms400_downsampled.pcd", *cloud);
	std::cout << "original cloud size: " << cloud->size() << std::endl;
	double resolution = computeCloudResolution(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(6 * resolution);
	iss_detector.setNonMaxRadius(4 * resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);
	iss_detector.setInputCloud(cloud);
	iss_detector.compute(*keypoints);  // �������õ��ؼ������
	std::cout << "key points size: " << keypoints->size() << std::endl;

	pcl::visualization::PCLVisualizer viewer("iss viewer");
	viewer.addPointCloud(cloud);  // ԭʼ����
	// չʾ�ؼ������
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints, 0, 255, 0);
	viewer.addPointCloud(keypoints, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}

void Harris() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("./table_scene_lms400_downsampled.pcd", *cloud);
	std::cout << "original cloud size: " << cloud->size() << std::endl;

	double resolution = computeCloudResolution(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	// ע��keypoints��harris_detectorģ�����������ǡ�PointXYZI��������PointXYZ
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris_detector;
	harris_detector.setNonMaxSupression(true);
	harris_detector.setRadiusSearch(10 * resolution);
	harris_detector.setThreshold(1E-6);
	harris_detector.setSearchMethod(tree);  // ��дҲ���ԣ�Ĭ�Ϲ���kdtree
	harris_detector.setInputCloud(cloud);
	harris_detector.compute(*tmp_keypoints);  // ����õ��ؼ���ĵ���
	pcl::console::print_highlight("Detected %d points!\n", tmp_keypoints->size());

	// ����Ū�����ս��
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*tmp_keypoints, *keypoints);

	pcl::visualization::PCLVisualizer viewer("harris viewer");
	viewer.addPointCloud(cloud, "original cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(cloud, 0, 255, 0);
	viewer.addPointCloud(keypoints, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}

void SIFT() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("./table_scene_lms400_downsampled.pcd", *cloud);
	std::cout << "original cloud size: " << cloud->size() << std::endl;

	double resolution = computeCloudResolution(cloud);  // ģ�ͷֱ���
	// ������
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree_n);
	//ne.setRadiusSearch(10 * resolution);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// ��������
	for (size_t i = 0; i < cloud_normals->points.size(); ++i) {
		cloud_normals->points[i].x = cloud->points[i].x;
		cloud_normals->points[i].y = cloud->points[i].y;
		cloud_normals->points[i].z = cloud->points[i].z;
	}
	// sift����
	const float min_scale = 0.001f;
	const int n_octaves = 5;  // 3
	const int n_scales_per_octave = 6;  // 4
	const float min_contrast = 0.001f;
	// ʹ�÷�������Ϊǿ�ȼ���ؼ��㣬��������rgb��zֵ�����Զ��壬�����Ҫȥ��API��
	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointXYZ> sift;  // pcl::PointXYZ��������pcl::PointWithScale�����߶���Ϣ
	pcl::PointCloud<pcl::PointXYZ>::Ptr keyspoints(new pcl::PointCloud<pcl::PointXYZ>());
	// ע��������������� pcl::PointNormal
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud_normals);
	sift.compute(*keyspoints);
	std::cout << "No of SIFT points in the result are " << keyspoints->points.size() << std::endl;

	pcl::visualization::PCLVisualizer viewer("SIFT Viewer");
	viewer.addPointCloud(cloud);  // ԭʼ����

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keyspoints_color_handler(keyspoints, 0, 255, 0);
	viewer.addPointCloud(keyspoints, keyspoints_color_handler, "keypoints");  // �ؼ���ĵ���
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}


int main(int argc, char* argv[]) {
	//ISS();
	Harris();
	//SIFT();
	return 0;
}