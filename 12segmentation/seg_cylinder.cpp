#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>  // pcl::ModelCoefficients
#include <pcl/segmentation/sac_segmentation.h>  // pcl::SACSegmentation��pcl::SACSegmentationFromNormals

#include <pcl/filters/passthrough.h>  // pcl::PassThrough
#include <pcl/features/normal_3d.h>  // pcl::NormalEstimation, ��_omp�Ŀ�����OpenMP
#include <pcl/filters/extract_indices.h>  // pcl::ExtractIndices


int main(int argc, char* argv[]) {
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PassThrough<pcl::PointXYZ> pass;  // ֱͨ�˲���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;  // ���߹��ƶ���
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;  // �ָ���
	pcl::ExtractIndices<pcl::PointXYZ> extract;  // ����ȡ���󣨼��������������ȡ����Ҫ��ĵ㣩
	pcl::ExtractIndices<pcl::Normal> extract_normals;  // ������ȡ����
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

	reader.read("./table_scene_mug_stereo_textured.pcd", *cloud);
	std::cout << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	// ����һ�� passthrough ������ȥ�� spurious NaNs
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");  // ������Сд��
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points" << std::endl;

	// ���߹���
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// �����ָ����Ϊ�� planar model ����ȫ���Ĳ���
	seg.setOptimizeCoefficients(true);  // ��ѡ���ã��Ƿ��Ż�ģ��ϵ��
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers_plane, *coefficients_plane);  // �õ� plane inliers �� coefficients
	std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;

	// ����������ȡƽ���inliers
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);  // ���Ϊtrue�����Ǳ���Ҫ���˵ĵ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	extract.filter(*cloud_plane);
	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points" << std::endl;
	if (!cloud_plane->points.empty()) 
		writer.write("./table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

	/* ------------------------------------------------------------------- */
	
	// ȥ�� planar inliers����ȡ֮��Ĳ���
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);  // ȫ������
	extract_normals.setIndices(inliers_plane);  // �������������ƽ��ķ�������������ǰ��Ż���extract_normals.setNegative(true);��ȡ��������cloud_normals2���ǳ�ƽ��������з���
	extract_normals.filter(*cloud_normals2);

	// ����Բ����ָ�������
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);  // ���÷ָ�ģ��ΪԲ����
	seg.setMethodType(pcl::SAC_RANSAC);  // ����RANSAC�㷨���в�������
	seg.setNormalDistanceWeight(0.1);  // ���淨��Ȩ��ϵ��
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);  // �ڵ㵽ģ�͵������� 0.05m
	seg.setRadiusLimits(0, 0.1);  // Բ���뾶��ΧΪ0 -> 0.1m
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
	// д��Ӳ��
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else {
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
		writer.write("./table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder);
	}
	system("pause");
	return 0;
}