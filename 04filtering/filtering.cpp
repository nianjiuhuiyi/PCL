#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  // ���ͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>  // pcl::VoxelGrid ����Ҫ
#include <pcl/filters/uniform_sampling.h>  // pcl::UniformSampling ����Ҫ
#include <pcl/filters/statistical_outlier_removal.h>  // pcl::StatisticalOutlierRemoval ����Ҫ

#define PRINT_POINT( point ) \
	std::cout << "(" << point.x  << ", "<< point.y << ", " << point.z << ")" << std::endl

void pass_through() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (auto &point : cloud->points) {
		point.x = 2 * rand() / (RAND_MAX + 1.0f);
		point.y = 2 * rand() / (RAND_MAX + 1.0f);
		point.z = 2 * rand() / (RAND_MAX + 1.0f);
	}

	std::cout << "filtering before filtering: " << std::endl;
	for (auto &point : cloud->points) {
		PRINT_POINT(point);
	}

	// ����һ�����˵Ķ���
	pcl::PassThrough<pcl::PointXYZ> passThrough;
	passThrough.setInputCloud(cloud);  // 1.��������Դ
	passThrough.setFilterFieldName("z");  // 2.���ù���ʱ����Ҫ�������͵�Z�ֶ�
	passThrough.setFilterLimits(0.0, 1.0);  // 3.���ù��˷�Χ(�������Χ�ڵı���)
	passThrough.filter(*cloud_filtered);  // 4.ִ�й��ˣ���������cloud_filtered
	std::cout << "\nCloud after filtering" << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
		PRINT_POINT(cloud_filtered->points.at(i));
	}

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	// �����һֱ����ֱ�����Ʊ���Ⱦ
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {}
}

void voxel_grid() {
	// ������pcl::PointCloud<pcl::PointXYZ>::Ptr �������Ӧ��Ҳ�ǿ��Ե�
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// ���ļ���ȡ����ͼ
	pcl::PCDReader reader;
	reader.read("C:\\Users\\Administrator\\Downloads\\table_scene_lms400.pcd", *cloud);
	std::cout << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

	// ����һ������߷ֱ���1cm�����ع�������cloud��Ϊ�������ݣ�cloud_filtered��Ϊ�������
	float leftSize = 0.01f;  // ����1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leftSize, leftSize, leftSize);
	sor.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;;

	pcl::PCDWriter writer;
	writer.write("./table_scene_lms400_downsampled.pcd", *cloud_filtered);
}

void uniform_sampleing() {
	// ��ȡ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("C:\\Users\\Administrator\\Downloads\\table_scene_lms400.pcd", *cloud);
	std::cout << "original cloud size: " << cloud->size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_uniform(new pcl::PointCloud<pcl::PointXYZ>());
	// ʹ��unifromSampling�²���
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	double radius = 0.005f;
	uniform_sampling.setRadiusSearch(radius);
	uniform_sampling.filter(*cloud_uniform);
	std::cout << "UniformSampling size: " << cloud_uniform->size() << std::endl;
}


int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("C:\\Users\\Administrator\\Downloads\\table_scene_lms400.pcd", *cloud);
	std::cout << "Cloud before filtering: " << std::endl;
	std::cout << *cloud << std::endl;

	/*
	������������ÿ�����������ʱ��������ھӸ���Ϊ50����
	���ñ�׼����ֵΪ1������ζ�����о����ѯ���ƽ������ı�׼ƫ�������1����׼ƫ������е㶼�������Ϊ��Ⱥֵ��ɾ��
	*/
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);  // ����ƽ��������Ƶ�����ھӵ�����k
	sor.setStddevMulThresh(1.0);  // ���ñ�׼����ֵϵ��
	sor.filter(*cloud_filtered);  // ���˲���������б���
	std::cout << "Cloud after filtering: " << std::endl;
	std::cout << *cloud_filtered << std::endl;

	pcl::PCDWriter writer;
	// ���ǽ����������ĵ��Ϊ _inliers.pcd 
	writer.write<pcl::PointXYZ>("./table_scene_lms400_inliers.pcd", *cloud_filtered);

	// ʹ�õ�����ͨ�Ĺ����������Ƕ�������ȡ�����õ���ȥ����Щ�㣬��Ϊ _outliers.pcd
	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("./table_scene_lms400_outliers.pcd", *cloud_filtered);

	system("pause");
	return 0;
}