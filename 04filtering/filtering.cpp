#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  // ���ͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>  // pcl::VoxelGrid ����Ҫ
#include <pcl/filters/uniform_sampling.h>  // pcl::UniformSampling ����Ҫ
#include <pcl/filters/statistical_outlier_removal.h>  // pcl::StatisticalOutlierRemoval ����Ҫ

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

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

void StatisticalOutlierRemoval() {
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
}


int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

	cloud->width = 100; cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (auto &point : cloud->points) {
		point.x = 1 * rand() / (RAND_MAX + 1.0f);
		point.y = 1 * rand() / (RAND_MAX + 1.0f);
		point.z = 1 * rand() / (RAND_MAX + 1.0f);
	}

	bool IF_CONDITION = false;  // true�������˲���false���뾶��Ⱥֵ�˲�
	if (IF_CONDITION) {  // �����˲�
		// ������������ (z��ֵ����0.25��С��0.75)
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
			new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.25)));
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
			new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.75)));
		// ��������������
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setInputCloud(cloud);
		condrem.setCondition(range_cond);
		condrem.setKeepOrganized(true);
		// Ӧ�ù���,������浽cloud_filtered
		condrem.filter(*cloud_filtered);
	}
	else {  // �뾶��Ⱥֵ�˲�
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
		// ����������
		ror.setInputCloud(cloud);
		ror.setRadiusSearch(0.15);   // 0.15���ǳ���,Խ�������뾶��󣬱����µĵ�ͻ����
		ror.setMinNeighborsInRadius(2);  // �������С�ھӸ�����Խ�����������ľ��٣����µĵ�ͻ���
		ror.filter(*cloud_filtered);
	}
	
	std::cout << "Cloud before filtering: " << std::endl;
	for (auto iter = cloud->points.cbegin(); iter != cloud->points.cend(); ++iter) {
		PRINT_POINT((*iter));
	}
	std::cout << "\n\nCloud after filtering: " << std::endl;
	for (auto &point : cloud_filtered->points) {
		PRINT_POINT(point);
	}

	// չʾ������չʾԭ���ĵ���(��ɫ)���ٰѹ��˺�ĵ���(��ɫ)��ӽ�ȥ����ɫ�ı�������ɫ��һ���֣�λ����һ���ģ���ɫ�ľͻ����ɫ�ĸ��ǣ����Կ��������Ǻ�ġ��̵Ķ��У�
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);  // ������ɫ
	// ����ǰ�ĵ��ƣ�(����ָ����ɫ��Ҳ����ȥ��single_color����������)��rgb����ɫ��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	// ���˺�ĵ��ƣ�(��ɫ)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_filtered, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color2, "cloud_filtered");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_filtered");
	viewer->addCoordinateSystem(1.0);
	while (!viewer->wasStopped()) {
		viewer->spinOnce();
	}
	return 0;
}