#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>  // pcl::ModelCoefficients
#include <pcl/segmentation/sac_segmentation.h>  // pcl::SACSegmentation

int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// ����15��������ƣ�x��yΪ�������zΪ1.0
	// ��points��0��3��6����λ�õ�zֵ�����޸ģ�������Ϊ��Ⱥֵ
	cloud->width = 15;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1.0;
	}
	cloud->points[0].z = 2.0; cloud->points[3].z = -2.0; cloud->points[6].z = 4.0;
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;
	for (std::size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;

	/*
	- �����ָ�ʱ����Ҫ��ģ��ϡ����� coefficients ���洢�ڵ�ĵ��������϶��� inliers��
	- ָ������ֵ����DistanceThreshold���ĵط����þ�����ֵȷ���������ģ���ж�Զ���ܱ�����Ϊ��Ⱥ�㡣
	- ���������ֵ��0.01m����ֻҪ�㵽z=1ƽ�����С�ڸ���ֵ�Ĵ��㶼��Ϊ�ڲ��㿴���������ڸ���ֵ��������Ⱥ�㡣
	- ��ʹ��RANSAC����(pcl::SAC_RANSAC)��Ϊ�ɿ��Ĺ�������
	*/
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;  // �ָ����
	seg.setOptimizeCoefficients(true);  // ��ѡ���ã��Ƿ��Ż�ģ��ϵ��
	// ��ѡ���ã����÷ָ��ģ�����͡��ָ��㷨��������ֵ���������
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud);
	// ִ�зָ���������洢�ָ������浽�㼯�� inliers ���洢ƽ��ģ��ϡ�� coefficients 
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return -1;
	}
	// ������ӡ�������ƽ��ģ�͵Ĳ������� ax+by+ca+d=0 ��ʽ��,���RANSAC����һ�����㷨��SACMODEL_PLANEƽ��ģ��
	auto &values = coefficients->values;
	std::cout << "Model Coefficients: " << values[0] << " " << values[1] << " " << values[2] << " " << values[3] << std::endl;

	std::cout << "Model inliers: " << inliers->indices.size() << std::endl;
	for (std::size_t i = 0; i < inliers->indices.size(); ++i)
		std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
		<< cloud->points[inliers->indices[i]].y << " "
		<< cloud->points[inliers->indices[i]].z << std::endl;

	system("pause");
	return 0;
}