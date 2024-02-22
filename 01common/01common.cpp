#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/angles.h>


int main(int argc, char **argv) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("C:\\Users\\Administrator\\OneDrive\\�ĵ�\\note\\pcl\\rabbit.pcd", *source_cloud) < 0) {
		std::cout << "Error loading point cloud file!" << std::endl;
		return -1;
	}

	// ��ʽһ��ʹ�� Matrix4f
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	// ����һ����ת����(see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = M_PI / 4;  // ��ת�Ƕȣ�ע���ǻ�����
	transform_1(0, 0) = cos(theta);
	transform_1(0, 1) = -sin(theta);
	transform_1(1, 0) = sin(theta);
	transform_1(1, 1) = cos(theta);
	transform_1(0, 3) = 2.5;  // ��x���϶���һ��2.5m��ƫ��
	std::cout << transform_1 << "\n\n" << std::endl;  // ��ֱ�Ӵ�ӡ

	// ��ʽ����ʹ�� Affine3f
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.translation() << 0.8, 0.0, 0.0;  // ��x��ƫ��0.8m
	// ��Z����ת45�ȣ���ʱ�룩
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	std::cout << transform_2.matrix() << "\n\n" << std::endl;

	// ����仯��ĵ���ָ�벢ִ�б任�õ��任������
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// transform_2Ҳ��һ����,��Ҫ<pcl/common/transforms.h>ͷ�ļ�
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);

	// ���ӻ�
	std::cout << "��ɫ��ԭʼ����\n" << "��ɫ���任��ĵ���" << std::endl;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);  // �����ʼ������ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 255, 0, 0);  // ����任�������ɫ

	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
	// �ѵ�����ӽ����Ӵ��ڣ�������ɫ���ݽ�ȥ
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	// ��������ϵϵͳ
	viewer.addCoordinateSystem(0.5, "cloud", 0);
	// ���ñ�����ɫ(�ڻ�ɫ)
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	// ������Ⱦ����(���С)
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");  // ע�����������ָ�����46�еı���һ��
	// ���ÿ��Ǵ���λ�ã��Ǳ��룩
	viewer.setPosition(800, 400);
	// һֱѭ����ֱ����q�˳�
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	
	return 0;
}