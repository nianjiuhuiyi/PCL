#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/angles.h>


int main(int argc, char **argv) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("C:\\Users\\Administrator\\OneDrive\\文档\\note\\pcl\\rabbit.pcd", *source_cloud) < 0) {
		std::cout << "Error loading point cloud file!" << std::endl;
		return -1;
	}

	// 方式一：使用 Matrix4f
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	// 定义一个旋转矩阵(see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = M_PI / 4;  // 旋转角度，注意是弧度制
	transform_1(0, 0) = cos(theta);
	transform_1(0, 1) = -sin(theta);
	transform_1(1, 0) = sin(theta);
	transform_1(1, 1) = cos(theta);
	transform_1(0, 3) = 2.5;  // 在x轴上定义一个2.5m的偏移
	std::cout << transform_1 << "\n\n" << std::endl;  // 可直接打印

	// 方式二：使用 Affine3f
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.translation() << 0.8, 0.0, 0.0;  // 在x轴偏移0.8m
	// 绕Z轴先转45度（逆时针）
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	std::cout << transform_2.matrix() << "\n\n" << std::endl;

	// 定义变化后的点云指针并执行变换得到变换后数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// transform_2也是一样的,需要<pcl/common/transforms.h>头文件
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);

	// 可视化
	std::cout << "白色：原始点云\n" << "红色：变换后的点云" << std::endl;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);  // 定义初始点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 255, 0, 0);  // 定义变换后点云颜色

	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
	// 把点云添加进可视窗口，并把颜色传递进去
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	// 设置坐标系系统
	viewer.addCoordinateSystem(0.5, "cloud", 0);
	// 设置背景颜色(黑灰色)
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	// 设置渲染属性(点大小)
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");  // 注意后面这个名字跟上面46行的保持一致
	// 设置可是窗口位置（非必须）
	viewer.setPosition(800, 400);
	// 一直循环，直到按q退出
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	
	return 0;
}