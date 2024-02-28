#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/range_image/range_image.h>  // 注意别忘了这俩头文件
#include <pcl/visualization/range_image_visualizer.h>

int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// 创建一个矩形形状的点云
	for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
		for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
			cloud->points.push_back(pcl::PointXYZ(2.0f - y, y, z));
		}
	}
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->height = 1;

	// 或者直接加载一个已有的点云
	//pcl::io::loadPCDFile("C:\\Users\\Administrator\\Downloads\\rabbit.pcd", *cloud);  // 

	// 根据前面得到的点云图(核心是下面这些参数)，通过1deg的分辨率生成深度图
	float angularResolution = static_cast<float>(1.0f * (M_PI / 180.0f));  // 1°
	float maxAngleWidth = static_cast<float>(360.0f * (M_PI / 180.0f));  // 360°
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180°
	Eigen::Affine3f sensorPose = static_cast<Eigen::Affine3f>(Eigen::Translation3f(0.0f, 0.0f, 0.0f));  // 采集位置
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;  // 相机坐标系
	float noiseLevel = 0.0f;
	float minRange = 0.0f;
	int borderSize = 1;

	//boost::shared_ptr<pcl::RangeImage> rangeImage(new pcl::RangeImage());
	std::shared_ptr<pcl::RangeImage> rangeImage(new pcl::RangeImage());
	rangeImage->createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	std::cout << *rangeImage << std::endl;
	
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	// 添加原始点云图 (橘红色)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 255, 100, 0);
	viewer.addPointCloud(cloud, cloud_color, "original image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original image");

	// 添加深度图点云（黑色的点）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_img_color_handler(rangeImage, 0, 0, 0);
	viewer.addPointCloud(rangeImage, range_img_color_handler, "range image");  
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "range image");  // 这里名字要跟上一行中的名字保持一致

	viewer.initCameraParameters();
	viewer.addCoordinateSystem(1.0);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
		pcl_sleep(0.01);
	}
	return 0;
}