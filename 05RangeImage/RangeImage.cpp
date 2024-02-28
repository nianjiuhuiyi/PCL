#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/range_image/range_image.h>  // ע�����������ͷ�ļ�
#include <pcl/visualization/range_image_visualizer.h>

int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// ����һ��������״�ĵ���
	for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
		for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
			cloud->points.push_back(pcl::PointXYZ(2.0f - y, y, z));
		}
	}
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->height = 1;

	// ����ֱ�Ӽ���һ�����еĵ���
	//pcl::io::loadPCDFile("C:\\Users\\Administrator\\Downloads\\rabbit.pcd", *cloud);  // 

	// ����ǰ��õ��ĵ���ͼ(������������Щ����)��ͨ��1deg�ķֱ����������ͼ
	float angularResolution = static_cast<float>(1.0f * (M_PI / 180.0f));  // 1��
	float maxAngleWidth = static_cast<float>(360.0f * (M_PI / 180.0f));  // 360��
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180��
	Eigen::Affine3f sensorPose = static_cast<Eigen::Affine3f>(Eigen::Translation3f(0.0f, 0.0f, 0.0f));  // �ɼ�λ��
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;  // �������ϵ
	float noiseLevel = 0.0f;
	float minRange = 0.0f;
	int borderSize = 1;

	//boost::shared_ptr<pcl::RangeImage> rangeImage(new pcl::RangeImage());
	std::shared_ptr<pcl::RangeImage> rangeImage(new pcl::RangeImage());
	rangeImage->createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	std::cout << *rangeImage << std::endl;
	
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	// ���ԭʼ����ͼ (�ٺ�ɫ)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 255, 100, 0);
	viewer.addPointCloud(cloud, cloud_color, "original image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original image");

	// ������ͼ���ƣ���ɫ�ĵ㣩
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_img_color_handler(rangeImage, 0, 0, 0);
	viewer.addPointCloud(rangeImage, range_img_color_handler, "range image");  
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "range image");  // ��������Ҫ����һ���е����ֱ���һ��

	viewer.initCameraParameters();
	viewer.addCoordinateSystem(1.0);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
		pcl_sleep(0.01);
	}
	return 0;
}