#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/range_image/range_image.h>  // ע�����������ͷ�ļ�
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>  // �����Ҫ������������Ĳ���


void createRangeImage() {
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
}


// ȫ�ֲ���
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
bool setUnseenToMaxRange = false;


void printUsage(const char* progName) {
	std::cout << "\n\nUsage: " << progName << " [options] <sample.pcd>\n\n"
		<< "Options:\n"
		<< "-----------------------------------------\n"
		<< "-r <float>  angular resolution in degree (default " << angular_resolution << ")\n"
		<< "-c <int>    coordinate frame (default " << (int)coordinate_frame << ")\n"
		<< "-m          Treat all unseen points to max range\n"
		<< "-h          this help\n"
		<< "\n\n";
}


int main(int argc, char* argv[]) {
	if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
		printUsage(argv[0]);
		return 0;
	}
	if (pcl::console::find_argument(argc, argv, "-m") >= 0) {
		setUnseenToMaxRange = true;
		std::cout << "Setting unseen values in range image to maximum range readings." << std::endl;
	}
	int tmp_coordinate_frame;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0) {
		std::cout << "Using coordinate frame " << tmp_coordinate_frame << std::endl;
		coordinate_frame = static_cast<pcl::RangeImage::CoordinateFrame>(tmp_coordinate_frame);
	}
	if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0) {
		std::cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
	}
	angular_resolution = pcl::deg2rad(angular_resolution);

	// ��һ������ȡPCD�ļ���û��pcd�ļ��Ļ����Լ�����һ�����ƣ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointWithViewpoint>::Ptr far_ranges;  // �����Ϣһ�㶼û�У���ֵ�������ǿ�ָ��
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());  // ������λ��
	std::vector<int> pcl_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
	if (!pcl_filename_indices.empty()) {
		std::string filename = argv[pcl_filename_indices.at(0)];
		if (pcl::io::loadPCDFile(filename, *cloud) == -1) {
			std::cout << "Was not able to opem file \"" << filename << "\".\n";
			printUsage(argv[0]);
			return 0;
		}
		auto &sensor_origin = cloud->sensor_origin_;
		scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(sensor_origin[0], sensor_origin[1], sensor_origin[2])) * Eigen::Affine3f(cloud->sensor_orientation_);

		// ������Ҳû������ļ���������Ҳû����������ļ�
		std::string far_ranges_filename = filename.replace(filename.find(".pcd"), 4, "_far_ranges.pcd");
		if (pcl::io::loadPCDFile(far_ranges_filename, *far_ranges) == -1) {
			std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
		}
	}
	else {  // �ļ�����ʧ�ܾ��Լ�����һ������
		std:cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
		for (float x = -0.5f; x <= 0.5f; x += 0.01f) {
			for (float y = -0.5f; y <= 0.5f; y+= 0.01f) {
				cloud->points.push_back(pcl::PointXYZ(x, y, 2.0f - y));
			}
		}
		cloud->width = (int)cloud->points.size();
		cloud->height = 1;
	}

	// �ڶ������ӵ��ƴ������ͼRangeImage
	float noise_level = 0.0f;
	float min_range = 0.0f;
	int border_size = 1;
	std::shared_ptr<pcl::RangeImage> rangeImage(new pcl::RangeImage());
	// �˺����еĲ�������ȥ����һС��
	rangeImage->createFromPointCloud(*cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

	// Ҫ���ж��ǲ��ǿ�ָ��
	if (far_ranges) rangeImage->integrateFarRanges(*far_ranges);
	if (setUnseenToMaxRange) rangeImage->setUnseenToMaxRange();

	// ����������ȡ�߽�borders
	pcl::RangeImageBorderExtractor border_exteator(&(*rangeImage));  // �������ָ�룬��������ָ��Ž�ȥ���Զ�ת�����ˣ����Ƚ����ã���ȡ��ַ��
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_exteator.compute(border_descriptions);  // ��ȡ�߽����������

	pcl::PointCloud<pcl::PointWithRange>::Ptr
		border_points(new pcl::PointCloud<pcl::PointWithRange>()),  // ����߽�
		veil_points(new pcl::PointCloud<pcl::PointWithRange>()),    // veil�߽�
		shadow_points(new pcl::PointCloud<pcl::PointWithRange>());  // ��Ӱ�߽�
	for (uint32_t y = 0; y < rangeImage->height; ++y) {
		for (uint32_t x = 0; x < rangeImage->width; ++x) {
			auto idx = y * rangeImage->width + x;
			auto &desc = border_descriptions.points.at(idx);
			if (desc.traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
				border_points->points.push_back(rangeImage->points[idx]);
			if (desc.traits[pcl::BORDER_TRAIT__VEIL_POINT])
				veil_points->points.push_back(rangeImage->points[idx]);
			if (desc.traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
				shadow_points->points.push_back(rangeImage->points[idx]);
		}
	}

	// ���Ĳ���3Dչʾ
	// 4.1��չʾԭʼ����
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0f, "global");  // ��������ϵ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 0, 0);
	viewer.addPointCloud(cloud, cloud_color_handler, "original point cloud");

	// 4.1��չʾ���ͼ��Ҳ���Բ�Ҫ��
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_img_color_handler(rangeImage, 150, 150, 150);
	//viewer.addPointCloud(rangeImage, range_img_color_handler, "range image");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "range image");

	// 4.2��չʾ�߽��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_handler(border_points, 0, 255, 0);
	viewer.addPointCloud<pcl::PointWithRange>(border_points, border_points_handler, "border points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_handler(veil_points, 255, 0, 0);
	viewer.addPointCloud<pcl::PointWithRange>(veil_points, veil_points_handler, "veil points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_handler(shadow_points, 0, 255, 255);
	viewer.addPointCloud<pcl::PointWithRange>(shadow_points, shadow_points_handler, "shadow points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");

	// show points in range image
	pcl::visualization::RangeImageVisualizer* range_img_border_widget = nullptr;
	range_img_border_widget = pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(
		*rangeImage, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false, border_descriptions, "Range image with borders");

	while (!viewer.wasStopped()) {
		range_img_border_widget->spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);
	}
	return 0;
}