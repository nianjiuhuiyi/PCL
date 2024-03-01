#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>  // ��������
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/range_image/range_image.h>  // ���ͼ��
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>  // ���ͼ���ұ߽��

#include <pcl/keypoints/narf_keypoint.h>  // �ؼ�����ҵ�ͷ�ļ�
#include <pcl/features/narf_descriptor.h>


// ȫ�ֲ��������忴��һ�ڵ�ʹ��
float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CoordinateFrame::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

bool rotation_invariant = true;  // ��һ�������Ĳ������Ǳ��룩

void printUsage(const char *progName) {
	std::cout << "\n\nUsage: " << progName << " [options] <sample.pcd>\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
		<< "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
		<< "-m           Treat all unseen points as maximum range readings\n"
		<< "-s <float>   support size for the interest points (diameter of the used sphere - "
					<< "default " << support_size << ")\n"
		<< "-o <0/1>     switch rotational invariant version of the feature on/off "
					<< " (defaule " << (int)rotation_invariant << ")\n"
		<< "-h           this help!\n\n\n";
}

// �����ӿڵ�λ��
void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose) {
	// �ӿڵ�ԭ��pos_vector
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	// ��ת+ƽ��look_at_vector
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	// up_vector
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	// �����������λ��
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2], up_vector[0], up_vector[1], up_vector[2]);
}


int main(int argc, char* argv[]) {

	if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
		printUsage(argv[0]);
		return 0;
	}
	if (pcl::console::find_argument(argc, argv, "-m") >= 0) {
		setUnseenToMaxRange = true;
		std::cout << "Setting unseen values in range image to maximum range readings.\n";
	}
	int tmp_coordinate_frame;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0) {
		coordinate_frame = (pcl::RangeImage::CoordinateFrame)tmp_coordinate_frame;
		std::cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
	}
	if (pcl::console::parse(argc, argv, "-s", support_size) >= 0)
		std::cout << "Setting support size to " << support_size << ".\n";
	if (pcl::console::parse<float>(argc, argv, "-r", angular_resolution) >= 0)
		std::cout << "Setting angular resolution to " << angular_resolution << "deg.\n";

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
		setUnseenToMaxRange = true;
		std:cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
		for (float x = -0.5f; x <= 0.5f; x += 0.01f) {
			for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
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

	// ������������չʾ�������˵Ĵ���������ͼ��ȡ�߽���һ���ģ�
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> rangeImage_handler(rangeImage, 0, 0, 0);
	viewer.addPointCloud(rangeImage, rangeImage_handler, "range image");  // ���ͼ�ĵ���
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	//viewer.addCoordinateSystem(1.0f, "global");  // ��������ϵ��û���У���û������������ϵ��
	// ԭʼ���ƣ�����ʾ�ɲ���ʾ
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 0, 0);
	//viewer.addPointCloud(cloud, cloud_color_handler, "original point cloud");

	viewer.initCameraParameters();
	// �еĽ̳���û��ִ����һ�������Ǻ����
	setViewerPose(viewer, rangeImage->getTransformationToWorldSystem());

	// ���������ͼ�ĵ���չʾ���������ͼչʾ(�������ⵯ����һ��С��)
	pcl::visualization::RangeImageVisualizer range_img_widget("Range image");
	range_img_widget.showRangeImage(*rangeImage);


	// ���Ĳ�����ȡ NARF �ؼ���
	// ����RangeImageBorderExtractor���������������б�Ե��ȡ�ģ���ΪNARF�ĵ�һ������Ҫ̽������ͼ�ı�Ե
	pcl::RangeImageBorderExtractor range_image_border_extrator;  // ������ȡ��Ե
	pcl::NarfKeypoint narf_keypoint_detector;  // �������ؼ���
	narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extrator);
	narf_keypoint_detector.setRangeImage(&(*rangeImage));
	narf_keypoint_detector.getParameters().support_size = support_size;  // ����NARF�Ĳ���
	// �������������ò����ģ��̳����е�Ҳ��ע�͵���
	//narf_keypoint_detector.getParameters().add_points_on_straight_edges = true;  
	//narf_keypoint_detector.getParameters().distance_for_additional_points = 0.5;

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);
	std::cout << "Found " << keypoint_indices.points.size() << " key points." << std::endl;

	// ���Ƿ������ͼ(�ǵ��Ƶ�)�ĵ�������չʾ����ʾ�ؼ��㣬Ҫ�û��Ͱ��⼸��ע�ӵ�
	//for (size_t i = 0; i < keypoint_indices.points.size(); ++i) {
	//	range_img_widget.markPoint(keypoint_indices.points[i] % rangeImage->width, keypoint_indices.points[i] / rangeImage->width);   // ���д���������
	//}  


	// ���岽����3Dͼ��չʾ�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	keypoints->points.resize(keypoint_indices.size());
	for (size_t i = 0; i < keypoint_indices.size(); ++i) {
		keypoints->points[i].getVector3fMap() = rangeImage->points[keypoint_indices.points[i]].getVector3fMap();
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(keypoints, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");


	// ���Ǳ��룩��ȡ��Ȥ���NARF������ (������Ǻ������ô�)
	std::vector<int> keypoints_indices2;
	keypoints_indices2.resize(keypoint_indices.points.size());
	for (size_t i = 0; i < keypoints_indices2.size(); ++i)
		keypoints_indices2[i] = keypoint_indices.points[i];
	pcl::NarfDescriptor narf_descriptor(&(*rangeImage), &keypoints_indices2);
	narf_descriptor.getParameters().support_size = support_size;
	narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
	pcl::PointCloud<pcl::Narf36> narf_descriptors;
	narf_descriptor.compute(narf_descriptors);
	std::cout << "Extracted " << narf_descriptors.size() << " descriptors for " << keypoint_indices.points.size() << " keypoints." << std::endl;

	while (!viewer.wasStopped()) {
		range_img_widget.spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);
	}
	return 0;
}
