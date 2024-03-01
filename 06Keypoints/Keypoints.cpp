#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>  // 参数分析
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/range_image/range_image.h>  // 深度图像
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>  // 深度图像找边界点

#include <pcl/keypoints/narf_keypoint.h>  // 关键点查找的头文件
#include <pcl/features/narf_descriptor.h>


// 全局参数，具体看上一节的使用
float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CoordinateFrame::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

bool rotation_invariant = true;  // 这一节新增的参数（非必须）

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

// 设置视口的位姿
void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose) {
	// 视口的原点pos_vector
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	// 旋转+平移look_at_vector
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	// up_vector
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	// 设置照相机的位姿
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

	// 第一步：读取PCD文件（没给pcd文件的话就自己创建一个点云）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointWithViewpoint>::Ptr far_ranges;  // 这个信息一般都没有，大抵到后面就是空指针
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());  // 传感器位置
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

		// 好像本来也没有这个文件，命令行也没传进来这个文件
		std::string far_ranges_filename = filename.replace(filename.find(".pcd"), 4, "_far_ranges.pcd");
		if (pcl::io::loadPCDFile(far_ranges_filename, *far_ranges) == -1) {
			std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
		}
	}
	else {  // 文件加载失败就自己创建一个点云
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

	// 第二步：从点云创建深度图RangeImage
	float noise_level = 0.0f;
	float min_range = 0.0f;
	int border_size = 1;
	std::shared_ptr<pcl::RangeImage> rangeImage(new pcl::RangeImage());
	// 此函数中的参数含义去看上一小节
	rangeImage->createFromPointCloud(*cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

	// 要先判断是不是空指针
	if (far_ranges) rangeImage->integrateFarRanges(*far_ranges);
	if (setUnseenToMaxRange) rangeImage->setUnseenToMaxRange();

	// 第三步：点云展示，（至此的代码跟从深度图提取边界是一样的）
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> rangeImage_handler(rangeImage, 0, 0, 0);
	viewer.addPointCloud(rangeImage, rangeImage_handler, "range image");  // 深度图的点云
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	//viewer.addCoordinateSystem(1.0f, "global");  // 设置坐标系（没这行，就没有那三根坐标系）
	// 原始点云，可显示可不显示
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 0, 0);
	//viewer.addPointCloud(cloud, cloud_color_handler, "original point cloud");

	viewer.initCameraParameters();
	// 有的教程又没有执行这一步，不是很理解
	setViewerPose(viewer, rangeImage->getTransformationToWorldSystem());

	// 上面是深度图的点云展示，这是深度图展示(就是另外弹出的一个小框)
	pcl::visualization::RangeImageVisualizer range_img_widget("Range image");
	range_img_widget.showRangeImage(*rangeImage);


	// 第四步：提取 NARF 关键点
	// 创建RangeImageBorderExtractor对象，它是用来进行边缘提取的，因为NARF的第一步就需要探测出深度图的边缘
	pcl::RangeImageBorderExtractor range_image_border_extrator;  // 用来提取边缘
	pcl::NarfKeypoint narf_keypoint_detector;  // 用来检测关键点
	narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extrator);
	narf_keypoint_detector.setRangeImage(&(*rangeImage));
	narf_keypoint_detector.getParameters().support_size = support_size;  // 设置NARF的参数
	// 下面这两行设置参数的，教程里有但也是注释掉的
	//narf_keypoint_detector.getParameters().add_points_on_straight_edges = true;  
	//narf_keypoint_detector.getParameters().distance_for_additional_points = 0.5;

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);
	std::cout << "Found " << keypoint_indices.points.size() << " key points." << std::endl;

	// 看是否在深度图(非点云的)的单独窗口展示中显示关键点，要得话就把这几行注视掉
	//for (size_t i = 0; i < keypoint_indices.points.size(); ++i) {
	//	range_img_widget.markPoint(keypoint_indices.points[i] % rangeImage->width, keypoint_indices.points[i] / rangeImage->width);   // 这行代码有问题
	//}  


	// 第五步：在3D图中展示关键点
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	keypoints->points.resize(keypoint_indices.size());
	for (size_t i = 0; i < keypoint_indices.size(); ++i) {
		keypoints->points[i].getVector3fMap() = rangeImage->points[keypoint_indices.points[i]].getVector3fMap();
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(keypoints, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");


	// （非必须）提取兴趣点的NARF描述符 (这个不是很明白用处)
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
