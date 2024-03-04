#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

// 其它形状模型，就先去试试看对应的头文件
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>  // 球体模型
#include <pcl/sample_consensus/sac_model_plane.h>  // 平面模型
#include <pcl/visualization/pcl_visualizer.h>


/*
- 使用方法：
 * random_sample_consensus     创建包含外部点的平面
 * random_sample_consensus -f  创建包含外部点的平面，并计算平面内部点
 *
 * random_sample_consensus -s  创建包含外部点的球体
 * random_sample_consensus -sf 创建包含外部点的球体，并计算球体内部点
*/

int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// 给点云填充数据
	cloud->width = 500;
	cloud->height = 1;  //无序点云
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		pcl::PointXYZ &point = cloud->points.at(i);
		if (pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0) {
			/*
			1 * rand() / (RAND_MAX + 1.0) 只能得到0~1之间的数
			2 * rand() / (RAND_MAX + 1.0) 得到0~2之间的数
			再减去1就是为了得到 -1.0~1.0之间的数
			*/
			point.x = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			point.y = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			// 根据 x^2+y^2+z^2=1，来设置球的点云数据
			if (i % 5 == 0)  // 可能散落在球体外
				point.z = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			else if (i % 2 == 0)  // 在球体正方向内
				point.z = sqrt(1 - pow(point.x, 2) - pow(point.y, 2));
			else  // 在球体负方向内
				point.z = -sqrt(1 - pow(point.x, 2) - pow(point.y, 2));
		}
		else {
			point.x = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			point.y = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			// 用x+y+z=1设置一部分点云数据，此时拿点云组成的菱形平面作为内点
			if (i % 2 == 0)
				point.z = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			else
				point.z = -1 * (point.x + point.y);
		}
	}

	// 存储内部点的索引（在全局点中为内部点的索引）
	std::vector<int> inliers;

	// 创建RandomSampleConsensus object并计算合适的模型
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));  // 针对球模型的对象
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));  // 针对平面模型的对象
	if (pcl::console::find_argument(argc, argv, "-f") >= 0) {
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
		ransac.setDistanceThreshold(.01);  // 与平面距离小于0.01的点成为局内点
		ransac.computeModel();
		ransac.getInliers(inliers);  // 存储估计为局内点的索引

		// 得到平面方程
		Eigen::VectorXf coefficient;
		ransac.getModelCoefficients(coefficient);
		std::cout << "平面方程为：\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + " << coefficient[3] << " = 0" << std::endl;
	}
	else if (pcl::console::find_argument(argc, argv, "-sf") >= 0) {
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_sphere);
		ransac.setDistanceThreshold(0.01);
		ransac.computeModel();
		ransac.getInliers(inliers);

		/*
		下面是错误的，都是0，不能像平面方程那样去得到，先放这里做个启示吧
		Eigen::VectorXf coefficient;
		ransac.getModelCoefficients(coefficient);
		std::cout << "球的方程为：\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + " << coefficient[3] << " = 1" << std::endl;
		*/
	}

	// 将cloud中指定索引的点拷贝到 finale_cloud 点云中
	pcl::copyPointCloud(*cloud, inliers, *final_cloud);

	// 可视化
	using pcl::visualization::PCL_VISUALIZER_POINT_SIZE;
	pcl::visualization::PCLVisualizer viewer("Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud(cloud, "sample cloud");
	viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

	if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0) {
		// 这种情况下才去展示 final_cloud，且要再判断下它不为空指针
		if (final_cloud != nullptr) {
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(final_cloud, 255, 0, 0);
			viewer.addPointCloud(final_cloud, color_handler, "final cloud");
			viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 4, "final cloud");
		}
	}

	viewer.addCoordinateSystem(1.0, "global");
	viewer.initCameraParameters();

	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return 0;
}