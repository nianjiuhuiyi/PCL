﻿#include <iostream>
#include <pcl/pcl_config.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main(int argc, char **argv) {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	
	// Fill in the cloud data;
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (auto &point : cloud) {
		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	
	pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
	std::cout << "Saved" << cloud.size() << "data points to test_pcd.pcd" << std::endl;

	for (const auto &point : cloud) {
		std::cout << "  " << point.x << " " << point.y << " " << point.z << std::endl;
	}

	std::cout << PCL_VERSION << std::endl;

	system("pause");
	return 0;
}