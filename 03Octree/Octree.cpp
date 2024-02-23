#include <iostream>
#include <sstream>

#include <pcl/point_cloud.h>  // 点云类
#include <pcl/point_types.h>  // 点云数据类型
#include <pcl/io/openni2_grabber.h>  // 点云获取接口类
#include <pcl/visualization/cloud_viewer.h>  // 点云可视化类

#include <pcl/compression/octree_pointcloud_compression.h>  // 点云压缩类



class SimpleOpenNIViewer {
public:
	SimpleOpenNIViewer() : viewer("Point Cloud Compression Example") {
	}
	
	/*
	在OpenNIGrabber采集循环执行的回调函数cloud_cb_中，首先把获取的点云压缩到stringstream缓冲区，下一步就是解压缩，
	它对压缩了的二进制数据进行解码，存储在新的点云中解码了,点云被发送到点云可视化对象中进行实时可视化
	*/
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		if (!this->viewer.wasStopped()) {
			// 存储压缩点云的字节流对象
			std::stringstream compressedData;
			// 存储输出点云
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

			// 压缩点云
			this->PointCloudEncoder->encodePointCloud(cloud, compressedData);

			// 解压缩点云
			this->PointCloudDecoder->decodePointCloud(compressedData, cloudOut);

			// 可视化解压缩点云
			viewer.showCloud(cloudOut);
		}
	
	}

	void run() {
		bool showStatistics = true;  // 设置在标准设备上输出打印压缩结果信息
		// 压缩选项详情在: pcl/compression/compression_profiles.h
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
		
		// 初始化压缩和解压缩对象，其中压缩对象需设定压缩参数选项，解压缩按照数据源自行判断
		this->PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
		this->PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();

		/*
		下面的代码为OpenNI兼容设备实例化一个新的采样器，并且启动循环回调接口，每从设备获取一帧数据就回调函数一次，
		这里的回调函数就是实现数据压缩和可视化解压缩结果。
		*/
		// 创建从OpenNI获取点云的抓取对象
		pcl::Grabber *interface = new pcl::Grabber();

		// 建立回调函数
		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f = boost::bind(&this->cloud_cb_, this, boost::placeholders::_1);

		// 建立回调函数和回调信息的绑定
		boost::signals2::connection con = interface->registerCallback(f);


	}

	pcl::visualization::CloudViewer viewer;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudEncoder, *PointCloudDecoder;
};


int main(int argc, char* argv[]) {

	return 0;
}