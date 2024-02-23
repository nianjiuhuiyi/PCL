#include <iostream>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/common/transforms.h>

int user_data;




void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	// 设置背景色为粉红色
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	// 添加一个圆心为o，半径为0.25m的球体
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "I only run once" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer) {
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	// 每次刷新时，移除text，添加新的text
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}

// 回调函数完成显示
void demo_callBack() {
	// 创建点云的智能指针
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	// 加载pcd
	pcl::io::loadPCDFile("C:\\Users\\Administrator\\OneDrive\\文档\\note\\pcl\\rabbit.pcd", *cloud);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	// 这会一直阻塞，直到点云被渲染
	viewer.showCloud(cloud);

	// 只会调用一次，（非必须）
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	// 每次可视化迭代都会调用一次（频繁调用） (非必须)
	viewer.runOnVisualizationThread(viewerPsycho);

	// 循环判断是否退出
	while (!viewer.wasStopped()) {
		// 这里就可以对点云做很多处理
		user_data++;
	}
}

// 直接显示
void demo_show() {
	// 点云数据指针
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("C:\\Users\\Administrator\\OneDrive\\文档\\note\\pcl\\rabbit.pcd", *cloud);

	// 创建PCLVisualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	// 设置背景为灰色（非必须）
	viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);

	// 添加一个普通点云（可以指定颜色，也可以去掉single_color参数不设置）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 255); // 指点点云颜色
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

	// 添加一个0.5倍缩放的坐标系（非必须）
	viewer->addCoordinateSystem(0.5);
	// 一直循环，直到按q退出
	while (!viewer->wasStopped()) {
		// 每次循环调用内部的重绘函数
		viewer->spinOnce();
	}

}


// 反序列化,将点云数据加载到PointCloud对象中
void deserialize() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("C:\\Users\\Administrator\\OneDrive\\文档\\note\\pcl\\rabbit.pcd", *cloud) == 1) {
		PCL_ERROR("READ ERROR!");
		return;
	}
	std::cout << "Load " << cloud->width * cloud->height << "data points" << std::endl;

	// 打印方式一
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		std::cout << "  " << cloud->points[i].x << " " << cloud->points.at(i).y << " " << cloud->points[i].z << std::endl;
	}
	// 打印方式二
	for (auto &point : *cloud) {
		std::cout << "  " << point.x << " " << point.y << " " << point.z << std::endl;
	}
}
// 序列化
void serialize() {
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// 随机生成5个点
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (auto &point : cloud) {
		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);  // 保存成明文
	pcl::io::savePCDFileBinary("test_pcl_bin.pcd", cloud); // 保存成二进制
	pcl::io::savePCDFile("123.pcd", cloud, false);  // flase是存明文，true是存二进制
	std::cout << "Saved" << cloud.size() << "data points to test_pcd.pcd" << std::endl;
}

int main(int argc, char **argv) {
	// demo_callBack();
	// demo_show();
	//transform();

	deserialize();

	system("pause");
	return 0;
}
