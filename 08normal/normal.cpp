#include <iostream>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
// pcl::PointCloud<pcl::PointXYZ>、pcl::PCLPointCloud2::Ptr之间格式转换需要
#include <pcl/conversions.h> 
#include <pcl/features/normal_3d.h>  // 法向量需要的头文件

void func1(pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(original_cloud);

	// 创建一个空的kdtree，将值传递给法向量值估算对象
	// 这个kd_tree对象将会在ne内部根据输入的数据集进行填充（这里设置没有其它的search surface）
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(kd_tree);

	// 使用一个半径为3cm的球体中的所有邻居点
	ne.setRadiusSearch(0.03);
	ne.compute(*cloud_normals);
}

void func2(pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
	// 准备一个indice索引合集，为了简单，直接使用点云的前10%的点
	std::vector<int> indices(std::floor(original_cloud->points.size() / 10));
	for (size_t i = 0; i < indices.size(); ++i) indices[i] = i;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(original_cloud);
	// 设置indices索引 (需要这个智能指针类型，直接&indices取地址放到ne.setIndices(&indices)不对)
	// 下面的 std::vector<int>(indices) 用了拷贝构造，构建了一个新的指针对象
	std::shared_ptr<std::vector<int> > indicesptr(new std::vector<int>(indices));
	ne.setIndices(indicesptr);  // 比 func1 多的就是这个

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(kd_tree);

	ne.setRadiusSearch(0.03);  // 半径内搜索，使用k近邻也是可以的
	ne.compute(*cloud_normals);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
func3(pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
	// 把传入的pcl::PointCloud<pcl::PointXYZ>格式转成pcl::PCLPointCloud2
	// pcl::io::loadPCDFile("./table_scene_lms400.pcd", *cloud); 而不是这样去读取文件
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*original_cloud, *cloud);  // 这个函数

	// 先对这很多的点云做下采样（PCLPointCloud2格式才能做grid降采样）
	float leftSiz = 0.01f;
	pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
	grid.setInputCloud(cloud);
	grid.setLeafSize(leftSiz, leftSiz, leftSiz);
	grid.filter(*cloud_filtered);

	// 那最终数据从 pcl::PCLPointCloud2 转换成 pcl::PointCloud<pcl::PointXYZ>
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(*cloud_filtered, *downsample_cloud);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	// 将降采样后的数据作为输入点云
	ne.setInputCloud(downsample_cloud);
	// 传原始的数据作为search surface
	ne.setSearchSurface(original_cloud);  // 主要是加了这函数

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(kd_tree);
	ne.setRadiusSearch(0.03);  // 3cm
	ne.compute(*cloud_normals);

	return downsample_cloud;
}


int main(int argc, char* argv[]) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("./table_scene_lms400.pcd", *cloud);

	// 定义输出数据集
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());

	auto start = std::chrono::high_resolution_clock::now();	

	//func1(cloud, cloud_normals);
	//func2(cloud, cloud_normals);  // 这个不会有东西展示
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud = func3(cloud, cloud_normals);

	auto end = std::chrono::high_resolution_clock::now();
	std::cout << "用时：" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " s" << std::endl;

	pcl::visualization::PCLVisualizer viewer("3D viewer");
	//viewer.addPointCloud(cloud, "original cloud");  // 看要不要重叠显示原点云

	int level = 100;  // 表示每n个点绘制一个法向量
	float scale = 0.01;  // 表示法向量长度缩放为0.01倍
	// 使用这个方法时，点云数量和法线数量一定要相等才行，不然是没有东西展示的，比如func2()就是
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(downsample_cloud, cloud_normals, level, scale, "normal");
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	
	return 0;
}