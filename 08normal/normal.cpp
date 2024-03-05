#include <iostream>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
// pcl::PointCloud<pcl::PointXYZ>��pcl::PCLPointCloud2::Ptr֮���ʽת����Ҫ
#include <pcl/conversions.h> 
#include <pcl/features/normal_3d.h>  // ��������Ҫ��ͷ�ļ�

void func1(pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(original_cloud);

	// ����һ���յ�kdtree����ֵ���ݸ�������ֵ�������
	// ���kd_tree���󽫻���ne�ڲ�������������ݼ�������䣨��������û��������search surface��
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(kd_tree);

	// ʹ��һ���뾶Ϊ3cm�������е������ھӵ�
	ne.setRadiusSearch(0.03);
	ne.compute(*cloud_normals);
}

void func2(pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
	// ׼��һ��indice�����ϼ���Ϊ�˼򵥣�ֱ��ʹ�õ��Ƶ�ǰ10%�ĵ�
	std::vector<int> indices(std::floor(original_cloud->points.size() / 10));
	for (size_t i = 0; i < indices.size(); ++i) indices[i] = i;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(original_cloud);
	// ����indices���� (��Ҫ�������ָ�����ͣ�ֱ��&indicesȡ��ַ�ŵ�ne.setIndices(&indices)����)
	// ����� std::vector<int>(indices) ���˿������죬������һ���µ�ָ�����
	std::shared_ptr<std::vector<int> > indicesptr(new std::vector<int>(indices));
	ne.setIndices(indicesptr);  // �� func1 ��ľ������

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(kd_tree);

	ne.setRadiusSearch(0.03);  // �뾶��������ʹ��k����Ҳ�ǿ��Ե�
	ne.compute(*cloud_normals);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
func3(pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
	// �Ѵ����pcl::PointCloud<pcl::PointXYZ>��ʽת��pcl::PCLPointCloud2
	// pcl::io::loadPCDFile("./table_scene_lms400.pcd", *cloud); ����������ȥ��ȡ�ļ�
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*original_cloud, *cloud);  // �������

	// �ȶ���ܶ�ĵ������²�����PCLPointCloud2��ʽ������grid��������
	float leftSiz = 0.01f;
	pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
	grid.setInputCloud(cloud);
	grid.setLeafSize(leftSiz, leftSiz, leftSiz);
	grid.filter(*cloud_filtered);

	// ���������ݴ� pcl::PCLPointCloud2 ת���� pcl::PointCloud<pcl::PointXYZ>
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(*cloud_filtered, *downsample_cloud);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	// �����������������Ϊ�������
	ne.setInputCloud(downsample_cloud);
	// ��ԭʼ��������Ϊsearch surface
	ne.setSearchSurface(original_cloud);  // ��Ҫ�Ǽ����⺯��

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(kd_tree);
	ne.setRadiusSearch(0.03);  // 3cm
	ne.compute(*cloud_normals);

	return downsample_cloud;
}


int main(int argc, char* argv[]) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("./table_scene_lms400.pcd", *cloud);

	// ����������ݼ�
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());

	auto start = std::chrono::high_resolution_clock::now();	

	//func1(cloud, cloud_normals);
	//func2(cloud, cloud_normals);  // ��������ж���չʾ
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud = func3(cloud, cloud_normals);

	auto end = std::chrono::high_resolution_clock::now();
	std::cout << "��ʱ��" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " s" << std::endl;

	pcl::visualization::PCLVisualizer viewer("3D viewer");
	//viewer.addPointCloud(cloud, "original cloud");  // ��Ҫ��Ҫ�ص���ʾԭ����

	int level = 100;  // ��ʾÿn�������һ��������
	float scale = 0.01;  // ��ʾ��������������Ϊ0.01��
	// ʹ���������ʱ�����������ͷ�������һ��Ҫ��Ȳ��У���Ȼ��û�ж���չʾ�ģ�����func2()����
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(downsample_cloud, cloud_normals, level, scale, "normal");
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	
	return 0;
}