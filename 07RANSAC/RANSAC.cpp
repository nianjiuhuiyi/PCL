#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

// ������״ģ�ͣ�����ȥ���Կ���Ӧ��ͷ�ļ�
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>  // ����ģ��
#include <pcl/sample_consensus/sac_model_plane.h>  // ƽ��ģ��
#include <pcl/visualization/pcl_visualizer.h>


/*
- ʹ�÷�����
 * random_sample_consensus     ���������ⲿ���ƽ��
 * random_sample_consensus -f  ���������ⲿ���ƽ�棬������ƽ���ڲ���
 *
 * random_sample_consensus -s  ���������ⲿ�������
 * random_sample_consensus -sf ���������ⲿ������壬�����������ڲ���
*/

int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// �������������
	cloud->width = 500;
	cloud->height = 1;  //�������
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		pcl::PointXYZ &point = cloud->points.at(i);
		if (pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0) {
			/*
			1 * rand() / (RAND_MAX + 1.0) ֻ�ܵõ�0~1֮�����
			2 * rand() / (RAND_MAX + 1.0) �õ�0~2֮�����
			�ټ�ȥ1����Ϊ�˵õ� -1.0~1.0֮�����
			*/
			point.x = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			point.y = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			// ���� x^2+y^2+z^2=1����������ĵ�������
			if (i % 5 == 0)  // ����ɢ����������
				point.z = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			else if (i % 2 == 0)  // ��������������
				point.z = sqrt(1 - pow(point.x, 2) - pow(point.y, 2));
			else  // �����帺������
				point.z = -sqrt(1 - pow(point.x, 2) - pow(point.y, 2));
		}
		else {
			point.x = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			point.y = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			// ��x+y+z=1����һ���ֵ������ݣ���ʱ�õ�����ɵ�����ƽ����Ϊ�ڵ�
			if (i % 2 == 0)
				point.z = 2 * rand() / (RAND_MAX + 1.0) - 1.0;
			else
				point.z = -1 * (point.x + point.y);
		}
	}

	// �洢�ڲ������������ȫ�ֵ���Ϊ�ڲ����������
	std::vector<int> inliers;

	// ����RandomSampleConsensus object��������ʵ�ģ��
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));  // �����ģ�͵Ķ���
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));  // ���ƽ��ģ�͵Ķ���
	if (pcl::console::find_argument(argc, argv, "-f") >= 0) {
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
		ransac.setDistanceThreshold(.01);  // ��ƽ�����С��0.01�ĵ��Ϊ���ڵ�
		ransac.computeModel();
		ransac.getInliers(inliers);  // �洢����Ϊ���ڵ������

		// �õ�ƽ�淽��
		Eigen::VectorXf coefficient;
		ransac.getModelCoefficients(coefficient);
		std::cout << "ƽ�淽��Ϊ��\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + " << coefficient[3] << " = 0" << std::endl;
	}
	else if (pcl::console::find_argument(argc, argv, "-sf") >= 0) {
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_sphere);
		ransac.setDistanceThreshold(0.01);
		ransac.computeModel();
		ransac.getInliers(inliers);

		/*
		�����Ǵ���ģ�����0��������ƽ�淽������ȥ�õ����ȷ�����������ʾ��
		Eigen::VectorXf coefficient;
		ransac.getModelCoefficients(coefficient);
		std::cout << "��ķ���Ϊ��\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + " << coefficient[3] << " = 1" << std::endl;
		*/
	}

	// ��cloud��ָ�������ĵ㿽���� finale_cloud ������
	pcl::copyPointCloud(*cloud, inliers, *final_cloud);

	// ���ӻ�
	using pcl::visualization::PCL_VISUALIZER_POINT_SIZE;
	pcl::visualization::PCLVisualizer viewer("Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud(cloud, "sample cloud");
	viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

	if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0) {
		// ��������²�ȥչʾ final_cloud����Ҫ���ж�������Ϊ��ָ��
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