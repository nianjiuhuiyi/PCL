#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d &matrix) {
	printf("Rotation matrix: \n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("\nTranslation vector: \n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

// �鿴���Ļص����������鿴������λ�ڶ���ʱ��ֻҪ����������ͻ���ô˺���
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* nothing) {
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>());

	int iterations = 1;  // Ĭ�ϵ��������������Ǵ����������ģ�������ֱ���ֶ�ָ���ˣ�

	 // �ⴴ�����󣬻����Ĭ�Ϲ��캯�������ü�()�����洴��ָ��ʱ����()�����Ϊ��������
	pcl::console::TicToc time; 
	time.tic();
	if (pcl::io::loadPLYFile("./monkey.ply", *cloud_in) < 0) {
		PCL_ERROR("Error loading cloud %s.\n", "./monkey.ply");
		return -1;
	}
	std::cout << "Load file ./monkey.ply" << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << std::endl;

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	// һ����ת����see https://en.wikipedia.org/wiki/Rotation_matrix ��
	double theta = M_PI / 8;  // The angle of rotation in radians
	transformation_matrix(0, 0) = std::cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = std::cos(theta);
	// ��Z��ƫ��0.4m
	transformation_matrix(2, 3) = 0.4;
	print4x4Matrix(transformation_matrix);

	// ִ�б任
	pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
	*cloud_trans = *cloud_icp;  // ��cloud_icp���ݽ�cloud_trans(��ֻ��Ϊ��չʾ�Ա�)

	// ICP�㷨
	time.tic();
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_icp);
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	// ���ICP�㷨�Ƿ������� �����˳����� �������true�����ǽ�ת������洢��4x4�����У�Ȼ���ӡ���Ծ���ת����
	if (icp.hasConverged()) {
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : ckoud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4Matrix(transformation_matrix);
	}
	else {
		PCL_ERROR("\nICP has not converged.\n");
		return -1;
	}

	// ���ӻ�
	pcl::visualization::PCLVisualizer viewer("ICP Demo");
	// ����������ֱ�ķ�����Ӵ�
	int v1(0), v2(1);
	// (xmin, ymin, xmax, ymax, viewport)
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);  // ���
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);  // �ұ�
	float bckgr_gray_lvl = 0.0;  // ��ɫ
	float txt_gray_lvl = 1.0 - bckgr_gray_lvl;

	// ԭʼ�����ǰ�ɫ���������ڶ���ӣ�
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, (int)255*txt_gray_lvl, (int)255*txt_gray_lvl, (int)255*txt_gray_lvl);
	viewer.addPointCloud(cloud_in, cloud_in_color, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_in, cloud_in_color, "cloud_in_v2", v2);
	// �任��ĵ�������ɫ���ŵ�����ӿڣ�
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_trans_color(cloud_trans, 20, 180, 20);
	viewer.addPointCloud(cloud_trans, cloud_trans_color, "cloud_trans_v1", v1);
	// ICP��׼��ĵ����Ǻ�ɫ���ŵ��ұ��ӿڣ�
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color(cloud_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_icp, cloud_icp_color, "cloud_icp_v2", v2);

	// ��ÿ���ӿ�����ı�����
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
	std::stringstream ss;
	ss << iterations;
	std::string iteration_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iteration_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	viewer.setBackgroundColor(bckgr_gray_lvl, bckgr_gray_lvl, bckgr_gray_lvl, v1);
	viewer.setBackgroundColor(bckgr_gray_lvl, bckgr_gray_lvl, bckgr_gray_lvl, v2);

	// set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);

	// Register keyboard callback :
	viewer.registerKeyboardCallback(&keyboardEventOccurred);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();

		// �����¿ո�
		if (next_iteration) {
			time.tic();
			// ���¼���������ͻ�������������Ȼ����ȥ�ж��Ƿ�Ϊ�ո�
			// Ȼ��ͻ�����鿴��ѭ��������һ���֣�������ICP�����Խ��ж��롣
			// ��סǰ��Ӧ������icp���������/����ƣ�����֮ǰͨ��setMaximumIterations������������������Ϊ1
			icp.align(*cloud_icp);
			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;
			
			// ��ǰ��һ��Ҫ���ICP�Ƿ�����
			if (icp.hasConverged()) {
				// ���������滻13�У��൱���Ǹ����ı�����������������������ʱ���ӿɶ���
				printf("\033[13A");
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;

				//����getFinalTransformation���������ڵ�����������ɵĸ��Ծ���ת�����˴�Ϊ1�ε�������
				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
				print4x4Matrix(transformation_matrix);

				ss.str("");  // �൱�ڰ������ss��������
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, cloud_icp_color, "cloud_icp_v2");
			}
			else {
				PCL_ERROR("\nICP has not converged.\n");
				return -1;
			}
			//�ⲻ��������Ҫ�ġ� ������ǽ����һ���������¾�����ˣ���ô������Ǵӿ�ʼ����ǰ������ת������
		}
		next_iteration = false;
	}

	return 0;

}
