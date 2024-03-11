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

// 查看器的回调函数，当查看器窗口位于顶部时，只要按任意键，就会调用此函数
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* nothing) {
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

int main(int argc, char* argv[]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>());

	int iterations = 1;  // 默认迭代次数（本来是传参数进来的，我这里直接手动指定了）

	 // 这创建对象，会调用默认构造函数，不用加()。上面创建指针时加了()，理解为匿名对象
	pcl::console::TicToc time; 
	time.tic();
	if (pcl::io::loadPLYFile("./monkey.ply", *cloud_in) < 0) {
		PCL_ERROR("Error loading cloud %s.\n", "./monkey.ply");
		return -1;
	}
	std::cout << "Load file ./monkey.ply" << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << std::endl;

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	// 一个旋转矩阵（see https://en.wikipedia.org/wiki/Rotation_matrix ）
	double theta = M_PI / 8;  // The angle of rotation in radians
	transformation_matrix(0, 0) = std::cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = std::cos(theta);
	// 在Z轴偏移0.4m
	transformation_matrix(2, 3) = 0.4;
	print4x4Matrix(transformation_matrix);

	// 执行变换
	pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
	*cloud_trans = *cloud_icp;  // 把cloud_icp备份进cloud_trans(这只是为了展示对比)

	// ICP算法
	time.tic();
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_icp);
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	// 检查ICP算法是否收敛； 否则退出程序。 如果返回true，我们将转换矩阵存储在4x4矩阵中，然后打印刚性矩阵转换。
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

	// 可视化
	pcl::visualization::PCLVisualizer viewer("ICP Demo");
	// 创建两个垂直的分离的视窗
	int v1(0), v2(1);
	// (xmin, ymin, xmax, ymax, viewport)
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);  // 左边
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);  // 右边
	float bckgr_gray_lvl = 0.0;  // 黑色
	float txt_gray_lvl = 1.0 - bckgr_gray_lvl;

	// 原始点云是白色（两个窗口都添加）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, (int)255*txt_gray_lvl, (int)255*txt_gray_lvl, (int)255*txt_gray_lvl);
	viewer.addPointCloud(cloud_in, cloud_in_color, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_in, cloud_in_color, "cloud_in_v2", v2);
	// 变换后的点云是绿色（放到左边视口）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_trans_color(cloud_trans, 20, 180, 20);
	viewer.addPointCloud(cloud_trans, cloud_trans_color, "cloud_trans_v1", v1);
	// ICP配准后的点云是红色（放到右边视口）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color(cloud_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_icp, cloud_icp_color, "cloud_icp_v2", v2);

	// 在每个视口添加文本描述
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

		// 当按下空格
		if (next_iteration) {
			time.tic();
			// 按下键盘任意键就会调用这个函数，然后再去判断是否为空格
			// 然后就会允许查看器循环下面这一部分，即调用ICP对象以进行对齐。
			// 记住前面应配置了icp对象的输入/输出云，并且之前通过setMaximumIterations将其最大迭代次数设置为1
			icp.align(*cloud_icp);
			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;
			
			// 和前面一样要检查ICP是否收敛
			if (icp.hasConverged()) {
				// 这是向上替换13行，相当于是更新文本，而不是新增，这样迭代时更加可读。
				printf("\033[13A");
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;

				//函数getFinalTransformation（）返回在迭代过程中完成的刚性矩阵转换（此处为1次迭代）。
				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
				print4x4Matrix(transformation_matrix);

				ss.str("");  // 相当于把上面的ss对象重置
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, cloud_icp_color, "cloud_icp_v2");
			}
			else {
				PCL_ERROR("\nICP has not converged.\n");
				return -1;
			}
			//这不是我们想要的。 如果我们将最后一个矩阵与新矩阵相乘，那么结果就是从开始到当前迭代的转换矩阵。
		}
		next_iteration = false;
	}

	return 0;

}
