#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>  // pcl::VoxelGrid
#include <pcl/features/normal_3d_omp.h>  // 带 omp 应该是指openMP 
#include <pcl/features/fpfh_omp.h>  // pcl::FPFHEstimationOMP
#include <pcl/registration/sample_consensus_prerejective.h>  // pcl::SampleConsensusPrerejective
#include <pcl/common/time.h>  // pcl::ScopeTime
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointNormal> PointcloudN;

int main(int argc, char* argv[]) {

	PointcloudN::Ptr object(new PointcloudN());
	PointcloudN::Ptr object_aligned(new PointcloudN());
	PointcloudN::Ptr scene(new PointcloudN());
	if (pcl::io::loadPCDFile("./chef.pcd", *object) < 0 ||
		pcl::io::loadPCDFile<pcl::PointNormal>("./rs1.pcd", *scene) < 0) {
		pcl::console::print_error("Error loading object/scene file!\n");
		return 1;
	}

	// 为了加快速度，使用PCL的pcl::VoxelGrid类将对象和场景点云的采样率下降至5mm
	pcl::console::print_highlight("DownSampling...\n");
	pcl::VoxelGrid<pcl::PointNormal> grid;
	const float leaf = 0.005f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(object);
	grid.filter(*object);  // 这应该是直接把结果又给到了object上
	grid.setInputCloud(scene);
	grid.filter(*scene);

	// for scene 估计法线
	pcl::console::print_highlight("Estimating scene normals...\n");
	pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest;
	nest.setRadiusSearch(0.01);
	nest.setInputCloud(scene);
	nest.compute(*scene);

	// 对于下采样点云中的每个点，使用PCL中的pcl::FPFHEstimationOMP类来计算用于对齐过程中用于匹配的快速点特征直方图(FPFH)描述符。
	pcl::console::print_highlight("Estimating features...\n");
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(object);
	fest.setInputNormals(object);
	fest.compute(*object_features);
	fest.setInputCloud(scene);
	fest.setInputNormals(scene);
	fest.compute(*scene_features);

	// 进行对齐 (一些详细的看下面的注解)
	// SampleConsensusPrerejective 实现了有效的RANSAC
	pcl::console::print_highlight("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
	align.setInputSource(object);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene);
	align.setTargetFeatures(scene_features);
	align.setMaximumIterations(50000);  // RANSAC的最大迭代次数
	align.setNumberOfSamples(3);  // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness(5);  // Number of nearest features to use
	align.setSimilarityThreshold(0.9f);  // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(2.5f * leaf);  // Inlier threshold
	align.setInlierFraction(0.25f);  // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}
	// 如果找到了一个具有足够inliers的姿势(占对象点总数的25%以上)，则该算法会收敛，并可以打印可视化结果
	if (align.hasConverged()) {  // hasConverged() 返回的布尔值，代表点云匹配正确
		Eigen::Matrix4f trans_mat = align.getFinalTransformation();
		using pcl::console::print_info;
		print_info("\n");
		print_info("    | %6.3f %6.3f %6.3f | \n", trans_mat(0, 0), trans_mat(0, 1), trans_mat(0, 2));
		print_info("R = | %6.3f %6.3f %6.3f | \n", trans_mat(1, 0), trans_mat(1, 1), trans_mat(1, 2));
		print_info("    | %6.3f %6.3f %6.3f | \n", trans_mat(2, 0), trans_mat(2, 1), trans_mat(2, 2));
		print_info("\n");
		print_info("t = < %0.3f, %0.3f, %0.3f >\n", trans_mat(0, 3), trans_mat(1, 3), trans_mat(2, 3));
		print_info("\n");
		print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

		// show alignment
		pcl::visualization::PCLVisualizer viewer("Alignment");
		viewer.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scene, 0.0, 255.0, 0.0), "scene");
		viewer.addPointCloud(object_aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
		viewer.spin();
	}
	else {
		pcl::console::print_error("Alignment failed!\n");
	}
	return 0;
}