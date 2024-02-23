#include <iostream>
#include <sstream>

#include <pcl/point_cloud.h>  // ������
#include <pcl/point_types.h>  // ������������
#include <pcl/io/openni2_grabber.h>  // ���ƻ�ȡ�ӿ���
#include <pcl/visualization/cloud_viewer.h>  // ���ƿ��ӻ���

#include <pcl/compression/octree_pointcloud_compression.h>  // ����ѹ����



class SimpleOpenNIViewer {
public:
	SimpleOpenNIViewer() : viewer("Point Cloud Compression Example") {
	}
	
	/*
	��OpenNIGrabber�ɼ�ѭ��ִ�еĻص�����cloud_cb_�У����Ȱѻ�ȡ�ĵ���ѹ����stringstream����������һ�����ǽ�ѹ����
	����ѹ���˵Ķ��������ݽ��н��룬�洢���µĵ����н�����,���Ʊ����͵����ƿ��ӻ������н���ʵʱ���ӻ�
	*/
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		if (!this->viewer.wasStopped()) {
			// �洢ѹ�����Ƶ��ֽ�������
			std::stringstream compressedData;
			// �洢�������
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

			// ѹ������
			this->PointCloudEncoder->encodePointCloud(cloud, compressedData);

			// ��ѹ������
			this->PointCloudDecoder->decodePointCloud(compressedData, cloudOut);

			// ���ӻ���ѹ������
			viewer.showCloud(cloudOut);
		}
	
	}

	void run() {
		bool showStatistics = true;  // �����ڱ�׼�豸�������ӡѹ�������Ϣ
		// ѹ��ѡ��������: pcl/compression/compression_profiles.h
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
		
		// ��ʼ��ѹ���ͽ�ѹ����������ѹ���������趨ѹ������ѡ���ѹ����������Դ�����ж�
		this->PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
		this->PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();

		/*
		����Ĵ���ΪOpenNI�����豸ʵ����һ���µĲ���������������ѭ���ص��ӿڣ�ÿ���豸��ȡһ֡���ݾͻص�����һ�Σ�
		����Ļص���������ʵ������ѹ���Ϳ��ӻ���ѹ�������
		*/
		// ������OpenNI��ȡ���Ƶ�ץȡ����
		pcl::Grabber *interface = new pcl::Grabber();

		// �����ص�����
		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f = boost::bind(&this->cloud_cb_, this, boost::placeholders::_1);

		// �����ص������ͻص���Ϣ�İ�
		boost::signals2::connection con = interface->registerCallback(f);


	}

	pcl::visualization::CloudViewer viewer;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudEncoder, *PointCloudDecoder;
};


int main(int argc, char* argv[]) {

	return 0;
}