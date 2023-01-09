#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/keypoints/harris_3d.h> //harris角点估计类头文件
#include <cstdlib>
#include <vector>
#include <pcl/console/parse.h>
using namespace std;



int main(int argc,char *argv[]) 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile (argv[1], *input_cloud);
	pcl::PCDWriter writer;
	float r_normal;
	float r_keypoint;

	r_normal=stof(argv[2]);
	r_keypoint=stof(argv[3]);

	typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ColorHandlerT3;

	/**
	 * 首先输入代估计关键点的点云，创建 Harris 关键点估计对象，并创建 Harris_keypoints 对象用于保存 Harris 关键点
	 * 注意此处PCL的point类型设置为pcl::PointXYZI，即除了x,y,z坐标还必须包含强度信息。
	*/
	pcl::PointCloud<pcl::PointXYZI>::Ptr Harris_keypoints (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI,pcl::Normal>* harris_detector = new pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI,pcl::Normal> ;

	/**
	 * 接下来，设置 Harris 特征检测对象参数， 
	 * setRadius 函数用于设置法向量估计的半径
	 * setRadiusSearch 函数用于设置关键点估计的近邻搜索半径
	 * 可以根据输入代估测关键点点云的尺度，设置下面的参数
	*/
	//harris_detector->setNonMaxSupression(true);
	harris_detector->setRadius(r_normal);
	harris_detector->setRadiusSearch(r_keypoint);
	harris_detector->setInputCloud (input_cloud);
	//harris_detector->setNormals(normal_source);
	//harris_detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::LOWE);
	harris_detector->compute (*Harris_keypoints);
	cout<<"Harris_keypoints 数量："<<Harris_keypoints->size()<<endl;
	writer.write<pcl::PointXYZI> ("Harris_keypoints.pcd",*Harris_keypoints,false);

	pcl::visualization::PCLVisualizer visu3("clouds");
	visu3.setBackgroundColor(255,255,255);
	visu3.addPointCloud (Harris_keypoints, ColorHandlerT3 (Harris_keypoints, 0.0, 0.0, 255.0), "Harris_keypoints");
	visu3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"Harris_keypoints");
	visu3.addPointCloud(input_cloud,"input_cloud");
	visu3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,0,"input_cloud");
	visu3.spin ();
}