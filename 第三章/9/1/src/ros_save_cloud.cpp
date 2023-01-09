#include <ros/ros.h> //ros/ros.h������ʹ��ROSϵͳ�������ͷ�ļ�
#include <pcl_ros/point_cloud.h> //�����ඨ��ͷ�ļ�
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <sstream>
#include <string.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr cloud(new PointCloudT);


//std::string world_frame_id;
std::string frame_id;

std::string path;

//�ص������������ƴ洢ΪPCD��ʽ��PCD��ʽ������֧�������������ʹ洢��ASCII���BinaryCompressed�������ƣ�
//����ʹ��BinaryCompressed�������ͣ�����ռ�ݵĴ洢�ռ�С������������ݴ洢���ٶȡ�
void
cloud_cb (const PointCloudT::ConstPtr& callback_cloud)
{
  *cloud = *callback_cloud; //���Ƹ�ֵ����ȡ msgs �а����ĵ�����Ϣ
  frame_id = cloud->header.frame_id;
  std::stringstream ss;
  std::string s1;
  ss<<ros::Time::now();
  s1=ss.str();

  // cloud saving:
  std::string file_name=path+"cloud_"+frame_id.substr(0, frame_id.length())+"_"+s1+".pcd"; //�����ļ���
  std::cout<<file_name<<std::endl; //����ļ���
  pcl::io::savePCDFileBinary(file_name,*cloud);	//�����Ʊ��浽 PCD �ļ�
}

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "save_cloud"); //��ʼ��ROS��ָ���ڵ�����ƣ�ע��ڵ�����Ʊ���Ψһ
  ros::NodeHandle nh("~"); //Ϊ������̵Ľڵ㴴�����
 // std::cout << "node successfully created!" << std::endl;
	
  //Read some parameters from launch file:
  std::string pointcloud_topic = "/camera/depth_registered/points"; //����topic
  path = "/home/suyang/pointtest/";//���õ��ƴ洢·��������Ϊ����·����Ҳ����ʹ�����·��
 /* nh.param("pointcloud_topic", pointcloud_topic, std::string("/camera/depth_registered/points"));
  nh.param("world_frame_id", world_frame_id, std::string("/odom"));
  nh.param("path", path, std::string("/home/ubuntu1/pointtest/"));
  std::cout << "Read some parameters from launch file." << std::endl;*/
	
	
  // Subscribers:
  ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);//����������
  std::cout<<"receive messages successfully!"<<std::endl;
	
  ros::spin(); //������ѭ�����ȴ���Ϣ���Ȼ�������Ϣ�ص�������һ�� ros::ok() ����false����������ѭ��

  return 0;
}

