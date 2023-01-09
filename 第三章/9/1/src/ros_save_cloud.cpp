#include <ros/ros.h> //ros/ros.h包含了使用ROS系统最基本的头文件
#include <pcl_ros/point_cloud.h> //点云类定义头文件
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <sstream>
#include <string.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr cloud(new PointCloudT);


//std::string world_frame_id;
std::string frame_id;

std::string path;

//回调函数。将点云存储为PCD格式，PCD格式的数据支持两种数据类型存储：ASCII码和BinaryCompressed（二进制）
//这里使用BinaryCompressed数据类型，因其占据的存储空间小，便于提高数据存储的速度。
void
cloud_cb (const PointCloudT::ConstPtr& callback_cloud)
{
  *cloud = *callback_cloud; //点云赋值，获取 msgs 中包含的点云信息
  frame_id = cloud->header.frame_id;
  std::stringstream ss;
  std::string s1;
  ss<<ros::Time::now();
  s1=ss.str();

  // cloud saving:
  std::string file_name=path+"cloud_"+frame_id.substr(0, frame_id.length())+"_"+s1+".pcd"; //设置文件名
  std::cout<<file_name<<std::endl; //输出文件名
  pcl::io::savePCDFileBinary(file_name,*cloud);	//将点云保存到 PCD 文件
}

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "save_cloud"); //初始化ROS并指定节点的名称，注意节点的名称必须唯一
  ros::NodeHandle nh("~"); //为这个进程的节点创建句柄
 // std::cout << "node successfully created!" << std::endl;
	
  //Read some parameters from launch file:
  std::string pointcloud_topic = "/camera/depth_registered/points"; //设置topic
  path = "/home/suyang/pointtest/";//设置点云存储路径，这里为绝对路径，也可以使用相对路径
 /* nh.param("pointcloud_topic", pointcloud_topic, std::string("/camera/depth_registered/points"));
  nh.param("world_frame_id", world_frame_id, std::string("/odom"));
  nh.param("path", path, std::string("/home/ubuntu1/pointtest/"));
  std::cout << "Read some parameters from launch file." << std::endl;*/
	
	
  // Subscribers:
  ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);//创建订阅者
  std::cout<<"receive messages successfully!"<<std::endl;
	
  ros::spin(); //进入自循环，等待消息到达，然后调用消息回调函数，一旦 ros::ok() 返回false，即跳出自循环

  return 0;
}

