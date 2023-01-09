#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main(int argc,char**argv)
{
//实例化模板类 PointCloud，每一个点的类型都被设置成 pcl::PonintXYZ
pcl::PointCloud<pcl::PointXYZ> cloud;
// 创建点云。用随机点的值填充 PointCloud 点云对象
cloud.width=5;
cloud.height=1;
cloud.is_dense=false;
cloud.points.resize(cloud.width*cloud.height);
for(size_t i=0;i<cloud.points.size();++i)
{
cloud.points[i].x=1024*rand()/(RAND_MAX+1.0f);
cloud.points[i].y=1024*rand()/(RAND_MAX+1.0f);
cloud.points[i].z=1024*rand()/(RAND_MAX+1.0f);
}
//把 PointCloud 对象数据存储在 test_pcd.pcd 文件中
pcl::io::savePCDFileASCII("test_pcd.pcd",cloud);
std::cerr<<"Saved "<<cloud.points.size()<<" data points to test_pcd.pcd."<<std::endl;
for(size_t i=0;i<cloud.points.size();++i)
std::cerr<<"    "<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z<<std::endl;
return(0);
}
