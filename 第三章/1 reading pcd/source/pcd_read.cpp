#include <iostream>
#include <pcl/io/pcd_io.h> //PCD读写类头文件
#include <pcl/point_types.h> //PCL中支持的点类型文件

int
main(int argc,char** argv)
{
//创建一个 PointCloud<PointXYZ> boost共享指针并进行实例化
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

if(pcl::io::loadPCDFile<pcl::PointXYZ>("E:/BaiduNetdiskWorkspace/2022year/Point-Cloud-Processing-example/第三章/1 reading pcd/source/test_pcd.pcd",*cloud)==-1)//打开点云文件
{
PCL_ERROR("Couldn't read file test_pcd.pcd\n");
return(-1);
}
std::cout<<"Loaded "
<<cloud->width*cloud->height
<<" data points from test_pcd.pcd with the following fields: "
<<std::endl;
for(size_t i=0;i<cloud->points.size();++i)
std::cout<<"    "<<cloud->points[i].x
<<" "<<cloud->points[i].y
<<" "<<cloud->points[i].z<<std::endl;

return(0);
}
