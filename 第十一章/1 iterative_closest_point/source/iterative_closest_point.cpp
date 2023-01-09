/**
 * 程序随机产生一个点云作为源点云，并将其沿X轴平移后作为目标点云，
 * 利用ICP估计源到目标的刚体变换矩阵，中间对所有的信息打印到标准输入输出设备上
*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>//ICP配准类相关的头文件

int
 main (int argc, char** argv)
{
  //创建两个 pcl::PointCloud<pcl::PointXYZ> 共享指针，并初始化它们
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  

  /**
   * 用产生的随机点值构造源点云 cloud_in，并设置合适的参数（width,height,is_dense）
   * 同时打印出保存的点数量和它们的实际坐标值
  */
  cloud_in->width    = 5;//设置点云宽度
  cloud_in->height   = 1;//设置点云为无序点云
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  // 随机数据填充点云数据
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i) 
      std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
      cloud_in->points[i].z << std::endl;

  /**
   * 实现一个简单的点云刚体变换，以构造目标点云，并再次打印数据值
  */
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;
  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<
      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

  /**
   * 创建一个 IterativeClosestPoint 的对象，并设置对应的目标点云和源点云
   * */
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in); //把 cloud_in 设置为点云的源点云
  icp.setInputTarget(cloud_out); //把 cloud_out 设置为与 cloud_in 对应的匹配目标

  /**
   * 创建一个 pcl::PointCloud<pcl::PointXYZ> 实例 Final 对象，存储配准变换后的源点云
   * 应用ICP算法后，IterativeClosestPoint能够保存结果点云集，如果这两个点云配准正确的话
   * （也就是说仅仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云），
   * 那么 icp.hasConverged() （true），然后会输出最终变换矩阵的适合性评估和一些相关信息
  */
  pcl::PointCloud<pcl::PointXYZ> Final;//存储经过配准变换源点云后的点云
  icp.align(Final);//执行配准存储变换后的源点云到Final
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;//打印配准相关输入信息
  std::cout << "Transformation:" << std::endl; 
  std::cout << icp.getFinalTransformation() << std::endl; //打印输出最终估计的变换矩阵
  return (0);
}
