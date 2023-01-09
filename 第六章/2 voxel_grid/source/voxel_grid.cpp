#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
int
main (int argc, char** argv)
{
     pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
     pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
     // 点云读取对象
     pcl::PCDReader reader;
     // 读取点云文件中的数据到 cloud 对象
     reader.read ("../table_scene_lms400.pcd", *cloud);
     std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
          << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
     /***
      * 接下来，创建一个大小为 1cm 的 pcl::VoxelGrid 滤波器，输入数据作为滤波器的输入，
      * 滤波计算后的输出被存储在 cloud_filtered 中
      */
     pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //创建滤波对象
     sor.setInputCloud (cloud); //给滤波对象设置需要过滤的点云
     sor.setLeafSize (0.01f, 0.01f, 0.01f);//设置滤波时创建的体素大小为 1cm 立方体
     sor.filter (*cloud_filtered);//执行滤波处理，存储输出 cloud_filtered
     std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
          << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

     /**
      * 最后将数据写入磁盘以做他用，例如可视化等
     */
     pcl::PCDWriter writer;
     writer.write ("../table_scene_lms400_downsampled.pcd", *cloud_filtered, 
          Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
     return (0);
}
