#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>

/**
 * 法线估计类 NormalEstimation 的实际计算调用程序内部执行以下操作：
 * 对点云P中的每个点p:
 *   1.得到p点的最近邻元素
 *   2.计算p点的表面法线n
 *   3.检查n的方向是否一致指向视点，如果不是则翻转
*/

int
main ()
 {
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::io::loadPCDFile ("../table_scene_lms400.pcd", *cloud);
     
     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
     ne.setInputCloud (cloud);
     
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
     ne.setSearchMethod (tree);
     
     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

     ne.setRadiusSearch (0.03);
     
     ne.compute (*cloud_normals);
     // cloud_normals->points.size ()Ӧ����input cloud_downsampled->points.size ()����ͬ�ߴ�
     //���߿��ӻ�
     pcl::visualization::PCLVisualizer viewer("PCL Viewer");
     viewer.setBackgroundColor (0.0, 0.0, 0.0);
     viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);

     while (!viewer.wasStopped ())
     {
          viewer.spinOnce ();
     }

     return 0;
}
