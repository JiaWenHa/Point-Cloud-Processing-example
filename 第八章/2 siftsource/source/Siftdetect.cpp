#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>//包含SIFT关键点估计类头文件
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
using namespace std;

namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
    return p.z;
      }
    };
}

int
main(int argc, char *argv[])
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud_xyz);

  const float min_scale = stof(argv[2]);          
  const int n_octaves = stof(argv[3]);            
  const int n_scales_per_octave = stof(argv[4]);  
  const float min_contrast = stof(argv[5]);       
 
  //创建SIFT关键点估计对象
  pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;//创建SIFT关键点
  pcl::PointCloud<pcl::PointWithScale> result;
  sift.setInputCloud(cloud_xyz);//设置输入点云
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  sift.setSearchMethod(tree);//创建一个空的k-d数对象tree，并把它传递给SIFT检测对象
  /**
   * min_scale -- 用于设置尺度空间中最小尺度的标准偏差
   * n_octaves -- 高斯金字塔中组（Octave）的数目
   * n_scales_per_octave -- 每组(Octave)计算的尺度(scale)数目
  */
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);//ָ指定搜索关键点的尺度范围
  sift.setMinimumContrast(min_contrast);//设置限制关键点检测的阈值
  sift.compute(result);

  //为了后期处理与显示需要，需要将SIFT关键点检测结果转换为点类型为 pcl::PointXYZ 的数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(result, *cloud_temp);//将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据
 
  //���ӻ�������ƺ͹ؼ���
  pcl::visualization::PCLVisualizer viewer("Sift keypoint");
  viewer.setBackgroundColor( 255, 255, 255 );
  viewer.addPointCloud(cloud_xyz, "cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,0,"cloud");
  viewer.addPointCloud(cloud_temp, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"keypoints");

  while(!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  return 0;
  
}