#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int
main ()
{
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../room_scan1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);
}



// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/registration/ndt.h> //NDT配准类对应头文件
// #include <pcl/filters/approximate_voxel_grid.h> //滤波类对应头文件
// #include <pcl/visualization/pcl_visualizer.h>
// #include <boost/thread/thread.hpp>
// int
// main (int argc, char** argv)
// {
//   /***
//    * 加载两个pcd文件到共享指针，后续配准是完成对源点云到目标点云额的参考坐标系变换矩阵的估计，
//    * 即得到这里的第二组点云变换到第一组点云坐标系下的变换矩阵
//   */
//   //加载房间的第一次扫描点云数据作为目标
//   pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../room_scan1.pcd", *target_cloud) == -1)
//   {
//     PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
//     return (-1);
//   }
//   std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;
//   //加载从新视角得到的房间的第二次扫描点云数据作为源点云
//   pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../room_scan2.pcd", *input_cloud) == -1)
//   {
//     PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
//     return (-1);
//   }
//   std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

//   /***
//    * 过滤输入点云是为了缩短匹配时间，任何均匀地过滤数据的过滤器都可以完成此部分工作，这里只对源点云进行了滤波处理，
//    * 减少其数据量到原先的大概10%左右，而目标点云不需要滤波处理，因为NDT算法中，在目标点云对应的体素网格数据结构的统计
//    * 计算不使用单个点，而是使用包含在每个体素单元格中的点的统计数据
//   */
//   //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度
//   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
//   approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
//   approximate_voxel_filter.setInputCloud (input_cloud);
//   approximate_voxel_filter.filter (*filtered_cloud);
//   std::cout << "Filtered cloud contains " << filtered_cloud->size ()
//             << " data points from room_scan2.pcd" << std::endl;
  
//   //初始化正态分布变换（NDT）对象
//   pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
//   //根据输入数据的尺度设置NDT相关参数
//   ndt.setTransformationEpsilon (0.01);//为终止条件设置最小转换差异
//   //为More-Thuente线搜索设置最大步长
//   ndt.setStepSize (0.1);
//   //设置NDT网络结构的分辨率(VoxelGridCovariance)
//   ndt.setResolution (1.0);
//   //设置匹配迭代的最大次数
//   ndt.setMaximumIterations (35);
//   // 设置源点云
//   ndt.setInputCloud (filtered_cloud);
//   //设置目标点云
//   ndt.setInputTarget (target_cloud);
//   //设置使用机器人测距法得到的粗略初始变换矩阵结果
//   Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
//   Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
//   Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
//   //计算需要的刚体变换以便将输入的源点云匹配到目标点云
//   pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   ndt.align (*output_cloud, init_guess);
//   //此处的 output_cloud 不能作为最终的源点云变换，因为上面对源点云进行了滤波处理
//   std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
//             << " score: " << ndt.getFitnessScore () << std::endl;
//   //使用创建的变换对未过滤的输入点云进行变换
//   pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
//   //保存转换后的源点云作为最终的变换输出
//   pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);
//   // 初始化点云可视化对象
//   boost::shared_ptr<pcl::visualization::PCLVisualizer>
//   viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer_final->setBackgroundColor (0, 0, 0); //设置背景色为黑色
//   //对目标点云着色（红色）并可视化
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//   target_color (target_cloud, 255, 0, 0);
//   viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
//   viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                   1, "target cloud");
//   //对转换后的源点云着色（绿色）并可视化
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//   output_color (output_cloud, 0, 255, 0);
//   viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
//   viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                   1, "output cloud");
//   // 启动可视化
//   viewer_final->addCoordinateSystem (1.0);//显示 xyz 指示轴
//   viewer_final->initCameraParameters (); //初始化摄像头参数等
//   //等待直到可视化窗口关闭
//   while (!viewer_final->wasStopped ())
//   {
//     viewer_final->spinOnce (100);
//     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//   }
//   return (0);
// }
