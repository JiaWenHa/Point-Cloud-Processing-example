/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/
#include <boost/make_shared.hpp> //boost指针相关头文件
#include <pcl/point_types.h>//点类型定义相关头文件
#include <pcl/point_cloud.h>//点云类相关头文件
#include <pcl/point_representation.h>//点表示相关头文件
#include <pcl/io/pcd_io.h>//pcd文件打开存储类相关头文件
#include <pcl/filters/voxel_grid.h>//基于体素网格化的滤波类相关头文件
#include <pcl/filters/filter.h>//滤波相关头文件
#include <pcl/features/normal_3d.h>//法线特征相关头文件
#include <pcl/registration/icp.h>//icp类相关头文件
#include <pcl/registration/icp_nl.h>//非线性icp类相关头文件
#include <pcl/registration/transforms.h>//变换矩阵类相关头文件
#include <pcl/visualization/pcl_visualizer.h>//可视化类相关头文件

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
//�����Ͷ���
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//创建可视化对象
pcl::visualization::PCLVisualizer *p;
//定义存储左右视点ID
int vp_1, vp_2;
/**
 * 声明一个结构体，方便对点云以文件名和点云对象进行成对处理管理，在配准过程中，
 * 可以同时接受多个点云文件输入，程序从第一个文件开始，连续的两两配准处理，然后
 * 存储配准后的点云文件
*/
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;
  PCD() : cloud (new PointCloud) {};
};
struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};
//以< x, y, z, curvature >形式定义一个新的点表示
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    //定义点的维度
    nr_dimensions_ = 4;
  }
  //重载copyToFloatArray方法来将点转化为4维数组
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};
////////////////////////////////////////////////////////////////////////////////
/** �ڿ��ӻ����ڵĵ�һ�ӵ���ʾԴ���ƺ�Ŀ�����
*
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");
  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}
////////////////////////////////////////////////////////////////////////////////
/**�ڿ��ӻ����ڵĵڶ��ӵ���ʾԴ���ƺ�Ŀ�����
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");
  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);
  p->spinOnce();
}
////////////////////////////////////////////////////////////////////////////////
/***
 * 加载数据非常简单，我们迭代其他程序的参数，检查每一个参数是否指向一个pcd文件，如果是，
 * 则创建一个添加到点云矢量data中的 pcd 对象
*/
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  //第一个参数是命令本身，所以从第二个参数开始解析
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // PCD文件名至少为5个字符大小字符串
    if (fname.size () <= extension.size ())
      continue;
    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    //检查参数是否为一个pcd后缀的文件
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      //加载点云并保存在总体的点云列表中
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //从点云中移除 NAN 点
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
      models.push_back (m);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
/***
 * 
*/
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  PointCloud::Ptr src (new PointCloud); //存储滤波后的源点云
  PointCloud::Ptr tgt (new PointCloud); //存储滤波后的目标点云
  pcl::VoxelGrid<PointT> grid; //滤波处理对象
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05); //设置滤波处理时采用的体素大小
    grid.setInputCloud (cloud_src);
    grid.filter (*src);
    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }
  //计算点云法线
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
  pcl::NormalEstimation<PointT, PointNormalT> norm_est; //点云法线估计对象
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);//设置估计对象采用的搜索对象
  norm_est.setKSearch (30);//设置估计时进行搜索用的k数
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);//下面分别估计源和目标点云法线
  pcl::copyPointCloud (*src, *points_with_normals_src);
  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
  //
  //����˵�������Զ����ı�ʾ�����϶��壩
  MyPointRepresentation point_representation;
  //����'curvature'�ߴ�Ȩ���Ա�ʹ����x, y, zƽ��
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);
  //
  // 配准
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;//配准对象
  reg.setTransformationEpsilon (1e-6);//设置收敛判断条件，越小精度越大，收敛也越慢
  //将两个点云中的对应点对之间的(src<->tgt)最大距离设置为10厘米，大于此值的点对不考虑
  //注意：根据你的数据集大小来调整
  reg.setMaxCorrespondenceDistance (0.1);  
  //设置点表示
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
  reg.setInputCloud (points_with_normals_src);//设置源点云
  reg.setInputTarget (points_with_normals_tgt);//设置目标点云
  //
  //因为这是一个指导实例，我们希望显示配准过程的迭代过程，为达到该目的，ICP在内部进行计算时，限制
  //其最大的迭代次数为2，即每迭代两次就认为收敛，停止内部迭代
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2); //设置最大迭代次数

  //手动迭代，每手动迭代一次，在配准结果视口对迭代的最新的结果进行刷新显示
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);
    //在每次迭代过程中，我们记录并积累又ICP返回的变换
    points_with_normals_src = reg_result;
    reg.setInputCloud (points_with_normals_src);
    reg.align (*reg_result);
		//��ÿһ������֮���ۻ�ת��
    Ti = reg.getFinalTransformation () * Ti;
    //如果迭代N次找到的变换和迭代N-1中找到的变换之间的差异小于传给ICP的变换收敛阈值，
    //我们选择源与目标之间更靠近的对应点距离阈值来改善配准过程
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
      prev = reg.getLastIncrementalTransformation ();
    //���ӻ���ǰ״̬
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }
	//
  // 得到目标点云到源点云的变换
  targetToSource = Ti.inverse();
  //
  //把目标点云变换到源点云坐标系下
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
  p->removePointCloud ("source");
  p->removePointCloud ("target");
  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);
	PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();
  p->removePointCloud ("source"); 
  p->removePointCloud ("target");
  //����Դ���Ƶ�ת��Ŀ��
  *output += *cloud_src;
    final_transform = targetToSource;
 }
/* ---[ */
int main (int argc, char** argv)
{
  // 存储管理所有打开的点云
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data); //加载所有点云文件到data
  //检查用户输入
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    PCL_INFO ("Example: %s `rospack find pcl`/test/bun0.pcd `rospack find pcl`/test/bun4.pcd", argv[0]);
    return (-1);
  }
  PCL_INFO ("Loaded %d datasets.", (int)data.size ());
    //创建一个PCL可视化对象
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  //用左半窗口创建视口 vp_1
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  //用右半窗口创建视口 vp_2
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	PointCloud::Ptr result (new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  for (size_t i = 1; i < data.size (); ++i) //循环处理所有点云
  {
    source = data[i-1].cloud; //连续配准
    target = data[i].cloud; //相邻两组点云
    //可视化为配准的源和目标点云
    showCloudsLeft(source, target);
    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    /***
     * 调用子函数完成一组点云的配准，temp返回配准后两组点云在第一组点云坐标下的点云，
     * pairTransform 返回从目标点云target到源点云source的变换矩阵。
    */
    pairAlign (source, target, temp, pairTransform, true);
    //把当前的两两配对后的点云temp转换到全局坐标系下（第一个输入点云的坐标系）返回result
    pcl::transformPointCloud (*temp, *result, GlobalTransform);
    //用当前的两组点云之间的变换更新全局变换
    GlobalTransform = pairTransform * GlobalTransform;
		//保存转换到第一个点云坐标下的当前配准后的两组点云result到文件i.pcd
    std::stringstream ss;
    ss << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);
  }
}
/* ]--- */
