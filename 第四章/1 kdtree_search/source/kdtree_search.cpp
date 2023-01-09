#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>

int main (int argc, char**argv)
{
  //用系统时间初始化rand()函数的种子，利用时间初始化，每次运行时所产生的随机数都是不同的。
  srand (time (NULL));
  //创建点云对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //随机点云生成
  cloud->width =1000; //此处为点云数量
  cloud->height =1; //此处表示点云为无序点云
  cloud->points.resize (cloud->width * cloud->height);
  //循环填充点云数据
  for (size_t i=0; i< cloud->points.size (); ++i)
  {
    cloud->points[i].x =1024.0f* rand () / (RAND_MAX +1.0f);
    cloud->points[i].y =1024.0f* rand () / (RAND_MAX +1.0f);
    cloud->points[i].z =1024.0f* rand () / (RAND_MAX +1.0f);
  }

  /**
   * 下面的代码块创建了 kdTreeFLANN 对象，并把我们创建的点云设置成输入，然后创建一个
   * searchPoint变量作为查询点，并且为它分配随机坐标值
  */
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//创建k-d tree 对象
  kdtree.setInputCloud (cloud);//设置搜索空间
  pcl::PointXYZ searchPoint;//定义查询点并赋随机值
  searchPoint.x=1024.0f* rand () / (RAND_MAX +1.0f);
  searchPoint.y=1024.0f* rand () / (RAND_MAX +1.0f);
  searchPoint.z=1024.0f* rand () / (RAND_MAX +1.0f);

  /**
   * 现在创建一个整数（设置成10）和两个向量来存储搜索到的k近邻，两个向量中，
   * 一个向量存储搜索到查询点近邻的索引，另一个存储对应近邻的平方距离。
  */
  // k近邻搜索
  int K =10;
  std::vector<int> pointIdxNKNSearch(K);//存储查询点近邻索引
  std::vector<float> pointNKNSquaredDistance(K);//存储近邻点对应平方距离
  //打印相关信息
  std::cout<<"K nearest neighbor search at ("<<searchPoint.x
  <<" "<<searchPoint.y
  <<" "<<searchPoint.z
  <<") with K="<< K <<std::endl;

  /**
   * 假设 k-d tree 对象返回了多于0个近邻，搜索结果已经存储在我们之前创建的两个向量
   * pointIdxNKNSearch,pointNKNSquaredDistance中，并把所有10个近邻的位置打印输出
  */
  //执行k近邻搜索
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >0 )
  {
    //打印出所有近邻坐标
    for (size_t i=0; i<pointIdxNKNSearch.size (); ++i)
      std::cout<<"    "<<   cloud->points[ pointIdxNKNSearch[i] ].x 
      <<" "<< cloud->points[pointIdxNKNSearch[i] ].y 
      <<" "<< cloud->points[pointIdxNKNSearch[i] ].z 
      <<" (squared distance: "<<pointNKNSquaredDistance[i] <<")"<<std::endl;
  }

  /**
   * 下面代码展示找到给定 searchPoint 的某一半径（随机产生）内的所有近邻，它重新定义两个向量
   * pointIdxRadiusSearch,pointRadiusSquaredDistance来存储关于近邻的信息
  */
  // 半径r内近邻搜索方式
  std::vector<int> pointIdxRadiusSearch;//存储近邻索引
  std::vector<float> pointRadiusSquaredDistance;//存储近邻对应的平方距离
  float radius =256.0f* rand () / (RAND_MAX +1.0f);
  std::cout<<"Neighbors within radius search at ("<<searchPoint.x
  <<" "<<searchPoint.y
  <<" "<<searchPoint.z
  <<") with radius="<< radius <<std::endl;
  // 如果k-d tree对象在指定半径内返回多于0个近邻，它将打印输出向量中存储的点的坐标和距离
  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0 )
  {
    for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i)
      std::cout<<"    "<<   cloud->points[ pointIdxRadiusSearch[i] ].x 
      <<" "<< cloud->points[pointIdxRadiusSearch[i] ].y 
      <<" "<< cloud->points[pointIdxRadiusSearch[i] ].z 
      <<" (squared distance: "<<pointRadiusSquaredDistance[i] <<")"<<std::endl;
  }
  return 0;
}
