#include<iostream>
#include<boost/thread/thread.hpp>
#include<pcl/range_image/range_image.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/range_image_visualizer.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/features/range_image_border_extractor.h>
#include<pcl/keypoints/narf_keypoint.h>
#include<pcl/console/parse.h>

typedef pcl::PointXYZ PointType;

//参数
float angular_resolution = 0.5f;     //角坐标分辨率
float support_size = 0.2f;           //感兴趣点的尺寸（球面的直径）
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;     //坐标
bool setUnseenToMaxRange = false;     //是否将所有不可见的点看作最大

//打印帮助
void printUsage(const char * progName)
{
    std::cout<<"\n\nUsage: "<<progName <<" [options] <scene.pcd> \n\n"
        << "Options:\n"
        << "-------------------------------------------\n"
        << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
        << "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
        << "-m           Treat all unseen points as maximum range readings\n"
        << "-s <float>   support size for the interest points (diameter of the used sphere - "
        << "default " << support_size << ")\n"
        << "-h           this help\n"
        << "\n\n";
}

void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
//设置视角的位置
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
        look_at_vector[0], look_at_vector[1], look_at_vector[2],
        up_vector[0], up_vector[1], up_vector[2]);
    viewer.updateCamera();
}


//主函数
int main(int argc, char ** argv)
{
    //解析命令行参数
    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return 0;
    }
    if (pcl::console::find_argument(argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange = true;
        cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    int tmp_coordinate_frame;
    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        //以函数的方式初始化（0：相机框架；1：激光框架）
        coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
        cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
    }
    if (pcl::console::parse(argc, argv, "-s", support_size) >= 0)
        cout << "Setting support size to " << support_size << ".\n";
    if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
        cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
    angular_resolution = pcl::deg2rad(angular_resolution);

    //读取给定的pcd文件或者自行创建随机点云      

    //点云指针
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    //上面点云的别名
    pcl::PointCloud<PointType>&point_cloud = *point_cloud_ptr;
    //带视角的点构成的点云
    pcl::PointCloud<pcl::PointWithViewpoint>far_ranges;
    //仿射变换
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
    //检查参数中是否有pcd格式文件名，返回参数向量中的索引号
    std::vector<int>pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");

    //如果指定了pcd文件，读取pcd文件和对应的远距离pcd文件
    if (!pcd_filename_indices.empty())
    {
        //文件名
        std::string filename = argv[pcd_filename_indices[0]];

        //读取文件
        if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
        {
            std::cerr << "Was not able to open file \"" << filename << "\".\n";
            printUsage(argv[0]);
            return 0;
        }
        //设置传感器的姿势
        scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
            point_cloud.sensor_origin_[1],
            point_cloud.sensor_origin_[2]))*
            Eigen::Affine3f(point_cloud.sensor_orientation_);
        std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
        if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
            std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
    }

    //没有指定pcd文件，生成点云，并填充它
    else
    {
        setUnseenToMaxRange = true;
        cout << "\nNo *.pcd file given =>Genarating example point cloud.\n\n";
        for (float x = -0.5f; x <= 0.5f; x += 0.01f)
        {
            for (float y = -0.5f; y <= 0.5f; y += 0.01f)
            {
                PointType point; point.x = x; point.y = y; point.z = 2.0f - y;

                //设置点云中点坐标
                point_cloud.points.push_back(point);
            }
        }
        point_cloud.width = (int)point_cloud.points.size(); point_cloud.height = 1;
    }


    //从点云创建距离图像‘
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;

    //创建RangeImage对象（指针）
    boost::shared_ptr<pcl::RangeImage>range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage & range_image = *range_image_ptr;
    range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
        scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

    //整合远距离点云
    range_image.integrateFarRanges(far_ranges);
    if (setUnseenToMaxRange)
    {
        range_image.setUnseenToMaxRange();
    }

    //创建3D点云 可视化窗口，并显示点云
    pcl::visualization::PCLVisualizer viewer("3D Viewer");

    //设置背景颜色
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom< pcl::PointWithRange > range_image_color_handler(range_image_ptr, 0, 0, 0);

    //添加点云
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range_image");

    viewer.initCameraParameters();
    setViewerPose(viewer, range_image.getTransformationToWorldSystem());

    ///显示距离图像
    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
    range_image_widget.showRangeImage(range_image);

    //提取NARF关键点

    //创建深度图像的边界提取器，用于提取NARF关键点
    pcl::RangeImageBorderExtractor range_image_border_extractor;

    //创建NARF对象
    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;
    //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
    //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

    //用于存储关键点的索引
    pcl::PointCloud<int> keypoint_indices;
    //计算NARF关键点
    narf_keypoint_detector.compute(keypoint_indices);
    std::cout << "Found" << keypoint_indices.points.size() << " key points.\n";
    //在距离图像显示组件内显示关键点
    //for (size_ti=0; i<keypoint_indices.points.size (); ++i)
    //range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
    //keypoint_indices.points[i]/range_image.width);

    //在3D窗口中显示关键点
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> & keypoints = *keypoints_ptr;
    //点云变形，无序
    keypoints.points.resize(keypoint_indices.points.size());
    for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
    {
        keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>keypoints_color_handler(keypoints_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    //主循环
    while (!viewer.wasStopped())
    {
        range_image_widget.spinOnce();    //处理GUI事件
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}
