#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h> // 点云坐标变换
#include <pcl/visualization/pcl_visualizer.h>

//#include <pcl/point_cloud.h>        //点类型定义头文件
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件

#include "testmain.h"

TestMain::TestMain()
{

}

void TestMain::test_base1(){
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (auto &point : cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cout << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

    for (const auto &point : cloud)
        std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
}

// This function displays the help
static void showHelp_transform(char *program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}
int TestMain::test_transform(int argc, char ** argv){
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp_transform(argv[0]);
        return 0;
    }

    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

    if (filenames.size() != 1)
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

        if (filenames.size() != 1)
        {
            showHelp_transform(argv[0]);
            return -1;
        }
        else
        {
            file_is_pcd = true;
        }
    }

    // Load file | Works with PCD and PLY files 在参数中寻找pcd或者ply文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (file_is_pcd)
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp_transform(argv[0]);
            return -1;
        }
    }
    else
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp_transform(argv[0]);
            return -1;
        }
    }

    /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f ====================================第二种方法
    This is the "manual" method, perfect to understand but error prone !
  */
    // 创建4x4单位阵
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = M_PI / 4; // The angle of rotation in radians
    transform_1(0, 0) = std::cos(theta);
    transform_1(0, 1) = -sin(theta);
    transform_1(1, 0) = sin(theta);
    transform_1(1, 1) = std::cos(theta);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1(0, 3) = 2.5; //x轴平移 2.5

    // Print the transformation
    printf("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    /*  METHOD #2: Using a Affine3f ====================================第二种方法
    This method is easier and less error prone
  */
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 2.5, 0.0, 0.0;

    // The same rotation matrix as before; theta radians around Z axis //绕z轴旋转
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    printf("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

    // Visualization  可视化
    printf("\nPoint cloud colors :  white  = original point cloud\n"
           "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
    viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

    return 0;
}
//https://www.cnblogs.com/li-yao7758258/p/6437440.html
void TestMain::test_kdtree(){
    srand (time (NULL));   //用系统时间初始化随机种子
    //创建一个PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // 随机点云生成
    cloud->width = 1000;             //此处点云数量
    cloud->height = 1;                //表示点云为无序点云
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)   //循环填充点云数据
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f); // // 产生数值为0-1024的浮点数
        cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    }
    //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // pcl::KdTreeFLANN<PointT, Dist>::setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices)
    //设置搜索空间
    kdtree.setInputCloud (cloud);
    //设置查询点并赋随机值
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

    // K 临近搜索
    //创建一个整数（设置为10）和两个向量来存储搜索到的K近邻，两个向量中，一个存储搜索到查询点近邻的索引，另一个存储对应近邻的距离平方
    int K = 10;

    std::vector<int> pointIdxNKNSearch(K);      //存储查询点近邻索引
    std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方
    //打印相关信息
    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )  //执行K近邻搜索
    {
        //打印所有近邻坐标
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].y
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }
    /**********************************************************************************
       下面的代码展示查找到给定的searchPoint的某一半径（随机产生）内所有近邻，重新定义两个向量
       pointIdxRadiusSearch  pointRadiusSquaredDistance来存储关于近邻的信息
       ********************************************************************************/
    // 半径 R内近邻搜索方法

    std::vector<int> pointIdxRadiusSearch;           //存储索引
    std::vector<float> pointRadiusSquaredDistance;   //存储对应距离的平方

    float radius = 256.0f * rand () / (RAND_MAX + 1.0f);   //随机的生成某一半径
    //打印输出
    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;

    // radiusSearch  返回找到的点的个数
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )  //执行半径R内近邻搜索方法
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
                      << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
                      << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

}
