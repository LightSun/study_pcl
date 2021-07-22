#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>  //法线特征
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2
#include <pcl/features/pfh.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

//#include <pcl/point_types.h>
#include <pcl/features/pfh.h>

//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

#include "study_features.h"

typedef pcl::PointXYZ PointType;

static void showPCL(const pcl::PointCloud<PointType>::Ptr &cloud, const pcl::PointCloud<PointType>::Ptr &cloud2) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);

    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, 255, 0, 0);
    viewer->addPointCloud<PointType>(cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    if(cloud2 != NULL){
        pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color2(cloud2, 255, 0, 0);
        viewer->addPointCloud<PointType>(cloud2, single_color2, "sample cloud 2");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud 2");
    }
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}

void Study_Features::test_1(const std::string& pcdFile, const std::string& savePath)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile(pcdFile, *cloud);
    //TODO ... read, pass in or create a point cloud ...

    // 创建法向量估算类
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // 创建一个空的kdtree，将值传递给法向量估算对象
    // 这个tree对象将会在ne内部根据输入的数据集进行填充（这里设置没有其他的search surface）
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // 定义输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // 使用一个半径为3cm的球体中的所有邻居点
    ne.setRadiusSearch (0.03);

    ne.compute (*cloud_normals);

    // cloud_normals->points.size () 输出特征的点个数应当定于输入的点个数 cloud->points.size ()
    pcl::io::savePCDFileASCII(savePath, *cloud_normals);
}
void Study_Features::test_2(const std::string& pcdFile, const std::string& savePath)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcdFile, *cloud);
    //... read, pass in or create a point cloud ...

    // 准备一个indices索引集合，为了简单起见，我们直接使用点云的前10%的点
    std::vector<int> indices (std::floor (cloud->points.size () / 10));
    for (std::size_t i = 0; i < indices.size (); ++i) indices[i] = i;

    // 创建法向量估算类，设置输入点云
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // 设置indices索引
    boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
    ne.setIndices (indicesptr);

    // 创建一个空的kdtree，将值传递给法向量估算对象
    // 这个tree对象将会在ne内部根据输入的数据集进行填充（这里设置没有其他的search surface）
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // 定义输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // 使用一个半径为3cm的球体中的所有邻居点
    ne.setRadiusSearch (0.03);

    // 计算特征
    ne.compute (*cloud_normals);

    // cloud_normals->points.size () 输出特征的点个数应当定于输入的点个数 cloud->points.size ()
     pcl::io::savePCDFileASCII(savePath, *cloud_normals);
}
void Study_Features::test_3(const std::string& pcdFile, const std::string& downsample_pcdFile, const std::string& savePath)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcdFile, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(downsample_pcdFile, *cloud);

    //... read, pass in or create a point cloud ...
    // ... create a downsampled version of it ...

    // 创建法向量估算类，将降采样后的数据作为输入点云
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_downsampled);

    // 传入降采样之前的原始数据作为search surface
    ne.setSearchSurface (cloud);

    // 创建一个空的kdtree，将值传递给法向量估算对象
    // 这个tree对象将会在ne内部根据输入的数据集进行填充（这里设置没有其他的search surface）
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // 定义输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // 使用一个半径为3cm的球体中的所有邻居点
    ne.setRadiusSearch (0.03);

    // 计算特征
    ne.compute (*cloud_normals);

    // cloud_normals->points.size()输出特征的点个数应当定于输入的点个数cloud->points.size()
    pcl::io::savePCDFileASCII(savePath, *cloud_normals);
}

void Study_Features::test_fth(const std::string& pcdFile){
    //ism_test_cat.pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //
    pcl::PCDReader reader;
    reader.read(pcdFile, *cloud_ptr);
    // =====【2】计算法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_ptr);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);//设置近邻搜索算法
    // 输出点云 带有法线描述
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);//半价内搜索临近点 3cm
    // 计算表面法线特征
    ne.compute(*cloud_normals_ptr);

    //=======【3】创建PFH估计对象pfh，并将输入点云数据集cloud和法线normals传递给它=================
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;// phf特征估计其器
    pfh.setInputCloud(cloud_ptr);
    pfh.setInputNormals(cloud_normals_ptr);
    //如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);
    //创建一个空的kd树表示法，并把它传递给PFH估计对象。
    //基于已给的输入数据集，建立kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); //-- older call for PCL 1.5-
    pfh.setSearchMethod(tree2);//设置近邻搜索算法
    //输出数据集
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_fe_ptr(new pcl::PointCloud<pcl::PFHSignature125>());//phf特征
    //使用半径在5厘米范围内的所有邻元素。
    //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
    pfh.setRadiusSearch(0.05);
    //计算pfh特征值
    pfh.compute(*pfh_fe_ptr);
    std::cout << "phf feature size : " << pfh_fe_ptr->points.size() << std::endl;
    // 应该与input cloud->points.size ()有相同的大小，即每个点都有一个pfh特征向量

    // ========直方图可视化=============================
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*pfh_fe_ptr, 300); //设置的很坐标长度，该值越大，则显示的越细致
    plotter.plot();
}

void Study_Features::test_4(){
    //打开点云代码
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("table_scene_lms400.pcd", *cloud);
    //创建法线估计估计向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    //创建一个空的KdTree对象，并把它传递给法线估计向量
    //基于给出的输入数据集，KdTree将被建立
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    //存储输出数据
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //使用半径在查询点周围3厘米范围内的所有临近元素
    ne.setRadiusSearch (0.03);
    //计算特征值
    ne.compute (*cloud_normals);
    // cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同的尺寸
    //可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);

    while (!viewer.wasStopped ())
    {
         viewer.spinOnce ();
    }
}

void Study_Features::test_normal_estimation(int argc, char ** argv){
    int m;
    if(argc >= 2){
        m = atoi(argv[1]);
    }
    //打开点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);
    //创建法线估计向量
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    /****************************************************************************************
         三种法线估计方法
          COVARIANCE_MATRIX     模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
         AVERAGE_3D_GRADIENT    模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量
                                积计算法线
         AVERAGE_DEPTH——CHANGE  模式只创建了一个单一的积分图，从而平局深度变化计算法线
         ********************************************************************************************/
    switch (m) {
    case 1:
        ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
        break;
    case 2:
        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
        break;

    case 3:
        ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
        break;

    case 4:
        ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
        break;

    default:
        ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
        break;
    }
    //ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);  //设置法线估计的方式AVERAGE_3D_GRADIENT

    ne.setMaxDepthChangeFactor(0.02f);   //设置深度变化系数
    ne.setNormalSmoothingSize(10.0f);   //设置法线优化时考虑的邻域的大小
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    //可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);  //将法线加入到点云中
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

void Study_Features::test_pfh_estimation(){
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::io::loadPCDFile("./data/target.pcd", *cloud);
    pcl::io::loadPCDFile("bunny.pcd", *cloud);

    // estimate normals ------------------------------------------------------------- 计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for normal estimation.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    //normalEstimation.setIndices()
    normalEstimation.setInputCloud(cloud);
    // For every point, use all neighbors in a radius of 3cm.
    normalEstimation.setRadiusSearch(0.03);

    // A kd-tree is a data structure that makes searches efficient. More about it later.
    // The normal estimation object will use it to find nearest neighbors.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    // Calculate the normals.
    normalEstimation.compute(*normals);

    // Create the PFH estimation class, and pass the input dataset+normals to it ------计算PFH直方图
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
    pfh.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

    // Use all neighbors in a sphere of radius 5cm
    // 使用一个半径为5厘米的球体，作为搜索邻域
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    // 重点： 半径必须要比用于估算法向量的邻域半径要大
    pfh.setRadiusSearch(0.08);

    // Compute the features
    pfh.compute(*pfhs);

    unsigned long size = pfhs->points.size();
    for (int j = 0; j < size; ++j) {
        pcl::PFHSignature125 &signature125 = pfhs->points[j];
        float* h = signature125.histogram;

        printf("%d: %f,%f,%f \n", j, h[1], h[2], h[3]);
    }

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.01, "normals");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}
