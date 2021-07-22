#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

//#include <iostream>
#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

//#include <iostream>
//#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>             //模型系数头文件
#include <pcl/filters/project_inliers.h>          //投影滤波类头文件

//#include <iostream>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/bilateral.h>
typedef pcl::PointXYZ PointType;

#include "study_fliter.h"

typedef pcl::PointXYZ PointT;

void Study_Filter::test_pass_through(){
    srand (time (NULL));   //用系统时间初始化随机种子
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                  << cloud->points[i].y << " "
                  << cloud->points[i].z << std::endl;

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);          // 1. 设置输入源
    pass.setFilterFieldName("z");       // 2. 设置过滤域名
    pass.setFilterLimits(500, 1000);     // 3. 设置过滤范围
    //    pass.setFilterLimitsNegative(true); // 设置获取Limits之外的内容
    pass.filter(*cloud_filtered);       // 4. 执行过滤，并将结果输出到cloud_filtered

    std::cout << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        std::cout << "    " << cloud_filtered->points[i].x << " "
                  << cloud_filtered->points[i].y << " "
                  << cloud_filtered->points[i].z << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer(new pcl::visualization::PCLVisualizer("3D viewer A"));
    viewer->addCoordinateSystem (1.0, "global");//show the CoordinateSystem
    viewer->initCameraParameters();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cloud, 255, 20, 20);
    viewer->addPointCloud(cloud, handler, "input_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "input_cloud");

    while (!viewer->wasStopped())  //until q
    {
        viewer->spinOnce ();
    }
}

void Study_Filter::test_voxel_grid(const std::string& pcdFile, const std::string& out_downsample_pcd_file){
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
   // reader.read ("./data/table_scene_lms400.pcd", *cloud); // Remember to download the file first!
    reader.read(pcdFile, *cloud);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList (*cloud) << ").";

    // 创建一个长宽高分别是1cm的体素过滤器，cloud作为输入数据，cloud_filtered作为输出数据
    float leftSize = 0.01f;
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leftSize, leftSize, leftSize);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    // 将结果输出到文件
    pcl::PCDWriter writer;
    writer.write (out_downsample_pcd_file, *cloud_filtered);
}

void Study_Filter::test_statistical_removal(const std::string& in_pcd, const std::string& out_dir){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // 从文件读取点云
    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    //reader.read<pcl::PointXYZ> ("./data/table_scene_lms400.pcd", *cloud);
    reader.read<pcl::PointXYZ> (in_pcd, *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // 创建过滤器，每个点分析计算时考虑的最近邻居个数为50个；
    // 设置标准差阈值为1，这意味着所有距离查询点的平均距离的标准偏差均大于1个标准偏差的所有点都将被标记为离群值并删除。
    // 计算输出并将其存储在cloud_filtered中

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    // 设置平均距离估计的最近邻居的数量K
    sor.setMeanK (50);
    // 设置标准差阈值系数
    sor.setStddevMulThresh (1.0);
    // 执行过滤
    sor.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    // 将留下来的点保存到后缀为_inliers.pcd的文件
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (out_dir + "/table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    // 使用个相同的过滤器，但是对输出结果取反，则得到那些被过滤掉的点，保存到_outliers.pcd文件
    sor.setNegative (true);
    sor.filter (*cloud_filtered);
    writer.write<pcl::PointXYZ> (out_dir + "/table_scene_lms400_outliers.pcd", *cloud_filtered, false);
}

static void showPointClouds(const pcl::PointCloud<PointType>::Ptr &cloud, const pcl::PointCloud<PointType>::Ptr &cloud2) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);

    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud, 0, 255, 0);
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
void Study_Filter::test_condition_removal(int argc, char **argv){
    srand (time (NULL));   //用系统时间初始化随机种子
    if (argc != 2) {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 100;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    if (strcmp(argv[1], "-r") == 0) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.4);
        outrem.setMinNeighborsInRadius(2);
        // apply filter
        outrem.filter(*cloud_filtered);
    } else if (strcmp(argv[1], "-c") == 0) {
        // build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                                      new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                                      new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(cloud);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter(*cloud_filtered);
    } else {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        return;
    }
    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                  << cloud->points[i].y << " "
                  << cloud->points[i].z << std::endl;
    // display pointcloud after filtering
    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " "
                  << cloud_filtered->points[i].y << " "
                  << cloud_filtered->points[i].z << std::endl;

    showPointClouds(cloud, cloud_filtered);
}

void Study_Filter::test_project_inliers(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    //创建点云并打印出来
    cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before projection: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                  << cloud->points[i].y << " "
                  << cloud->points[i].z << std::endl;

    // 填充ModelCoefficients的值,使用ax+by+cz+d=0*面模型，其中 a=b=d=0,c=1 也就是X——Y*面
    //定义模型系数对象，并填充对应的数据
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // 创建ProjectInliers对象，使用ModelCoefficients作为投影对象的模型参数
    pcl::ProjectInliers<pcl::PointXYZ> proj;        //创建投影滤波对象
    proj.setModelType (pcl::SACMODEL_PLANE);        //设置对象对应的投影模型
    proj.setInputCloud (cloud);                     //设置输入点云
    proj.setModelCoefficients (coefficients);       //设置模型对应的系数
    proj.filter (*cloud_projected);                 //投影结果存储

    std::cerr << "Cloud after projection: " << std::endl;
    for (size_t i = 0; i < cloud_projected->points.size (); ++i)
        std::cerr << "    " << cloud_projected->points[i].x << " "
                  << cloud_projected->points[i].y << " "
                  << cloud_projected->points[i].z << std::endl;

   // showPointClouds(cloud, cloud_projected);
}

//input: table_scene_lms400.pcd
void Study_Filter::test_extract_indices(){
    /**********************************************************************************************************
     从输入的.PCD 文件载入数据后，创建一个VOxelGrid滤波器对数据进行下采样，在这里进行下才样是为了加速处理过程，
     越少的点意味着分割循环中处理起来越快
     **********************************************************************************************************/

    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2),
            cloud_filtered_blob (new pcl::PCLPointCloud2);//申明滤波前后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
            cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
            cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // 读取PCD文件
    pcl::PCDReader reader;
    reader.read ("table_scene_lms400.pcd", *cloud_blob);
     //统计滤波前的点云个数
    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // 创建体素栅格下采样: 下采样的大小为1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //体素栅格下采样对象
    sor.setInputCloud (cloud_blob);             //原始点云
    sor.setLeafSize (0.01f, 0.01f, 0.01f);    // 设置采样体素大小
    sor.filter (*cloud_filtered_blob);        //保存

    // 转换为模板点云
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // 保存下采样后的点云
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled1.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());//模型系数
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());//点索引

    pcl::SACSegmentation<pcl::PointXYZ> seg;                //创建分割对象
    seg.setOptimizeCoefficients (true);                     //设置对估计模型参数进行优化处理

    seg.setModelType (pcl::SACMODEL_PLANE);                 //设置分割模型类别
    seg.setMethodType (pcl::SAC_RANSAC);                    //设置用哪个随机参数估计方法
    seg.setMaxIterations (1000);                            //设置最大迭代次数
    seg.setDistanceThreshold (0.01);                        //判断是否为模型内点的距离阀值

    // 设置ExtractIndices的实际参数
    pcl::ExtractIndices<pcl::PointXYZ> extract;        //创建点云提取对象

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
   // while (cloud_filtered->points.size () > 0.3 * nr_points)
    while (cloud_filtered->points.size () > 0.1 * nr_points)
    {
      // 为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the inliers
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);
      std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

      std::stringstream ss;
      ss << "table_scene_lms400_plane_" << i << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

      // Create the filtering object
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered.swap (cloud_f);
      i++;
    }
}

typedef pcl::PointXYZI P_XYZI;
static void showPCL(const pcl::PointCloud<P_XYZI>::Ptr &cloud, const pcl::PointCloud<P_XYZI>::Ptr &cloud2) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);

    pcl::visualization::PointCloudColorHandlerCustom<P_XYZI> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<P_XYZI>(cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    if(cloud2 != NULL){
        pcl::visualization::PointCloudColorHandlerCustom<P_XYZI> single_color2(cloud2, 255, 0, 0);
        viewer->addPointCloud<P_XYZI>(cloud2, single_color2, "sample cloud 2");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud 2");
    }

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}
void Study_Filter::test_bilateral_filter(){
    //PointXYZI-> I means intensity(强度)
    std::string file = "table_scene_lms400.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // 需要PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZI>(file, *cloud);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
    // Apply the filter
    pcl::BilateralFilter<pcl::PointXYZI> fbf;
    fbf.setInputCloud(cloud);
    fbf.setSearchMethod(tree1);
    fbf.setStdDev(0.1);
    fbf.setHalfSize(0.1);
    fbf.filter(*cloud_filtered);

    showPCL(cloud, cloud_filtered);
}
