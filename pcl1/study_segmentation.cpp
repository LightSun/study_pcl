#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件

#include <pcl/visualization/pcl_visualizer.h>

//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include "study_segmentation.h"

typedef pcl::PointXYZ PointT;

void Study_Segmentation::test_1()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 填充点云
    cloud->width  = 15;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    // 生成数据，采用随机数填充点云的x,y坐标，都处于z为1的平面上
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1.0;
    }

    // 设置几个局外点，即重新设置几个点的z值，使其偏离z为1的平面
    cloud->points[0].z = 2.0;
    cloud->points[3].z = -2.0;
    cloud->points[6].z = 4.0;

    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;  //打印
    for (size_t i = 0; i < cloud->points.size (); ++i)
      std::cerr << "    " << cloud->points[i].x << " "
                          << cloud->points[i].y << " "
                          << cloud->points[i].z << std::endl;
    /**
         * 创建分割时所需要的模型系数对象 coefficients 及存储内点的点索引集合对象 inliers .
         * 这也是我们指定“阈值距离DistanceThreshold”的地方，该距离阈值确定点必须与模型有多远才能被视为离群点。
         * 这里距离阔值是 0.01m ,即只要点到 z=1 平面距离小于该阈值的点都作为内部点看待,而大于该阁值的则看做离群点。
         * 我们将使用RANSAC方法（`pcl::SAC_RANSAC`）作为可靠的估计器。因为RANSAC比较简单（其他强大的估算工具也以此为基础，并添加了其他更复杂的概念）。
         */
    //创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
    //分割平面，得到平面的法向量：
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients (true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType (pcl::SACMODEL_PLANE);   //设置模型类型
    seg.setMethodType (pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
    seg.setDistanceThreshold (0.01);    //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
                                         //表示点到估计模型的距离最大值，

    seg.setInputCloud (cloud);
    //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return;
    }
    //打印出平面模型
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (size_t i = 0; i < inliers->indices.size (); ++i)
      std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                 << cloud->points[inliers->indices[i]].y << " "
                                                 << cloud->points[inliers->indices[i]].z << std::endl;
}
/*
处理流程如下 :

1, 过滤掉远于 1. 5 m 的数据点
2, 估计每个点的表面法线
3, 分割出平面模型 （数据集中的桌面）并保存到磁盘中。
4, 分割圆出柱体模型（数据集中的杯子）并保存到磁盘中。
注意：由于数据中包含噪声，圆柱体模型并不十分严格 。
*/
void Study_Segmentation::test_cylinder_seg()
{
    // All the objects needed
    pcl::PCDReader reader;                          // PCD文件读取对象
    pcl::PassThrough<PointT> pass;                  // 直通滤波器
    pcl::NormalEstimation<PointT, pcl::Normal> ne;  // 法线估算对象

    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;   // 分割器
    pcl::PCDWriter writer;                                      // PCD文件写出对象
    pcl::ExtractIndices<PointT> extract;                        // 点提取对象
    pcl::ExtractIndices<pcl::Normal> extract_normals;           // 法线提取对象

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(
                new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // Read in the cloud data 读取点云数据
    reader.read("data-master/tutorials/table_scene_mug_stereo_textured.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points."
              << std::endl;
    writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    // 设置圆柱体分割对象参数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);   // 设置分割模型为圆柱体
    seg.setMethodType(pcl::SAC_RANSAC);         // 设置采用RANSAC算法进行参数估计
    seg.setNormalDistanceWeight(0.1);           // 设置表面法线权重系数
    seg.setMaxIterations(10000);                // 设置最大迭代次数10000
    seg.setDistanceThreshold(0.05);             // 设置内点到模型的最大距离 0.05m
    seg.setRadiusLimits(0, 0.1);                // 设置圆柱体的半径范围0 -> 0.1m
    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);
    if (cloud_cylinder->points.empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size()
                  << " data points." << std::endl;
        writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }
}

void Study_Segmentation::test_cluster_extraction(){
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(
                new pcl::PointCloud<pcl::PointXYZ>);
    //reader.read("data-master/tutorials/tabletop.pcd", *cloud);
   // reader.read("table_scene_lms400.pcd", *cloud);
    reader.read("F:/work/ply_data/source/RPC_BA32_3603_2013_rgb.pcd", *cloud);

    std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    // 执行降采样滤波，叶子大小1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points."
              << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    // 创建平面模型分割器并初始化参数
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    int i = 0, nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        // 移除剩余点云中最大的平面
        seg.setInputCloud(cloud_filtered);
        // 执行分割，将分割出来的平面点云索引保存到inliers中
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        // 从输入点云中取出平面内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        // 得到与平面相关的点cloud_plane
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points."
                  << std::endl;

        // Remove the planar inliers, extract the rest
        // 从点云中剔除这些平面内点，提取出剩下的点保存到cloud_f中，并重新赋值给cloud_filtered。
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    // 为提取算法的搜索方法创建一个KdTree对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    /**
         * 在这里，我们创建一个PointIndices的vector，该vector在vector <int>中包含实际的索引信息。
         * 每个检测到的簇的索引都保存在这里-请注意，cluster_indices是一个vector，包含多个检测到的簇的PointIndices的实例。
         * 因此，cluster_indices[0]包含我们点云中第一个 cluster(簇)的所有索引。
         *
         * 从点云中提取簇（集群）,并将点云索引保存在 cluster_indices 中。
         */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 设置临近搜索的搜索半径（搜索容差）为2cm
    ec.setMinClusterSize(100);    // 每个簇（集群）的最小大小
    ec.setMaxClusterSize(25000);  // 每个簇（集群）的最大大小
    ec.setSearchMethod(tree);     // 设置点云搜索算法
    ec.setInputCloud(cloud_filtered);   // 设置输入点云
    ec.extract(cluster_indices);        // 设置提取到的簇，将每个簇以索引的形式保存到cluster_indices;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // 为了从点云索引向量中分割出每个簇，必须迭代访问点云索引，
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {

        // 每次创建一个新的点云数据集，并且将所有当前簇的点写入到点云数据集中。
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        const std::vector<int> &indices = it->indices;

        for (std::vector<int>::const_iterator pit = indices.begin(); pit != indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
                  << std::endl;
        /*
                std::stringstream ss;
                ss << "cloud_cluster_" << j << ".pcd";
                writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //
            */
        std::stringstream ss;
        ss << "cloud_cluster_" << j;
        // Generate a random (bright) color
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> single_color(cloud_cluster);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, single_color, ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());

        j++;
    }
    std::cout << "cloud size: " << cluster_indices.size() << std::endl;

    viewer->addCoordinateSystem(0.5);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}
