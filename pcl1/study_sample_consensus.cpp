#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "study_sample_consensus.h"

using namespace std::literals::chrono_literals;

static boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

static pcl::visualization::PCLVisualizer::Ptr simpleVis2(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                         pcl::PointCloud<pcl::PointXYZ>::ConstPtr final = nullptr) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    //像素点大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    if (final != nullptr) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(final, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(final, color_handler, "final cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "final cloud");
    }

    viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters();
    return (viewer);
}

Study_Sample_Consensus::Study_Sample_Consensus()
{

}

void Study_Sample_Consensus::test_random1(int argc, char ** argv){
    // 初始化点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  //存储源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);   //存储提取的局内点

    // 填充点云数据
    cloud->width    = 500;                   //填充点云数目
    cloud->height   = 1;                     //无序点云
    cloud->is_dense = false;                 //是否密集的
    cloud->points.resize (cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
        {
            //根据命令行参数用x^2+y^2+Z^2=1设置一部分点云数据，此时点云组成1/4个球体作为内点
            cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
            cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
            if (i % 5 == 0) // 500/5 = 100    // 可能会散落在球体外
                cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);   //此处对应的点为局外点
            else if(i % 2 == 0) // (500-100) / 2  ,// 在球体正方向内
                cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                            - (cloud->points[i].y * cloud->points[i].y));
            else// 在球体负方向内
                cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                              - (cloud->points[i].y * cloud->points[i].y));
        }
        else
        { //用x+y+z=1设置一部分点云数据，此时地拿云组成的菱形平面作为内点
            cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
            cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
            if( i % 2 == 0)
                cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);   //对应的局外点
            else
                cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
        }
    }

    std::vector<int> inliers;  //存储局内点集合的点的索引的向量

    //创建随机采样一致性对象
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
            model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));    //针对球模型的对象
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));   //针对平面模型的对象
    if(pcl::console::find_argument (argc, argv, "-f") >= 0)
    {  //根据命令行参数，来随机估算对应平面模型，并存储估计的局内点
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        ransac.setDistanceThreshold (.01);           //与平面距离小于0.01 的点称为局内点考虑，注意局内点设置参数
        ransac.computeModel();                       //执行随机参数估计
        ransac.getInliers(inliers);                  //存储估计所得的局内点
    }
    else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
    {
        //根据命令行参数  来随机估算对应的圆球模型，存储估计的内点
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
        ransac.setDistanceThreshold (.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

    // 复制估算模型的所有的局内点到final中
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    std::cout << *final << std::endl;

    // 创建可视化对象并加入原始点云或者所有的局内点

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
        viewer = simpleVis(final);
    else
        viewer = simpleVis(cloud);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
   // return 0;
}
void Study_Sample_Consensus::test_random2(int argc, char ** argv){
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

    // populate our PointCloud with points
    cloud->width = 500;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        if (pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0) {  // -s   -sf
            cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
            if (i % 5 == 0)     // 可能会散落在球体外
                cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
            else if (i % 2 == 0)// 在球体正方向内
                cloud->points[i].z = sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                                          - (cloud->points[i].y * cloud->points[i].y));
            else // 在球体负方向内
                cloud->points[i].z = -sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                                           - (cloud->points[i].y * cloud->points[i].y));
        } else {
            cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
            if (i % 2 == 0)
                cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
            else
                cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
        }
    }

    std::vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model
    // 球形
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
            model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    // 平面
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    if (pcl::console::find_argument(argc, argv, "-f") >= 0) {  // -f
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    } else if (pcl::console::find_argument(argc, argv, "-sf") >= 0) {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

    // copies all inliers of the model computed to another PointCloudqq
    // 将cloud中指定索引的点拷贝到final点云中
    pcl::copyPointCloud(*cloud, inliers, *final);

    // creates the visualization object and adds either our original cloud or all of the inliers
    // depending on the command line arguments specified.
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
        viewer = simpleVis2(cloud, final);
    else
        viewer = simpleVis2(cloud);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}
