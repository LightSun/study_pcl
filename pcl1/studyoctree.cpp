#include <pcl/point_cloud.h>                //点云类型
#include <pcl/point_types.h>                //点数据类型
#include <pcl/io/openni2_grabber.h>          //点云获取接口类
#include <pcl/visualization/cloud_viewer.h> //点云可视化类

#include <pcl/compression/octree_pointcloud_compression.h> //点云压缩类
#include <pcl/octree/octree.h>                              //八叉树头文件

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
#define sleep(x) Sleep((x)*1000)
#endif

#include "studyoctree.h"

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer() : viewer(" Point Cloud Compression Example")
    {
    }
    /************************************************************************************************
  在OpenNIGrabber采集循环执行的回调函数cloud_cb_中，首先把获取的点云压缩到stringstream缓冲区，下一步就是解压缩，
  它对压缩了的二进制数据进行解码，存储在新的点云中解码了点云被发送到点云可视化对象中进行实时可视化
*************************************************************************************************/

    void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
        {
            // 存储压缩点云的字符流对象
            std::stringstream compressedData;
            // 存储输出点云
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

            // 压缩点云
            PointCloudEncoder->encodePointCloud(cloud, compressedData);

            // 解压缩点云
            PointCloudDecoder->decodePointCloud(compressedData, cloudOut);

            // 可视化解压缩的点云
            viewer.showCloud(cloudOut);
        }
    }
    /**************************************************************************************************************
 在函数中创建PointCloudCompression类的对象来编码和解码，这些对象把压缩配置文件作为配置压缩算法的参数
 所提供的压缩配置文件为OpenNI兼容设备采集到的点云预先确定的通用参数集，本例中使用MED_RES_ONLINE_COMPRESSION_WITH_COLOR
 配置参数集，用于快速在线的压缩，压缩配置方法可以在文件/io/include/pcl/compression/compression_profiles.h中找到，
  在PointCloudCompression构造函数中使用MANUAL——CONFIGURATION属性就可以手动的配置压缩算法的全部参数
******************************************************************************************************************/
    void run()
    {

        bool showStatistics = true; //设置在标准设备上输出打印出压缩结果信息

        // 压缩选项详情在: /io/include/pcl/compression/compression_profiles.h
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

        // 初始化压缩和解压缩对象  其中压缩对象需要设定压缩参数选项，解压缩按照数据源自行判断
        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
        /***********************************************************************************************************
    下面的代码为OpenNI兼容设备实例化一个新的采样器，并且启动循环回调接口，每从设备获取一帧数据就回调函数一次，，这里的
    回调函数就是实现数据压缩和可视化解压缩结果。
   ************************************************************************************************************/
        //创建从OpenNI获取点云的抓取对象
        pcl::Grabber *interface = new pcl::io::OpenNI2Grabber(); //pcl::OpenNI2Grabber

        // 建立回调函数
        boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

        //建立回调函数和回调信息的绑定
        boost::signals2::connection c = interface->registerCallback(f);

        // 开始接受点云的数据流
        interface->start();

        while (!viewer.wasStopped())
        {
            sleep(1);
        }

        interface->stop();

        // 删除压缩与解压缩的实例
        delete (PointCloudEncoder);
        delete (PointCloudDecoder);
    }

    pcl::visualization::CloudViewer viewer;

    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudDecoder;
};

StudyOctree::StudyOctree()
{

}

void StudyOctree::test_compress(){
    SimpleOpenNIViewer v;
    v.run();
}

// https://pcl.readthedocs.io/projects/tutorials/en/latest/octree.html#octree-search
void StudyOctree::test_octree_search(){
    srand((unsigned int)time(NULL)); //用系统时间初始化随机种子与 srand (time (NULL))的区别

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建点云数据
    cloud->width = 1000;
    cloud->height = 1; //无序
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) //随机循环产生点云的坐标值
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
    /****************************************************************************
      创建一个octree实例，用设置分辨率进行初始化，该octree用它的页节点存放点索引向量，
      分辨率参数描述最低一级octree的最小体素的尺寸，因此octree的深度是分辨率和点云空间维度
      的函数，如果知道点云的边界框，应该用defineBoundingbox方法把它分配给octree然后通过
      点云指针把所有点增加到ctree中。
      *****************************************************************************/
    float resolution = 128.0f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution); //初始化Octree

    octree.setInputCloud(cloud);      //设置输入点云    这两句是最关键的建立PointCloud和octree之间的联系
    octree.addPointsFromInputCloud(); //构建octree

    pcl::PointXYZ searchPoint; //设置searchPoint

    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    /*************************************************************************************
     一旦PointCloud和octree联系一起，就能进行搜索操作，这里使用的是“体素近邻搜索”，把查询点所在体素中
      其他点的索引作为查询结果返回，结果以点索引向量的形式保存，因此搜索点和搜索结果之间的距离取决于octree的分辨率参数
   *****************************************************************************************/

    std::vector<int> pointIdxVec; //存储-体素-近邻搜索结果向量

    //体素搜索
    if (octree.voxelSearch(searchPoint, pointIdxVec)) //执行搜索，返回结果到pointIdxVec
    {
        std::cout << "Neighbors within voxel search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z << ")"
                  << std::endl;

        for (size_t i = 0; i < pointIdxVec.size(); ++i) //打印结果点坐标
            std::cout << "    " << cloud->points[pointIdxVec[i]].x
                      << " " << cloud->points[pointIdxVec[i]].y
                      << " " << cloud->points[pointIdxVec[i]].z << std::endl;
    }

    /**********************************************************************************
     K 被设置为10 ，K近邻搜索  方法把搜索结果写到两个分开的向量，第一个pointIdxNKNSearch包含搜索结果
      （结果点的索引的向量）  第二个向量pointNKNSquaredDistance存储搜索点与近邻之间的距离的平方。
   *************************************************************************************/

    //K 近邻搜索
    int K = 10;

    std::vector<int> pointIdxNKNSearch;         //结果点的索引的向量
    std::vector<float> pointNKNSquaredDistance; //搜索点与近邻之间的距离的平方

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
                      << " " << cloud->points[pointIdxNKNSearch[i]].y
                      << " " << cloud->points[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    // 半径内近邻搜索

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;

    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
                      << " " << cloud->points[pointIdxRadiusSearch[i]].y
                      << " " << cloud->points[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }
}

void StudyOctree::test_octree_search2(){
    srand((unsigned int)time(NULL)); //用系统时间初始化随机种子

    // 八叉树的分辨率，即体素的大小
    float resolution = 32.0f;

    // 初始化空间变化检测对象
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>); //创建点云实例cloudA生成的点云数据用于建立八叉树octree对象

    // 为cloudA点云填充点数据
    cloudA->width = 128;                                   //设置点云cloudA的点数
    cloudA->height = 1;                                    //无序点
    cloudA->points.resize(cloudA->width * cloudA->height); //总数

    for (size_t i = 0; i < cloudA->points.size(); ++i) //循环填充
    {
        cloudA->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudA->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudA->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // 添加点云到八叉树中，构建八叉树
    octree.setInputCloud(cloudA);     //设置输入点云
    octree.addPointsFromInputCloud(); //从输入点云构建八叉树
    /***********************************************************************************
       点云cloudA是参考点云用其建立的八叉树对象描述它的空间分布，octreePointCloudChangeDetector
       类继承自Octree2BufBae类，Octree2BufBae类允许同时在内存中保存和管理两个octree，另外它应用了内存池
       该机制能够重新利用已经分配了的节点对象，因此减少了在生成点云八叉树对象时昂贵的内存分配和释放操作
       通过访问 octree.switchBuffers ()重置八叉树 octree对象的缓冲区，但把之前的octree数据仍然保留在内存中
      ************************************************************************************/
    // 交换八叉树的缓冲，但是CloudA对应的八叉树结构仍然在内存中
    octree.switchBuffers();
    //cloudB点云用于建立新的八叉树结构，与前一个cloudA对应的八叉树共享octree对象，同时在内存中驻留
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>); //实例化点云对象cloudB

    // 为cloudB创建点云
    cloudB->width = 128;
    cloudB->height = 1;
    cloudB->points.resize(cloudB->width * cloudB->height);

    for (size_t i = 0; i < cloudB->points.size(); ++i)
    {
        cloudB->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudB->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudB->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // 添加cloudB到八叉树中
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();

    /**************************************************************************************************************
     为了检索获取存在于couodB的点集R，此R并没有cloudA中的元素，可以调用getPointIndicesFromNewVoxels方法，通过探测两个八叉树之间
     体素的不同，它返回cloudB 中新加点的索引的向量，通过索引向量可以获取R点集  很明显这样就探测了cloudB相对于cloudA变化的点集，但是只能探测
     到在cloudA上增加的点集，不能探测减少的
   ****************************************************************************************************************/

    std::vector<int> newPointIdxVector; //存储新添加的索引的向量

    // 获取前一cloudA对应八叉树在cloudB对应在八叉树中没有的点集
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    // 打印点集
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (size_t i = 0; i < newPointIdxVector.size(); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i]
                     << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
                     << cloudB->points[newPointIdxVector[i]].y << " "
                     << cloudB->points[newPointIdxVector[i]].z << std::endl;
}
