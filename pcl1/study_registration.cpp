#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

//#include <iostream>
#include <thread>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>                // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>  // 滤波文件头
#include <pcl/visualization/pcl_visualizer.h>

#include "study_registration.h"

using namespace std::chrono_literals;

void Study_Registration::test_Icp()
{
    // 定义输入和输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    // 随机填充无序点云
    cloud_in->width = 5;
    cloud_in->height = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
              << std::endl;
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
        std::cout << "    " <<
                     cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
                     cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;

    // 在点云上执行简单的刚性变换，将cloud_out中的x平移0.7f米，然后再次输出数据值。
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    // 打印这些点
    std::cout << "Transformed " << cloud_in->points.size() << " data points:"
              << std::endl;
    for (size_t i = 0; i < cloud_out->points.size(); ++i)
        std::cout << "    " << cloud_out->points[i].x << " " <<
                     cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

    // 创建IterativeClosestPoint的实例
    // setInputSource将cloud_in作为输入点云
    // setInputTarget将平移后的cloud_out作为目标点云
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
    // 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
    // （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
    // 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
    std::cout << matrix << std::endl;
}

void Study_Registration::test_ndt(){
    // Loading first scan of room.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("data-master/tutorials/room_scan1.pcd", *target_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return;
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

    // Loading second scan of room from new perspective.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("data-master/tutorials/room_scan2.pcd", *input_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
        return;
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;

    //将输入扫描过滤到原始尺寸的大约 10% 以提高配准速度。
    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);//体素大小。小立方体
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size()
              << " data points from room_scan2.pcd" << std::endl;

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    //NDT 算法使用一个体素化数据结构和More-Thuente 线搜索
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    //设置终止条件的最小变化差
    ndt.setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance). 体素网格
    ndt.setResolution(1.0);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(35);//最大迭代次数

    // Setting point cloud to be aligned.
    ndt.setInputSource(filtered_cloud);
    // Setting point cloud to be aligned to.
    //目标点云target_cloud不需要进行滤波处理，因为NDT算法在目标点云对应的体素Voxel网格数据结构计算时，
    //不使用单个点，而是使用体素的点。即已做了降采样处理
    ndt.setInputTarget(target_cloud);

    //设置使用机器人测距法得到的粗略初始变换矩阵
    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1.79387, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //求出input_cloud变换到target_cloud的变换矩阵
    ndt.align(*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

    // Saving transformed input cloud.
    pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr
            viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped()) {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// 首先从定义类型开始，以免使代码混乱
// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

//./alignment-prerejective ./data/chef.pcd ./data/rs1.pcd
void Study_Registration::test_robust(int argc, char ** argv){
    // 然后，我们实例化必要的数据容器，检查输入参数并加载对象和场景点云。
    // Point clouds
    PointCloudT::Ptr object (new PointCloudT);
    PointCloudT::Ptr object_aligned (new PointCloudT);
    PointCloudT::Ptr scene (new PointCloudT);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    // Get input object and scene
    if (argc != 3)
    {
        pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
        return;
    }

    // Load object and scene
    pcl::console::print_highlight ("Loading point clouds...\n");
    if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
            pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
    {
        pcl::console::print_error ("Error loading object/scene file!\n");
        return;
    }

    // 1, 降采样 Downsample
    // 为了加快处理速度，我们使用PCL的：pcl::VoxelGrid类将对象和场景点云的采样率下采样至5 mm。
    pcl::console::print_highlight ("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 0.005f;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (object);
    grid.filter (*object);
    grid.setInputCloud (scene);
    grid.filter (*scene);

    // Estimate normals for scene
    //2, 估计场景法线 Estimate normals for scene
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT,PointNT> nest;
    nest.setRadiusSearch (0.01);
    nest.setInputCloud (scene);
    nest.compute (*scene);

    // 3, 特征估计 Estimate features
    // 对于下采样点云中的每个点，我们现在使用PCL的pcl::FPFHEstimationOMP<>类来计算用于对齐过程中用于匹配的快速点特征直方图（FPFH）描述符。
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (0.025);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    fest.setInputCloud (scene);
    fest.setInputNormals (scene);
    fest.compute (*scene_features);

    // Perform alignment
    // SampleConsensusPrerejective 实现了有效的RANSAC姿势估计循环
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
    align.setInputSource (object);
    align.setSourceFeatures (object_features);
    align.setInputTarget (scene);
    align.setTargetFeatures (scene_features);
    align.setMaximumIterations (50000); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (5); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("Alignment");
        align.align (*object_aligned);
    }

    if (align.hasConverged ())
    {
        // Print results
        printf ("\n");
        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

        // Show alignment
        pcl::visualization::PCLVisualizer visu("Alignment");
        visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        visu.spin ();
    }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
        return;
    }
}

void Study_Registration::test_robust_default(){
    const char* in[3];
    in[0] = "test_robust.exe";
    in[1] = "D:/Program Files (x86)/PCL 1.9.1/share/doc/pcl-1.9/tutorials/sources/alignment_prerejective/chef.pcd";
    in[2] = "D:/Program Files (x86)/PCL 1.9.1/share/doc/pcl-1.9/tutorials/sources/alignment_prerejective/rs1.pcd";

    char** chs = (char**)in;
    test_robust(3, chs);
}

//-----------------------------------------------------
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT2;
typedef pcl::PointCloud<PointT2> PointCloudT2;

bool next_iteration = false;

static void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}
/**
 * 此函数是查看器的回调。 当查看器窗口位于顶部时，只要按任意键，就会调用此函数。 如果碰到“空格”； 将布尔值设置为true。
 * @param event
 * @param nothing
 */
static void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void*)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}

//./interactive_icp monkey.ply 1
void Study_Registration::test_interactive_icp(int argc, char** argv)
{
    // The point clouds we will be using
    PointCloudT2::Ptr cloud_in (new PointCloudT2);  // Original point cloud
    PointCloudT2::Ptr cloud_tr (new PointCloudT2);  // Transformed point cloud
    PointCloudT2::Ptr cloud_icp (new PointCloudT2);  // ICP output point cloud

//    我们检查程序的参数，设置初始ICP迭代的次数，然后尝试加载PLY文件。
    // Checking program arguments
    if (argc < 2)
    {
        printf ("Usage :\n");
        printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
        PCL_ERROR ("Provide one ply file.\n");
        return;
    }

    int iterations = 1;  // Default number of ICP iterations
    if (argc > 2)
    {
        // If the user passed the number of iteration as an argument
        iterations = atoi (argv[2]);
        if (iterations < 1)
        {
            PCL_ERROR ("Number of initial iterations must be >= 1\n");
            return;
        }
    }

    pcl::console::TicToc time;
    time.tic ();
    if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return;
    }
    std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

    // 我们使用刚性矩阵变换来变换原始点云。
    // cloud_in包含原始点云。
    // cloud_tr和cloud_icp包含平移/旋转的点云。
    // cloud_tr是我们将用于显示的备份（绿点云）。

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = M_PI / 8;  // The angle of rotation in radians
    transformation_matrix (0, 0) = std::cos (theta);
    transformation_matrix (0, 1) = -sin (theta);
    transformation_matrix (1, 0) = sin (theta);
    transformation_matrix (1, 1) = std::cos (theta);

    // A translation on Z axis (0.4 meters)
    transformation_matrix (2, 3) = 0.4;

    // Display in terminal the transformation matrix
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    print4x4Matrix (transformation_matrix);

    // Executing the transformation
    pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
    *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

    // 这是ICP对象的创建。 我们设置ICP算法的参数。
    // setMaximumIterations（iterations）设置要执行的初始迭代次数（默认值为1）。
    // 然后，我们将点云转换为cloud_icp。 第一次对齐后，我们将在下一次使用该ICP对象时（当用户按下“空格”时）将ICP最大迭代次数设置为1。

    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<PointT2, PointT2> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);
    icp.align (*cloud_icp);
    icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    // 检查ICP算法是否收敛； 否则退出程序。 如果返回true，我们将转换矩阵存储在4x4矩阵中，然后打印刚性矩阵转换。
    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        return;
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT2> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT2> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT2> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();

        // The user pressed "space" :
        if (next_iteration)
        {
            // The Iterative Closest Point algorithm
            time.tic ();
            // 如果用户按下键盘上的任意键，则会调用keyboardEventOccurred函数。 此功能检查键是否为“空格”。
            // 如果是，则全局布尔值next_iteration设置为true，从而允许查看器循环输入代码的下一部分：调用ICP对象以进行对齐。
            // 记住，我们已经配置了该对象输入/输出云，并且之前通过setMaximumIterations将最大迭代次数设置为1。

            icp.align (*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

            // 和以前一样，我们检查ICP是否收敛，如果不收敛，则退出程序。
            if (icp.hasConverged ())
            {
                // printf（“ 033 [11A”）; 在终端增加11行以覆盖显示的最后一个矩阵是一个小技巧。
                // 简而言之，它允许替换文本而不是编写新行； 使输出更具可读性。 我们增加迭代次数以更新可视化器中的文本值。
                printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());

                // 这意味着，如果您已经完成了10次迭代，则此函数返回矩阵以将点云从迭代10转换为11。
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;

                // 函数getFinalTransformation（）返回在迭代过程中完成的刚性矩阵转换（此处为1次迭代）。
                transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR ("\nICP has not converged.\n");
                return;
            }

            //这不是我们想要的。 如果我们将最后一个矩阵与新矩阵相乘，那么结果就是从开始到当前迭代的转换矩阵。
        }
        next_iteration = false;
    }
}










