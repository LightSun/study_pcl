#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>

#include "study_surface_reconstruction.h"

void Study_Surface_reconstruction::test_resample()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    pcl::io::loadPCDFile("bun0.pcd", *cloud);

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    //移动最小二乘
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    // Reconstruct
    mls.process(mls_points);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");;
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }
    // Save output
    pcl::io::savePCDFile("bun0-mls.pcd", mls_points);
}
