#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkSmartPointer.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "studyio.h"

void StudyIO::stl2pcd(const std::string& stdPath, const std::string& savePcdPath){
    //read cad-module
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(stdPath.c_str());
    reader->Update();
    //to ply
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData = reader->GetOutput();
    polyData->GetNumberOfPoints();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    //ply to pcd
    pcl::io::vtkPolyDataToPointCloud(polyData, *cloud);
    pcl::io::savePCDFileASCII(savePcdPath, *cloud);
    //show
    pcl::visualization::PCLVisualizer viewer("stl2pcd");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud, "input_cloud");

    while (!viewer.wasStopped ())  //untill q is pressed
    {
        viewer.spinOnce ();
    }
}
void StudyIO::obj2pcd(const std::string& objPath, const std::string& savePcdPath){
    //https://www.pianshen.com/article/5961375675/
    //read
   // pcl::TextureMesh mesh;
   // pcl::io::loadOBJFile(objPath, mesh);
   // pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadOBJFile(objPath, *cloud);

    //obj to pcd
   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   // pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    pcl::io::savePCDFileASCII(savePcdPath, *cloud);
    //show
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer(new pcl::visualization::PCLVisualizer("3D viewer A"));

    viewer->addPointCloud(cloud, "input_cloud");

    while (!viewer->wasStopped ())  //until q
    {
        viewer->spinOnce ();
    }
}
