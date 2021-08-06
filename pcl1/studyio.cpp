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


#include <liblas/liblas.hpp>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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
void StudyIO::las2pcd(const std::string& inPath, const std::string& outPath){
    //打开las文件
    std::ifstream ifs;
    ifs.open(inPath, std::ios::in | std::ios::binary);
    if (!ifs.is_open())
    {
        std::cout << "无法打开.las" << std::endl;
        return;
    }
    liblas::ReaderFactory readerFactory;
    liblas::Reader reader = readerFactory.CreateWithStream(ifs);
    //写点云
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOutput(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloudOutput->clear();
    while (reader.ReadNextPoint())
    {
        double x = reader.GetPoint().GetX();
        double y = reader.GetPoint().GetY();
        double z = reader.GetPoint().GetZ();
        uint16_t red = reader.GetPoint().GetColor()[0];
        uint16_t green = reader.GetPoint().GetColor()[1];
        uint16_t blue = reader.GetPoint().GetColor()[2];

        /*****颜色说明
           *   uint16_t  范围为0-256*256 ；
           *   pcl 中 R  G  B利用的unsigned char  0-256；
           *   因此这里将uint16_t 除以256  得到  三位数的0-256的值
           *   从而进行rgba 值32位 的位运算。
           *
           ******/

        pcl::PointXYZRGBA thePt;  //int rgba = 255<<24 | ((int)r) << 16 | ((int)g) << 8 | ((int)b);
        thePt.x = x; thePt.y = y; thePt.z = z;
        thePt.rgba = (uint32_t)255 << 24 | (uint32_t)(red / 256) << 16 | (uint32_t)(green / 256) << 8 | (uint32_t)(blue / 256);
        //uint32_t rgb = *reinterpret_cast<int*>(&thePt.rgb);  //reinterpret_cast 强制转换
        cloudOutput->push_back(thePt);
    }

    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //viewer->setBackgroundColor(0, 0, 0); //设置背景
    //viewer->addPointCloud(cloudOutput,"cloudssd");
    //while (!viewer->wasStopped()){
    //    viewer->spinOnce();
    //}
    pcl::io::savePCDFileASCII(outPath, *cloudOutput);
    cloudOutput->clear();
}

void StudyIO::pcd2las(const std::string& inPath, const std::string& outPath){
    std::ofstream ofs(outPath.c_str(), ios::out | ios::binary);
    if (!ofs.is_open())
       {
           std::cout << "err  to  open  file  las....." << std::endl;
           return;
       }
       liblas::Header header;
       header.SetVersionMajor(1);
       header.SetVersionMinor(2);
       header.SetDataFormatId(liblas::ePointFormat3);
       header.SetScale(0.001, 0.001, 0.001);

       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
       pcl::io::loadPCDFile(inPath, *cloud);
       std::cout << "total:" << cloud->points.size() << std::endl;
       //写liblas,
       liblas::Writer writer(ofs, header);
       liblas::Point point(&header);

       for (size_t i = 0; i < cloud->points.size(); i++)
       {
           double x = cloud->points[i].x;
           double y = cloud->points[i].y;
           double z = cloud->points[i].z;
           point.SetCoordinates(x, y, z);

   //        uint32_t red = (uint32_t)cloud->points[i].r;
   //        uint32_t green = (uint32_t)cloud->points[i].g;
   //        uint32_t blue = (uint32_t)cloud->points[i].b;
   //        liblas::Color pointColor(red, green, blue);
   //        point.SetColor(pointColor);
           writer.WritePoint(point);
           //std::cout << x << "," << y << "," << z << std::endl;
       }
       double minPt[3] = { 9999999, 9999999, 9999999};
       double maxPt[3] = { 0, 0, 0 };
       header.SetPointRecordsCount(cloud->points.size());
       header.SetPointRecordsByReturnCount(0, cloud->points.size());
       header.SetMax(maxPt[0], maxPt[1], maxPt[2]);
       header.SetMin(minPt[0], minPt[1], minPt[2]);
       writer.SetHeader(header);
}
