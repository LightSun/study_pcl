#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "testmain.h"
#include "studyoctree.h"
#include "study_sample_consensus.h"
#include "study_range_image.h"
#include "study_fliter.h"
#include "studyio.h"
#include "study_key_point.h"
#include "study_features.h"
#include "study_segmentation.h"
#include "study_registration.h"
#include "study_surface_reconstruction.h"

//based on pcl-1.9.1
//pcd资源 来自pcl官方和 pcl-learning(github)
int main(int argc, char ** argv)
{
    int result = 0;
   // TestMain::test_base1();
  //  result = TestMain::test_transform(argc, argv);
   // TestMain::test_kdtree();

   // StudyOctree::test_compress();
   // StudyOctree::test_octree_search2();

   // Study_Sample_Consensus::test_random1(argc, argv);
   // Study_Sample_Consensus::test_random2(argc, argv);

   // Study_range_image::test_create("bunny.pcd");
   // Study_range_image::test_extract_image_border(argc, argv);

   // StudyIO::stl2pcd("D:/360Downloads/pcl_files/71_final.stl", "D:/360Downloads/pcl_files/71_final_stl.pcd");
   // StudyIO::obj2pcd("D:/360Downloads/pcl_files/71_final.obj", "D:/360Downloads/pcl_files/71_final_obj.pcd");
   // StudyIO::pcd2las("D:/360Downloads/pcl_files/71_final_stl.pcd", "D:/360Downloads/pcl_files/71_final.las");
   // StudyIO::ply2pcd("F:/work/ply_data/source/RPC_BA32_3603_2013_rgb.ply", "F:/work/ply_data/source/RPC_BA32_3603_2013_rgb.pcd");

    // Study_Filter::test_pass_through();
    //Study_Filter::test_condition_removal(argc, argv);
    //Study_Filter::test_extract_indices();
   // Study_Filter::test_bilateral_filter();

    //Study_Key_Point::test_narf_extract(argc, argv);

   // std::string file = "D:/tools/pcl_runtimes/data-master/tutorials/table_scene_lms400.pcd";
   // Study_Features::test_1(file, "D:/tools/pcl_runtimes/table_scene_lms400_f1.pcd");
   // Study_Features::test_2(file, "D:/tools/pcl_runtimes/table_scene_lms400_f2.pcd");
   // Study_Features::test_3(file, "D:/tools/pcl_runtimes/table_scene_lms400_downsampled.pcd",
    //"D:/tools/pcl_runtimes/table_scene_lms400_f3.pcd");

  //  Study_Features::test_fth("D:/tools/pcl_runtimes/data-master/tutorials/ism_test_cat.pcd");
   // Study_Features::test_4();
    //Study_Features::test_normal_estimation(argc, argv);
   // Study_Features::test_pfh_estimation();

     // Study_Segmentation::test_1();
     // Study_Segmentation::test_cylinder_seg();
      Study_Segmentation::test_cluster_extraction();

  //  Study_Registration::test_Icp();
   // Study_Registration::test_ndt();
   // Study_Registration::test_robust_default();
   // Study_Registration::test_interactive_icp(argc, argv);

   // Study_Surface_reconstruction::test_resample();

    system("pause");
    return (result);
}
