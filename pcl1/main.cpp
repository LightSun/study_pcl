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

//based on pcl-1.9.1
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

    // Study_Filter::test_pass_through();
    //Study_Filter::test_condition_removal(argc, argv);

    Study_Key_Point::test_narf_extract(argc, argv);

    system("pause");
    return (result);
}
