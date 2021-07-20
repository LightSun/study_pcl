#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "testmain.h"
#include "studyoctree.h"
#include "study_sample_consensus.h"

int main(int argc, char ** argv)
{
    int result = 0;
   // TestMain::test_base1();
  //  result = TestMain::test_transform(argc, argv);
   // TestMain::test_kdtree();

   // StudyOctree::test_compress();
   // StudyOctree::test_octree_search2();
   // Study_Sample_Consensus::test_random1(argc, argv);
    Study_Sample_Consensus::test_random2(argc, argv);


    system("pause");
    return (result);
}
