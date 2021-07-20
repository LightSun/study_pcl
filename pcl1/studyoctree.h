#ifndef STUDYOCTREE_H
#define STUDYOCTREE_H


class StudyOctree
{
public:
    StudyOctree();

    static void test_compress();
    //体素搜索 https://pcl.readthedocs.io/projects/tutorials/en/latest/octree.html#octree-search
    static void test_octree_search();

    //无序点云数据集的空间变化检测(尤其是新增的数据)： https://www.cnblogs.com/li-yao7758258/p/6441595.html
    static void test_octree_search2();
};

#endif // STUDYOCTREE_H
