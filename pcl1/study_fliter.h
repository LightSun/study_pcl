#ifndef STUDY_FLITER_H
#define STUDY_FLITER_H

#include <string>

class Study_Filter
{
public:
    //直通滤波: https://robot.czxy.com/docs/pcl/chapter02/filtering/
    static void test_pass_through();
    /***
     * 通过体素网格实现降采样，可以减少点数量的同时，保证点云的形状特征，可以提高配准、曲面重建、形状识别等算法的速度，并保证准确性。
     * https://robot.czxy.com/docs/pcl/chapter02/filtering/
     */
    static void test_voxel_grid(const std::string& in_pcd, const std::string& out_downsample_pcd_file);

    //统计学离群点移除过滤器: 移除噪点
    //https://pcl-tutorials.readthedocs.io/en/latest/statistical_outlier.html#statistical-outlier-removal
    static void test_statistical_removal(const std::string& in_pcd, const std::string& out_dir);

    //半径离群值滤波: https://pcl-tutorials.readthedocs.io/en/latest/remove_outliers.html#remove-outliers
    static void test_condition_removal(int argc, char **argv);

    /*
    使用参数化模型投影点云
    如何将点投影到一个参数化模型上（*面或者球体等），参数化模型通过一组参数来设定，对于*面来说使用其等式形式.在PCL中有特意存储常见模型系数的数据结构
     */
    static void test_project_inliers();

    /*
从一个点云中提取索引. (物体分割)
      如何使用一个，基于某一分割算法提取点云中的一个子集: https://www.cnblogs.com/li-yao7758258/p/6473304.html
*/
    static void test_extract_indices();

    /*
双边滤波: https://github.com/MNewBie/PCL-Notes/blob/master/chapter8.md
非常耗时
*/
    static void test_bilateral_filter();
};

#endif // STUDY_FLITER_H
