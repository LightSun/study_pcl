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
};

#endif // STUDY_FLITER_H
