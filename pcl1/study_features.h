#ifndef STUDY_FEATURES_H
#define STUDY_FEATURES_H

#include <string>

/**
  点特征描述 + 提取
  描述表面几何形状的一个重要问题是首先推断其在坐标系中的方向，即 [估算其法线]
  鲁棒性：是Robust的音译，也就是健壮和强壮的意思
 */
class Study_Features
{
public:
    static void test_1(const std::string& pcdFile, const std::string& savePath);
    static void test_2(const std::string& pcdFile, const std::string& savePath);
    static void test_3(const std::string& pcdFile, const std::string& downsample_pcdFile, const std::string& savePath);

    //https://blog.csdn.net/shixin_0125/article/details/104432255
    //https://www.freesion.com/article/5364605568/
    static void test_fth(const std::string& pcdFile);

    //NormalEstimation + KdTree
    //法向量估计(运行耗时5min): https://www.cnblogs.com/li-yao7758258/p/6479255.html
    static void test_4();

    //估计一个点云的表面法线
    /*表面法线是几何体表面一个十分重要的属性，例如：在进行光照渲染时产生符合可视习惯的效果时需要表面法线的信息才能正常进行，对于一个已经已经知道的几何体表面，根据垂直于点表面的的矢量，因此推推处表面某一点的法线方向比较容易，然而由于我们获取的点云的数据集在真实的物体的表面表现为一组定点的样本，这样就会有两种方法解决：

    1 . 使用曲面重建技术，从获取的点云数据中得到采样点对应的曲面，然后从曲面模型中计算出表面法线

    2. 直接从点云数据中近似推断表面法线

    在确定表面一点法线的问题近似于估计表面的一个相切面法线的问题，因此转换过来就是求一个最小二乘法平面拟合的问题

    （3）使用积分图进行法线估计

    使用积分图计算一个有序的点云的法线，注意此方法只适用有序点云

    代码解析normal_estimation_using_integral_images.cpp */
    static void test_normal_estimation(int argc, char ** argv);

    //PFH点特征直方图描述子
    /*
表面法线和曲率估计是某个点周围的几何特征的基本表示方法。虽然计算起来比较容易，但是能够提供的信息并不多。
因为他们只是用很少的几个参数值近似地表示一个点的k邻域几何特征。
大部分场景下都会有很多相同或相似的特征值，故而只采用点特征就会少了很多全局的特征信息。
我们可以通过点特征直方图（Point Feature Histograms， PFH）来采集全局的特征信息。
*/
    //https://robot.czxy.com/docs/pcl/chapter02/descriptor/
    static void test_pfh_estimation();
};

#endif // STUDY_FEATURES_H
