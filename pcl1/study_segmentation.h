#ifndef STUDY_SEGMENTATION_H
#define STUDY_SEGMENTATION_H

/**
  点云分割
校准方法是（参考）：用PCL中基于RANSAC的平面检测方法检测出平面，得到平面：ax+by+cz+d=0。
对于一个平面，上式中xyz的系数，就是它的法向量。然后，雷达坐标系中的竖直向量是（0,0,1），
计算出从平面法向量旋转到竖直向量的旋转矩阵，再把此旋转矩阵应用到点云，点云即可得到旋转。
ps: https://blog.csdn.net/ethan_guo/article/details/80683181
```
pcl::transformPointCloud(*cloud_in, *cloud_final, rotation);
```
 */
class Study_Segmentation
{
public:
    static void test_1();

    static void test_2();
};

#endif // STUDY_SEGMENTATION_H
