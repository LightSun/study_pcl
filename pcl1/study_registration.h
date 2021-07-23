#ifndef STUDY_REGISTRATION_H
#define STUDY_REGISTRATION_H

/**
  原理： https://robot.czxy.com/docs/pcl/chapter03/registration_intro/
 */
class Study_Registration
{
public:

    //迭代最近点算法(ICP): Iterative Closest Point
    static void test_Icp();

    //正太分布变换配准
    static void test_ndt();

    //刚性物体的鲁棒姿态估计
    //Robust pose estimation of rigid objects
    //在具有杂波和遮挡的场景中找到刚体的对齐姿势
    static void test_robust(int argc, char ** argv);

    static void test_robust_default();

    /* 配准之交互式icp: https://robot.czxy.com/docs/pcl/chapter03/registration_interactive_icp/ */
    static void test_interactive_icp(int argc, char ** argv);
};

#endif // STUDY_REGISTRATION_H
