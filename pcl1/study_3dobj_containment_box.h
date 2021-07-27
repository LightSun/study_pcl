#ifndef STUDY_3DOBJ_CONTAINMENT_BOX_H
#define STUDY_3DOBJ_CONTAINMENT_BOX_H

/**
  3d对象 包容盒
原理简述
包围体（包容盒）是一个简单的几何空间，里面包含着复杂形状的物体。为物体添加包围体的目的是
快速的进行碰撞检测或者进行精确的碰撞检测之前进行过滤（即当包围体碰撞，才进行精确碰撞检测和处理）。
包围体类型包括球体、轴对齐包围盒（AABB）、有向包围盒（OBB）、8-DOP以及凸壳（CONVEX HULL）。

--常见包容盒（ Bounding Volumes）分类：
包容球：
    SPHERE 用球体包围整个几何体，用于相交测试很方便，但是其紧密型差，周围空隙较大，当物体变形后，包围球需要重新计算。
    当对象进行旋转运动时，包围球不需要做任何更新，这是包围球的优势
    ，即当几何对象频繁进行旋转运动时，使用包围球效率较高。
AABB包容盒：
    Axially Aligned Bounding Box，3D环境下的AABB盒即一个六面体，每个边都平行于一个坐标平面，较简单，但紧密性较差，
    当物体旋转、形变之后需要对AABB进行更新。本身的长宽高根据物体大小而定。
OBB包容盒：
    Oriented Bounding Box，此方法紧密型较好，可以降低参与相交测试的包容盒数目，因此性能要优于AABB和包容球。
    当物体发生旋转，仅需对OBB进行相同的旋转即可，
    但是当物体形变后，更新OBB的代价较大，故不适用那些软体的对象。

详见图： res/3dobj_containment_box.jpg
 */
class Study_3dobj_containment_box
{
public:
    static void test_moment_of_inertia();
};

#endif // STUDY_3DOBJ_CONTAINMENT_BOX_H
