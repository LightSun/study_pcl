#ifndef STUDY_RANGE_IMAGE_H
#define STUDY_RANGE_IMAGE_H

#include <string>
/**
  深度图
 * @brief The Study_range_image class
 */
class Study_range_image
{
public:

/**
  创建深度图
https://www.cnblogs.com/li-yao7758258/p/6474699.html
http://robot.czxy.com/docs/pcl/chapter02/range_image/#_1  推荐
*/
    static void test_create(const std::string& pcdPath);

    /**
 @Description:  如何从深度图像中提取边界
http://robot.czxy.com/docs/pcl/chapter02/range_image/#_5
https://www.cnblogs.com/li-yao7758258/p/6476046.html
     */
    static void test_extract_image_border(int argc, char** argv);
};

#endif // STUDY_RANGE_IMAGE_H
