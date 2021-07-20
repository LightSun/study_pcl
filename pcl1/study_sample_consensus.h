#ifndef STUDY_SAMPLE_CONSENSUS_H
#define STUDY_SAMPLE_CONSENSUS_H

/**
  抽样一致性算法
 * @brief The Study_Sample_Consensus class
 */
class Study_Sample_Consensus
{
public:
    Study_Sample_Consensus();

    static void test_random1(int argc, char ** argv);
    /**
     * 使用方法：
     *
     * random_sample_consensus     创建包含外部点的平面
     * random_sample_consensus -f  创建包含外部点的平面，并计算平面内部点
     *
     * random_sample_consensus -s  创建包含外部点的球体
     * random_sample_consensus -sf 创建包含外部点的球体，并计算球体内部点
     */
    static void test_random2(int argc, char ** argv);
};

#endif // STUDY_SAMPLE_CONSENSUS_H
