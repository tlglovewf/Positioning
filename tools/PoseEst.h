#ifndef _POSEEST_H_H_

#include "P_Interface.h"
using namespace Position;


//位姿估算器
class IPoseEstimator : public IBase
{
public:
    //根据批处理结果 估算位姿态
    virtual bool poseEstimate( PrjBatchVector &prjbatch) = 0;
};

//视觉定位器
class IVisualPositioner : public IBase
{
public:
    //定位目标
    virtual bool position(TrackerItem &item,const PrjBatchVector &batches) = 0;

};


//一个目标对应一个batch  最简单的生成方式
class TargetBatchesGenerator : public IBatchesGenerator
{
public:
//生成批处理对象
    virtual PrjBatchVector generate(const std::shared_ptr<IData> &pdata,
                                    const TrackerItemVector      &trackitems);
};

//批位姿估计器, 估算每个批的位姿,都是相对于初始化成功的第一帧率
class BatchPoseEstimator : public IPoseEstimator
{
public:
    //根据批处理结果 估算位姿态
    virtual bool poseEstimate( PrjBatchVector &prjbatch);
};

//批定位器
class BatchVisualPositioner : public IVisualPositioner
{
public:
    //定位目标
    virtual bool position(TrackerItem &item,const PrjBatchVector &batches);

};

#endif