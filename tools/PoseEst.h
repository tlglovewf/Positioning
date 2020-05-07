#ifndef _POSEEST_H_H_
#define _POSEEST_H_H_
#include "P_Interface.h"
using namespace Position;


//视觉定位器
class IVisualPositioner : public IBase
{
public:
    //定位目标
    virtual bool position(TrackerItem &item) = 0;

};

//一个目标对应一个batch  最简单的生成方式
class TargetBatchesGenerator : public IBatchesGenerator
{
public:
//生成批处理对象
    virtual PrjBatchVector generate(const std::shared_ptr<IData> &pdata,
                                    TrackerItemVector      &trackitems);
};

//批定位器
class BatchVisualPositioner : public IVisualPositioner
{
public:
    BatchVisualPositioner(const CameraParam &cam):mCamera(cam){}

    //定位目标
    virtual bool position(TrackerItem &target);

    //重置
    void reset(); 
protected:
    CameraParam mCamera;
};

#endif