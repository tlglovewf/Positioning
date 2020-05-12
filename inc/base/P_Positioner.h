/**
 *   P_Positioner.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __POSITIONING_H_H_
#define __POSITIONING_H_H_
#include "P_Interface.h"

namespace Position
{
    //定位基础类
    class TargetPositioner : public ITargetPositioner
    {
    public:
        //构造
        TargetPositioner(const CameraParam &cam):mCamera(cam)
        {

        }

         //定位
        virtual bool position(KeyFrameVector &frame);    

        //极线匹配(基于目标包围盒) 返回序号
        virtual int eplineMatch(const EpLine &epline,const TargetData &item, const TargetVector &targets);
        
        //极线匹配(基于块)
        virtual Point2f eplineMatch(const EpLine &epline, const FrameData &preframe, const FrameData &curframe,const Point2f &pt);

     protected:
        CameraParam   mCamera;
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
        //选择用于量测的帧
        virtual void selectFrame(const TrackerItem &item,int &idx1, int &idex2);

    protected:
        CameraParam mCamera;
    };
}

#endif