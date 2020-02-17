/**
 *   P_Positioning.h
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
    class Positioning : public IPositioning
    {
    public:
         //设置相机参数
        virtual void setParams(const CameraParam &cam) 
        {
            mCamera = std::move(cam);
        }
        //添加单张
        virtual void addFrame(IFrame *pframe) 
        {
            mFrames.push_back(pframe);
        }
        //添加多张
        virtual void addFrames(const FrameVector &framedatas) 
        {
            //插入数据
            if(!framedatas.empty())
                std::copy(framedatas.begin(), framedatas.end(), 
                          std::back_inserter(mFrames));
        }
        //处理
        virtual void position() 
        {
            //添加具体实现
            assert(NULL);
        }
     protected:
        CameraParam   mCamera;
        FrameVector   mFrames;
    };

    //单张图片定位
    class SingleImgPositioning : public Positioning
    {
    public:
        //处理
        virtual void position();
    };

    //多图片定位
    class MultiImgPositioning : public Positioning
    {
    public:
        //处理
        virtual void position();
    };

    //深度估计定位
    class DepthLImgPositioning : public Positioning
    {
    public:
        //处理
        virtual void position() 
        {
            //添加具体实现
            assert(NULL);
        }
    };
}

#endif