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
        //构造
        Positioning(const CameraParam &cam):mCamera(cam)
        {

        }
        //定位关键帧中目标
        virtual void position(IKeyFrame *frame)
        {
            assert(NULL);
        }

         //定位关键点
        virtual Mat position(const Mat &R, const Mat &t, const Point2f &pt1, const Point2f &pt2)
        {
            assert(NULL);
        }
        //定位场景地图关键帧中目标
        virtual void position(const std::shared_ptr<IMap> &pMap) 
        {
            assert(NULL);
        }
        //获取极线
        virtual EpLine computeEpLine(const cv::Mat &R, const cv::Mat &t,const cv::Point2f &pt) ;

        //极线匹配(基于目标包围盒)
        virtual TargetData eplineMatch(const EpLine &epline,const TargetData &item, const TargetVector &targets);
        
        //极线匹配(基于块)
        virtual Point2f eplineMatch(const EpLine &epline, const FrameData &preframe, const FrameData &curframe,const Point2f &pt);

        //反投
        virtual cv::Point2f backProject(const FrameData &frame,const BLHCoordinate &blh, cv::Mat &outimg);
     protected:
        CameraParam   mCamera;
    };

    //多图片定位
    class MultiImgPositioning : public Positioning
    {
    public:
        MultiImgPositioning(const CameraParam &cam):Positioning(cam){}
        //定位关键帧中目标
        virtual void position(IKeyFrame *frame);
         //定位场景地图关键帧中目标
        virtual void position(const std::shared_ptr<IMap> &pMap);
        //定位关键点
        virtual Mat position(const Mat &R, const Mat &t, const Point2f &pt1, const Point2f &pt2);
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