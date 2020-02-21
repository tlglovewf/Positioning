
/**
 *   P_Map.h
 *   
 *   add by tu li gen   2020.2.19
 * 
 */

#ifndef __PMAP_H_H_
#define __PMAP_H_H_
#include "P_Types.h"
namespace Position
{
    class PMap;

    class PMapNode
    {
    public:
          //设置位置
        void setPose(const cv::Mat &pose)
        {
            mPose = pose;
        }
        //获取位置
        cv::Mat getPose()const
        {
            return mPose;
        }

    protected:
        cv::Mat mPose;
        PMap   *mpMap;
    };

     //地图点
    class PMapPoint : public PMapNode
    {
    public:

    protected:
        cv::Mat     mPose;
    };

    //关键帧
    class PKeyFrame : public PMapNode
    {
    public:
    
    };

    //地图对象
    class PMap
    {
    public:
        typedef std::set<PMapPoint*>    MapPtSet;
        typedef MapPtSet::iterator      MapPtSetIter;

        typedef std::set<KeyFrame*>     KeyFmSet;
        typedef KeyFmSet::iterator      KeyFmSetIter;

        //加入/移除关键帧
        void addKeyFrame(PKeyFrame *pKF);
        void rmKeyFrame(PKeyFrame *pKF);
        //加入/移除地图点
        void addMapPoint(PMapPoint *pMp);
        void rmMapPoint(PMapPoint *pMp);
        //清空
        void clear();

        //获取所有地图点
        MapPtSet getAllMapPts()const
        {
            return mMapPts;
        }

        //获取所有帧
        KeyFmSet getAllFrames()const
        {
            return mMapFms;
        }
    protected:
        MapPtSet    mMapPts;
        KeyFmSet    mMapFms;
    };


   
}

#endif