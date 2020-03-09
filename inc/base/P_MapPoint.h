/**
 *   P_MapPoint.h
 *   
 *   add by tu li gen   2020.2.19
 * 
 */

#ifndef __PMAPPOINT_H_H_
#define __PMAPPOINT_H_H_

#include "P_Interface.h"

namespace Position
{
    class PMap;
     //地图点
    class PMapPoint : public IMapPoint
    {
    protected:
         //构造
        PMapPoint(const cv::Mat &pose, PMap *pMap,u64 index);
        PMapPoint(const cv::Point3f &pt, PMap *pMap,u64 index);
    public:
        friend class PMap;

           //设置世界位姿
        virtual void setWorldPos(const cv::Mat &Pos) 
        {
            mPose = Pos;
        }
        //获取位姿
        virtual cv::Mat getWorldPos() 
        {
            return mPose;
        }
        //获取序号
        virtual u64 index()const 
        {
            return mIndex;
        }
         //观察点
        virtual int observations() 
        {
            return mObsers.size();
        }
        //添加观察者
        virtual void addObservation(IKeyFrame *frame,int index) 
        {
            mObsers.insert(std::make_pair(frame,index));
        }
        //获取观察帧列表
        virtual KeyFrameMap getObservations() 
        {
            return mObsers;
        }
        //移除观察者
        virtual void rmObservation(IKeyFrame *frame) 
        {
            mObsers.erase(frame);
        }
        //是否在帧中
        virtual bool isInFrame(IKeyFrame *pFrame) 
        {
            return mObsers.find(pFrame) != mObsers.end();
        }
        //设置坏点
        virtual void setBadFlag() 
        {
            mbBad = true;
        }
        //返回是否为坏点
        virtual bool isBad() 
        {
            return mbBad;
        }
        
        //返回向量
        virtual cv::Mat normal() 
        {
            return mNormal;
        }

    protected:
        cv::Mat     mPose;
        cv::Mat     mNormal;
        PMap       *mpMap;
        u64         mIndex;

        double      mMinDistance;
        double      mMaxDistance;

        bool        mbBad;
        
        KeyFrameMap    mObsers;

        std::mutex mMutexPos;
        std::mutex mMutexFeatures;
    private:
        DISABLEDCP(PMapPoint)
    };
}

#endif