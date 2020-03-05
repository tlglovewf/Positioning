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
        PMapPoint(const cv::Mat &pose, PMap *pMap);
        PMapPoint(const cv::Point3f &pt, PMap *pMap);
    public:
        friend class PMap;

        //获取位置(世界坐标)
        virtual const Mat getPose()const 
        {
            return mPose;
        }
        //设置位置(世界坐标)
        virtual void setPose(const cv::Mat &pose)
        {
            mPose = pose;
        }
        //获取序号
        virtual u64 index()const 
        {
            return mIndex;
        }
         //观察点
        virtual int observations()const 
        {
            return mObsers.size();
        }
        //添加观察者
        virtual void addObservation(IFrame *frame,int index) 
        {
            mObsers.insert(std::make_pair(frame,index));
        }
        //获取观察帧列表
        virtual const FrameMap& getObservation()const 
        {
            return mObsers;
        }
        //移除观察者
        virtual void rmObservation(IFrame *frame) 
        {
            mObsers.erase(frame);
        }
        //是否在帧中
        virtual bool isInFrame(IFrame *pFrame) 
        {
            return mObsers.find(pFrame) != mObsers.end();
        }
        //设置坏点
        virtual void setBadFlag() 
        {
            mbBad = true;
        }
        //返回是否为坏点
        virtual bool isBad() const
        {
            return mbBad;
        }
        
        //返回向量
        virtual const cv::Mat& normal()const 
        {
            return mNormal;
        }

        //尺度不变性最大距离
        virtual double maxDistance()const 
        {
            return mMaxDistance;
        }
        //尺度不变性最小距离
        virtual double minDistance()const 
        {
            return mMinDistance;
        }

        //重置静态参数
        static void resetStaticParams()
        {
            s_nIndexCount = 0;
        }
    protected:
        cv::Mat     mPose;
        cv::Mat     mNormal;
        PMap       *mpMap;
        u64         mIndex;

        double      mMinDistance;
        double      mMaxDistance;

        bool        mbBad;
        
        FrameMap    mObsers;

        static u64  s_nIndexCount;

    private:
        DISABLEDCP(PMapPoint)
    };
}

#endif